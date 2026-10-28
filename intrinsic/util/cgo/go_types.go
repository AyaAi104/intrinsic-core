// Copyright 2026 Intrinsic Innovation LLC
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     https://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

// Package gotypes provides type conversion between native Go and C wrapper types.
package gotypes

/*
#include <limits.h>
#include <stddef.h>
#include <stdint.h>
#include <sys/types.h>

#if SIZE_MAX > UINT64_MAX
#error size_t larger than largest Go uint type
#endif

#include "intrinsic/util/cgo/c_types.h"

// Typedefs to work around https://golang.org/issue/19835:
typedef void (*write_func)(go_c_Handle status, const char*, size_t);
typedef void (*go_c_UnmarshalProtoFunc)(go_c_Handle, const char*, size_t);

// Forward declarations to work around https://github.com/golang/go/issues/19837:
extern void go_c_ReturnStatus(go_c_Handle, char*, size_t);
extern void go_c_UnmarshalProto(go_c_Handle, char*, size_t);
*/
import "C"

import (
	"fmt"
	"reflect"
	"sync"
	"unsafe"

	"intrinsic/util/cgo/handle"

	log "github.com/golang/glog"
	"google.golang.org/grpc/status"
	"google.golang.org/protobuf/proto"

	spb "google.golang.org/genproto/googleapis/rpc/status"
)

// Numeric limit constants.
// cgo miscompiles large unsigned constants to signed constants.
// For now, assume that C allows the full range of the types.
const (
	maxSize  = ^C.size_t(0) // https://golang.org/issue/20369
	maxGoInt = int(^uint(0) >> 1)
	minGoInt = ^maxGoInt
)

// MustSizeOrPanic returns n as a C size_t, or panics if the conversion overflows SIZE_MAX.
func MustSizeOrPanic(n int) C.size_t {
	s, ok := C.size_t(n), n >= 0 && uint64(n) <= uint64(maxSize)
	if !ok {
		panic(fmt.Sprintf("%d out of size_t range [0, %d]", n, maxSize))
	}
	return s
}

// CSize is an alias for C.size_t.
type CSize C.size_t

// toInt returns s as a Go int, or false if the conversion overflows.
func (s CSize) toInt() (int, bool) {
	return int(s), uint64(s) <= uint64(maxGoInt)
}

// MustIntOrPanic returns s as a Go int, or panics if the conversion overflows.
func MustIntOrPanic(s CSize) int {
	n, ok := s.toInt()
	if !ok {
		panic(fmt.Sprintf("%d out of Go int range [%d, %d]", n, minGoInt, maxGoInt))
	}
	return n
}

func bytesIn(in []byte) (*C.char, C.size_t) {
	return (*C.char)(unsafe.Pointer(unsafe.SliceData(in))), C.size_t(MustSizeOrPanic(len(in)))
}

func asByteSlice(data *C.char, length C.size_t) []byte {
	return unsafe.Slice((*byte)(unsafe.Pointer(data)), MustIntOrPanic(CSize(length)))
}

// AsBytes returns a byte slice representing the specified data from C++.
func AsBytes(ptr unsafe.Pointer, length CSize) []byte {
	return unsafe.Slice((*byte)(ptr), MustIntOrPanic(length))
}

// CopyStruct takes an input src containing one of the struct types defined in c.h,
// along with the a pointer to the expected struct type, and copies the struct
// to the pointed-to location. This is necessary to cast from struct types to
// themselves in a way that works across Go package boundaries (see
// https://golang.org/issue/13467). This will panic if the given types obviously
// do not match.
func CopyStruct(dst any, src any) {
	v := reflect.ValueOf(src)
	pv := reflect.ValueOf(dst)
	if err := isSame(types{v.Type(), pv.Type().Elem()}); err != nil {
		panic(err)
	}
	reflect.NewAt(v.Type(), unsafe.Pointer(pv.Pointer())).Elem().Set(v)
}

// types represents a pair of equivalent reflect.Types.
type types struct{ a, b reflect.Type }

// types -> error
var eqTypes sync.Map

// IsSame checks whether the two given struct types match (i.e. have the same fields in the same
// order).
func IsSame(a, b reflect.Type) error {
	return isSame(types{a, b})
}

func isSame(t types) error {
	v, ok := eqTypes.Load(t)
	if !ok {
		v, _ = eqTypes.LoadOrStore(t, compareTypes(t))
	}
	if v != nil {
		return v.(error)
	}
	return nil
}

func compareTypes(t types) error {
	a := t.a
	b := t.b
	// Check the structs, field by field.
	if a.Kind() != reflect.Struct {
		return fmt.Errorf("assertSame: expected struct ptr, got %v", a)
	}
	if b.Kind() != reflect.Struct {
		return fmt.Errorf("assertSame: expected struct ptr, got %v", b)
	}
	// The copy below writes the source struct's entire memory representation.
	// Matching field kinds alone does not bound nested structs or arrays.
	if a.Size() != b.Size() || a.Align() != b.Align() {
		return fmt.Errorf("assertSame: struct layout mismatch (%v and %v)", a, b)
	}
	if a.NumField() != b.NumField() {
		return fmt.Errorf("assertSame: NumField mismatch (%v and %v)", a, b)
	}
	for i := 0; i < a.NumField(); i++ {
		f, g := a.Field(i), b.Field(i)
		switch {
		case f.Name != g.Name:
			return fmt.Errorf("assertSame: field %d name mismatch (%s vs %s)", i, f.Name, g.Name)
		case f.Offset != g.Offset:
			return fmt.Errorf("assertSame: field %d offset mismatch (%d vs %d)", i, f.Offset, g.Offset)
		case f.Type.Kind() != g.Type.Kind():
			return fmt.Errorf("assertSame: field %d kind mismatch (%v vs %v)", i, f.Type.Kind(), g.Type.Kind())
		case f.Type.Size() != g.Type.Size() || f.Type.Align() != g.Type.Align():
			return fmt.Errorf("assertSame: field %d layout mismatch (%v vs %v)", i, f.Type, g.Type)
		}
	}
	return nil
}

//export go_c_UnmarshalProto
func go_c_UnmarshalProto(h C.go_c_Handle, data *C.char, length C.size_t) {
	val, err := handle.Value(handle.H(h))
	if err != nil {
		log.Fatalf("handle.Value(h) returned error: %v", err)
	}
	msg := val.(proto.Message)
	opts := proto.UnmarshalOptions{AllowPartial: true}
	if err := opts.Unmarshal(asByteSlice(data, length), msg); err != nil {
		panic(err)
	}
}

//export go_c_ReturnStatus
func go_c_ReturnStatus(h C.go_c_Handle, p *C.char, pLen C.size_t) {
	val, err := handle.Value(handle.H(h))
	if err != nil {
		log.Fatalf("handle.Value(h) returned error: %v", err)
	}

	errc := val.(chan error)
	size := MustIntOrPanic(CSize(pLen))
	okStatusLen := 0
	if size == okStatusLen {
		errc <- nil
		return
	}

	v := unsafe.Slice((*byte)(unsafe.Pointer(p)), size)
	statusProto := &spb.Status{}
	if err := proto.Unmarshal(v, statusProto); err != nil {
		log.Fatalf("failed to unmarshal proto: %v", err)
	}
	errc <- status.FromProto(statusProto).Err()
}

// StatusOut returns a reflect.Value that will be passed to a C++ function that will populate the
// error channel.
func StatusOut(inType reflect.Type, h handle.H) reflect.Value {
	statusOut := reflect.New(inType)
	CopyStruct(statusOut.Interface(), C.go_c_StatusOut{
		status:      C.go_c_Handle(h),
		write_func:  C.write_func(C.go_c_ReturnStatus),
		write_if_ok: true,
	})
	return reflect.Indirect(statusOut)
}

// StringIn returns a C.go_c_StringIn containing the given string.
func StringIn(s string) C.go_c_StringIn {
	return C.go_c_StringIn{
		data:   (*C.uint8_t)(unsafe.StringData(s)),
		length: C.size_t(MustSizeOrPanic(len(s))),
	}
}

// ProtoIn returns a C.go_c_ProtoIn that will serialize the given proto message.
func ProtoIn(msg proto.Message) C.go_c_ProtoIn {
	pb, err := proto.MarshalOptions{AllowPartial: true}.Marshal(msg)
	if err != nil {
		panic(err)
	}
	var res C.go_c_ProtoIn
	res.data, res.length = bytesIn(pb)
	res.name_data, res.name_length = bytesIn([]byte(string(proto.MessageName(msg))))
	return res
}

// ProtoOut returns a C.go_c_ProtoOut that will return the given proto message corresponding
// to the given Go handle.
// The returned value is meant to be passed to a C++ function that will populate the proto message.
func ProtoOut(msg proto.Message, h handle.H) C.go_c_ProtoOut {
	res := C.go_c_ProtoOut{
		message:        C.go_c_Handle(h),
		unmarshal_func: C.go_c_UnmarshalProtoFunc(C.go_c_UnmarshalProto),
	}
	res.name_data, res.name_length = bytesIn([]byte(string(proto.MessageName(msg))))
	return res
}

// TranslateToBasicCType translates Go `val` to the C output type given by `cval`.
//
// Supports: bool, int{8,16,32,64}, uint{8,16,32,64}, float{32,64}, and unsafe.Pointer.
func TranslateToBasicCType(val reflect.Value, cval any) error {
	rval := reflect.ValueOf(cval)
	if rval.Kind() != reflect.Ptr {
		return fmt.Errorf("cval is not a pointer")
	}

	elem := rval.Elem()
	if elem.Kind() != val.Kind() {
		return fmt.Errorf("output kind mismatch: %v vs %v (type of provided Go parameter vs return type of C function)",
			val.Kind(), elem.Kind())
	}
	switch val.Kind() {
	case reflect.Bool, reflect.Int8, reflect.Int16, reflect.Int32, reflect.Int64,
		reflect.Uint8, reflect.Uint16, reflect.Uint32, reflect.Uint64,
		reflect.Float32, reflect.Float64, reflect.UnsafePointer:

		elem.Set(val.Convert(elem.Type()))
	case reflect.Int, reflect.Uint:
		// Go's int and uint don't correspond directly to a C++ type.
		return fmt.Errorf("unsupported platform-dependent type %v", val.Type())
	default:
		return fmt.Errorf("unknown type (%v vs %v)", val.Type(), reflect.TypeOf(cval))
	}
	return nil
}
