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

package auth

import (
	"os"
	"path/filepath"
	"testing"
)

func TestTokenExchangeAddress(t *testing.T) {
	const endpointKey = "INTRINSIC_TOKEN_EXCHANGE_SERVER"
	const fileKey = "INTRINSIC_ENV_FILE"
	cases := []struct {
		name         string
		contents     string
		writeFile    bool
		explicitFile bool
		environment  *string
		want         string
		wantError    bool
	}{
		{name: "missing optional file preserves default", want: "flowstate.intrinsic.ai"},
		{name: "dotenv", contents: endpointKey + "=local.example", writeFile: true, want: "local.example"},
		{name: "export and quotes", contents: "export " + endpointKey + "='quoted.example'", writeFile: true, want: "quoted.example"},
		{name: "unrelated settings preserve default", contents: "UNRELATED=value", writeFile: true, want: "flowstate.intrinsic.ai"},
		{name: "explicit file", contents: endpointKey + "=explicit.example", writeFile: true, explicitFile: true, want: "explicit.example"},
		{name: "missing explicit file fails", explicitFile: true, wantError: true},
		{name: "malformed file fails", contents: "INTRINSIC_TOKEN_EXCHANGE_SERVER=\"unterminated", writeFile: true, wantError: true},
		{name: "environment wins over file", contents: endpointKey + "=file.example", writeFile: true, environment: stringPointer("environment.example"), want: "environment.example"},
		{name: "environment bypasses missing explicit file", explicitFile: true, environment: stringPointer("environment.example"), want: "environment.example"},
		{name: "environment bypasses malformed file", contents: "INTRINSIC_TOKEN_EXCHANGE_SERVER=\"unterminated", writeFile: true, environment: stringPointer("environment.example"), want: "environment.example"},
		{name: "explicit empty environment is preserved", environment: stringPointer(""), want: ""},
		{name: "empty file value is preserved", contents: endpointKey + "=", writeFile: true, want: ""},
	}
	for _, tc := range cases {
		t.Run(tc.name, func(t *testing.T) {
			t.Chdir(t.TempDir())
			t.Setenv(endpointKey, "")
			t.Setenv(fileKey, "")
			if err := os.Unsetenv(endpointKey); err != nil {
				t.Fatal(err)
			}
			if err := os.Unsetenv(fileKey); err != nil {
				t.Fatal(err)
			}
			filename := ".env"
			if tc.explicitFile {
				filename = filepath.Join(t.TempDir(), "selected.env")
				t.Setenv(fileKey, filename)
			}
			if tc.writeFile {
				if err := os.WriteFile(filename, []byte(tc.contents), 0600); err != nil {
					t.Fatal(err)
				}
			}
			if tc.environment != nil {
				t.Setenv(endpointKey, *tc.environment)
			}
			before := os.Environ()
			got, err := tokenExchangeAddress()
			if (err != nil) != tc.wantError {
				t.Fatalf("error = %v; wantError = %v", err, tc.wantError)
			}
			if got != tc.want {
				t.Errorf("address = %q; want %q", got, tc.want)
			}
			after := os.Environ()
			if len(before) != len(after) {
				t.Fatal("configuration changed process environment")
			}
			for i := range before {
				if before[i] != after[i] {
					t.Fatal("configuration changed process environment")
				}
			}
		})
	}
}

func stringPointer(value string) *string { return &value }
