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

func TestExplicitTokenExchangeAddressBypassesEnvironmentFile(t *testing.T) {
	t.Setenv("INTRINSIC_ENV_FILE", filepath.Join(t.TempDir(), "missing.env"))
	t.Setenv("INTRINSIC_TOKEN_EXCHANGE_SERVER", "environment.example")
	for _, fromEnvironment := range []bool{true, false} {
		if !fromEnvironment {
			if err := os.Unsetenv("INTRINSIC_TOKEN_EXCHANGE_SERVER"); err != nil {
				t.Fatal(err)
			}
		}
		for _, address := range []string{"explicit.example", ""} {
			source, err := (&ProjectToken{}).AsIDTokenCredentials(
				WithTokenExchangeServiceAddress("earlier.example"),
				WithTokenExchangeServiceAddress(address),
			)
			if err != nil {
				t.Fatal(err)
			}
			client, ok := source.cts.tp.(*TokensServiceClient)
			if !ok {
				t.Fatal("unexpected token provider")
			}
			if client.addr != address {
				t.Fatalf("address = %q; want %q", client.addr, address)
			}
		}
	}
}
