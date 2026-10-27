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
	_ "embed"
	"errors"
	"fmt"
	"os"

	"github.com/joho/godotenv"
)

// Packaged defaults keep installed binaries independent of their working directory.
//
//go:embed defaults.env
var defaultAuthEnvironment string

// tokenExchangeAddress resolves configuration without changing process variables.
// Explicit caller options are handled before this function is called.
func tokenExchangeAddress() (string, error) {
	const key = "INTRINSIC_TOKEN_EXCHANGE_SERVER"
	if address, ok := os.LookupEnv(key); ok {
		return address, nil
	}

	filename, explicit := os.LookupEnv("INTRINSIC_ENV_FILE")
	if !explicit {
		filename = ".env"
	}
	values, err := godotenv.Read(filename)
	if err != nil && (explicit || !errors.Is(err, os.ErrNotExist)) {
		return "", fmt.Errorf("read auth environment file: %w", err)
	}
	if address, ok := values[key]; ok {
		return address, nil
	}

	defaults, err := godotenv.Unmarshal(defaultAuthEnvironment)
	if err != nil {
		return "", fmt.Errorf("read packaged auth environment: %w", err)
	}
	address, ok := defaults[key]
	if !ok {
		return "", fmt.Errorf("packaged auth environment is missing %s", key)
	}
	return address, nil
}
