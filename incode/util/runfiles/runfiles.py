# Copyright 2026 Intrinsic Innovation LLC
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     https://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""Generic resolver that will attempt to resolve runfiles or external directories if available."""

import os

from python.runfiles import runfiles

Runfiles = runfiles.Runfiles
Create = runfiles.Create


def _get_runfiles_root(r: runfiles.Runfiles, frame: int = 1):
  """Get the root directory of the runfiles repository."""
  frame += 1  # ignore this function in call stack
  root = r.CurrentRepository(frame)
  if root == "":
    root = "_main"
  return root


def RlocationCurrentRepository(
    path: str, r: runfiles.Runfiles | None = None, frame: int = 1
):
  if r is None:
    r = runfiles.Create()
  if not (r is not None):
    raise AssertionError()
  frame += 1  # ignore this function in call stack
  return r.Rlocation(os.path.join(_get_runfiles_root(r, frame), path))
