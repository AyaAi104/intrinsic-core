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

from intrinsic.solutions import deployments

solution = deployments.connect()

# End-to-end test expectation, not an authorization check.
# nosemgrep: bandit.B101
assert "ai.intrinsic.adder" in solution.skills.get_skill_ids()

executive = solution.executive
resources = solution.resources
skills = solution.skills.ai.intrinsic

cc_adder = skills.adder(calculator=resources.my_cc_calculator, x=3, y=5)
go_adder = skills.adder(calculator=resources.my_go_calculator, x=2, y=4)
py_adder = skills.adder(calculator=resources.my_py_calculator, x=1, y=6)

executive.run([cc_adder, go_adder, py_adder])

# End-to-end test expectation, not an authorization check.
# nosemgrep: bandit.B101
assert executive.get_value(cc_adder.result).sum == 8
# End-to-end test expectation, not an authorization check.
# nosemgrep: bandit.B101
assert executive.get_value(go_adder.result).sum == 6
# End-to-end test expectation, not an authorization check.
# nosemgrep: bandit.B101
assert executive.get_value(py_adder.result).sum == 7
