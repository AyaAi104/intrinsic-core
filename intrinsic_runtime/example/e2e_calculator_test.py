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

solution = deployments.connect(address="localhost:17080")

# End-to-end test expectation, not an authorization check.
if not ('ai.intrinsic.adder' in solution.skills.get_skill_ids()):
  raise AssertionError()

executive = solution.executive
resources = solution.resources
skills = solution.skills.ai.intrinsic

cc_adder1 = skills.adder(calculator=resources.calculator_service, x=3, y=5)
cc_adder2 = skills.adder(calculator=resources.calculator_service, x=2, y=4)
cc_adder3 = skills.adder(calculator=resources.calculator_service, x=1, y=6)

executive.run([cc_adder1, cc_adder2, cc_adder3])

# End-to-end test expectation, not an authorization check.
if not (executive.get_value(cc_adder1.result).sum == 8):
  raise AssertionError()
# End-to-end test expectation, not an authorization check.
if not (executive.get_value(cc_adder2.result).sum == 6):
  raise AssertionError()
# End-to-end test expectation, not an authorization check.
if not (executive.get_value(cc_adder3.result).sum == 7):
  raise AssertionError()
