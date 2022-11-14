# Copyright 2022 Intelligent Robotics Lab
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.


from upf_msgs import msg as msgs

kind_to_string = {
    msgs.ExpressionItem.UNKNOWN: 'UNKNOWN',
    msgs.ExpressionItem.CONSTANT: 'CONSTANT',
    msgs.ExpressionItem.PARAMETER: 'PARAMETER',
    msgs.ExpressionItem.VARIABLE: 'VARIABLE',
    msgs.ExpressionItem.FLUENT_SYMBOL: 'FLUENT_SYMBOL',
    msgs.ExpressionItem.FUNCTION_SYMBOL: 'FUNCTION_SYMBOL',
    msgs.ExpressionItem.STATE_VARIABLE: 'STATE_VARIABLE',
    msgs.ExpressionItem.FUNCTION_APPLICATION: 'FUNCTION_APPLICATION',
    msgs.ExpressionItem.CONTAINER_ID: 'CONTAINER_ID',
}


def print_expr(expr: msgs.Expression):
    for i in range(len(expr.expressions)):
        print(
            f'{expr.level[i] * "    "} '
            f'[{kind_to_string[expr.expressions[i].kind]}] {expr.expressions[i]}')
