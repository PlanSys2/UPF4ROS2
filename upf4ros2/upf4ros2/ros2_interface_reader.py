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
# 

# This module started from the proto_reader.py module from the
# AIPlan4EU project, with the same license

# type: ignore[attr-defined]
# from functools import partial
import fractions
from typing import Tuple, Union, Optional
from typing import OrderedDict

from unified_planning import model
from unified_planning.exceptions import UPException
from unified_planning import Environment
from unified_planning.model import metrics
from unified_planning.model import (
    DurativeAction,
    Effect,
    InstantaneousAction,
    Fluent,
    Parameter,
    Problem,
    Variable,
)
from unified_planning.model.operators import OperatorKind
import unified_planning.plans
from upf4ros2.converter import Converter, handles
from unified_planning.model.effect import EffectKind

from upf_msgs import msg as msgs


def convert_type_str(s: str, problem: Problem) -> model.types.Type:
    if s == 'up:bool':
        return problem.env.type_manager.BoolType()
    elif s == 'up:integer':
        return problem.env.type_manager.IntType()
    elif 'up:integer[' in s:
        lb = int(s.split('[')[1].split(',')[0])
        ub = int(s.split(',')[1].split(']')[0])
        return problem.env.type_manager.IntType(lb, ub)
    elif s == 'up:real':
        return problem.env.type_manager.RealType()
    elif 'up:real[' in s:
        return problem.env.type_manager.RealType(
            lower_bound=fractions.Fraction(s.split('[')[1].split(',')[0]),
            upper_bound=fractions.Fraction(s.split(',')[1].split(']')[0]),
        )
    else:
        assert not s.startswith('up:'), f'Unhandled builtin type: {s}'
        return problem.user_type(s)


# The operators are based on SExpressions supported in PDDL.
def op_to_node_type(op: str) -> OperatorKind:
    if op == 'up:plus':
        return OperatorKind.PLUS
    elif op == 'up:minus':
        return OperatorKind.MINUS
    elif op == 'up:times':
        return OperatorKind.TIMES
    elif op == 'up:div':
        return OperatorKind.DIV
    elif op == 'up:equals':
        return OperatorKind.EQUALS
    elif op == 'up:le':
        return OperatorKind.LE
    elif op == 'up:lt':
        return OperatorKind.LT
    elif op == 'up:and':
        return OperatorKind.AND
    elif op == 'up:or':
        return OperatorKind.OR
    elif op == 'up:not':
        return OperatorKind.NOT
    elif op == 'up:exists':
        return OperatorKind.EXISTS
    elif op == 'up:forall':
        return OperatorKind.FORALL
    elif op == 'up:implies':
        return OperatorKind.IMPLIES
    elif op == 'up:iff':
        return OperatorKind.IFF

    raise ValueError(f'Unknown operator `{op}`')


class ROS2InterfaceReader(Converter):
    """Class to convert ROS 2 Interfaces representation to unified_planning Problem instance."""

    @handles(msgs.Parameter)
    def _convert_parameter(
        self, msg: msgs.Parameter, problem: Problem
    ) -> model.Parameter:
        # print("@handles(msgs.Parameter)")
        # print(msg)
        return model.Parameter(
            msg.name, convert_type_str(msg.type, problem), problem.env
        )

    @handles(msgs.Fluent)
    def _convert_fluent(self, msg: msgs.Fluent, problem: Problem) -> Fluent:
        value_type: model.types.Type = convert_type_str(msg.value_type, problem)
        # print("@handles(msgs.Fluent)")
        # print(msg)
        sig: list = []
        for p in msg.parameters:
            sig.append(self.convert(p, problem))
        fluent = model.Fluent(msg.name, value_type, sig, problem.env)
        return fluent

    @handles(msgs.ObjectDeclaration)
    def _convert_object(
        self, msg: msgs.ObjectDeclaration, problem: Problem
    ) -> model.Object:
        # print("@handles(msgs.ObjectDeclaration)")
        # print(msg)
        return model.Object(msg.name, convert_type_str(msg.type, problem))

    @handles(msgs.ExpressionItem)
    def _convert_expressionitem(
        self, msg: msgs.ExpressionItem, problem: Problem
    ) -> model.Expression:
        # print("@handles(msgs.ExpressionItem)")
        # print(msg)
        if msg.kind == msgs.ExpressionItem.CONSTANT:
            return self.convert(msg.atom[0], problem)

        elif msg.kind == msgs.ExpressionItem.PARAMETER:
            return problem.env.expression_manager.ParameterExp(
                param=Parameter(
                    msg.atom[0].symbol_atom[0],
                    convert_type_str(msg.type, problem), problem.env
                ),
            )
        elif msg.kind == msgs.ExpressionItem.VARIABLE:
            return problem.env.expression_manager.VariableExp(
                var=Variable(
                    msg.atom[0].symbol_atom[0],
                    convert_type_str(msg.atom[0].type, problem), problem.env
                ),
            )
        elif msg.kind == msgs.ExpressionItem.STATE_VARIABLE:
            args = []
            payload = None

            fluent = msg
            if fluent.kind == msgs.ExpressionItem.FLUENT_SYMBOL:
                payload = self.convert(fluent.atom[0], problem)

            # print([print(m) for m in msg.expressions])
            args.extend([self.convert(m, problem) for m in msg.expressions])
            if payload is not None:
                return problem.env.expression_manager.FluentExp(payload, tuple(args))
            else:
                raise UPException(f'Unable to form fluent expression {msg}')
        raise ValueError(f'Unknown expressionItem kind `{msg.kind}`')

    @handles(msgs.Expression)
    def _convert_expression(
        self, msg: msgs.Expression, problem: Problem
    ) -> model.Expression:
        current_index = 0
        current_expr = msg.expressions[current_index]
        current_level = msg.level[current_index]
        current_index += 1

        if current_expr.kind == msgs.ExpressionItem.CONSTANT:
            assert len(current_expr.atom) > 0
            return self.convert(current_expr.atom[0], problem)
        elif current_expr.kind == msgs.ExpressionItem.PARAMETER:
            return problem.env.expression_manager.ParameterExp(
                param = Parameter(
                    current_expr.atom[0].symbol_atom[0],
                    convert_type_str(current_expr.type, problem), problem.env
                ),
            )
        elif current_expr.kind == msgs.ExpressionItem.VARIABLE:
            return problem.env.expression_manager.VariableExp(
                var=Variable(
                    current_expr.atom[0].symbol_atom[0],
                    convert_type_str(current_expr.type, problem), problem.env
                ),
            )
        elif current_expr.kind == msgs.ExpressionItem.STATE_VARIABLE:
            args = []
            payload = None

            fluent = msg.expressions[current_index]
            fluent_level = msg.level[current_index]
            current_index += 1

            print(fluent)

            if fluent.kind == msgs.ExpressionItem.FLUENT_SYMBOL:
                payload = self.convert(fluent.atom[0], problem)
            
            while current_index < len(msg.level) and msg.level[current_index] >= fluent_level:
                args.append(self.convert(msg.expressions[current_index], problem))
                current_index += 1
            
            if payload is not None:
                print(payload)
                print(args)
                a = problem.env.expression_manager.FluentExp(payload, tuple(args))
                print (a)
                return a
            else:
                raise UPException(f"Unable to form fluent expression {msg}")
        
        raise ValueError(f"Unknown expression kind `{msg.kind}`")


#         elif msg.expressions[0].kind == msgs.ExpressionItem.STATE_VARIABLE:
#             args = []
#             payload = None
# 
#             fluent = msg.expressions.pop(0)
#             if fluent.kind == msgs.ExpressionItem.FLUENT_SYMBOL:
#                 payload = self.convert(fluent.atom[0], problem)
# 
#             print([print(m) for m in msg.expressions])
#             args.extend([self.convert(m, problem) for m in msg.expressions])
#             if payload is not None:
#                 return problem.env.expression_manager.FluentExp(payload, tuple(args))
#             else:
#                 raise UPException(f'Unable to form fluent expression {msg}')
#         elif (
#             msg.expressions[0].kind == msgs.ExpressionItem.FUNCTION_APPLICATION
#             and msg.expressions[0].type != 'up:time'
#         ):
#             node_type = None
#             args = []
#             payload = None
# 
#             symbol = msg.expressions.pop(0)
#             if symbol.kind == msgs.ExpressionItem.FUNCTION_SYMBOL:
#                 node_type = op_to_node_type(symbol.atom[0].symbol)
# 
#             if node_type in [OperatorKind.EXISTS, OperatorKind.FORALL]:
#                 variables = msg.expressions[:-1]
#                 quantified_expression = msg.expressions[-1]
#                 args.append(self.convert(quantified_expression, problem))
#                 payload = tuple(
#                     [self.convert(var, problem).variable() for var in variables]
#                 )
#             else:
#                 args.extend([self.convert(m, problem) for m in msg.expressions])
# 
#             assert node_type is not None
# 
#             return problem.env.expression_manager.create_node(
#                 node_type=node_type,
#                 args=tuple(args),
#                 payload=payload,
#             )
#         elif (
#             msg.expressions[0].kind == msgs.ExpressionItem.FUNCTION_APPLICATION
#             and msg.expressions[0].type == 'up:time'
#         ):
#             fn = msg.expressions[1].atom.symbol
#             if fn == 'up:start':
#                 kd = model.TimepointKind.START
#             elif fn == 'up:end':
#                 kd = model.TimepointKind.END
#             elif fn == 'up:global_start':
#                 kd = model.TimepointKind.GLOBAL_START
#             elif fn == 'up:global_end':
#                 kd = model.TimepointKind.GLOBAL_END
#             else:
#                 raise ValueError(f'Invalid temporal qualifier {fn}')
#             container = None
#             if len(msg.expressions) > 2:
#                 container = msg.list[2].atom.symbol
#             tp = model.timing.Timepoint(kd, container)
#             return problem.env.expression_manager.TimingExp(model.Timing(0, tp))
# 
#         raise ValueError(f'Unknown expression kind `{msg.kind}`')

    @handles(msgs.Atom)
    def _convert_atom(
        self, msg: msgs.Atom, problem: Problem
    ) -> Union[model.FNode, model.Fluent, model.Object]:
        # print("@handles(msgs.Atom)")
        # print(msg)
        # # print(msg)
        if len(msg.int_atom) > 0:
            return problem.env.expression_manager.Int(msg.int_atom[0])
        elif len(msg.real_atom) > 0:
            return problem.env.expression_manager.Real(
                fractions.Fraction(
                    msg.real_atom[0].numerator, msg.real_atom[0].denominator)
            )
        elif len(msg.boolean_atom) > 0:
            return problem.env.expression_manager.Bool(msg.boolean_atom[0])
        elif len(msg.symbol_atom) > 0:
            # If atom symbols, return the equivalent UP alternative
            # Note that parameters are directly handled at expression level
            if problem.has_object(msg.symbol_atom[0]):
                return problem.env.expression_manager.ObjectExp(
                    obj=problem.object(msg.symbol_atom[0])
                )
            else:
                return problem.fluent(msg.symbol_atom[0])
        else:
            raise ValueError('Atom empty')

    @handles(msgs.TypeDeclaration)
    def _convert_type_declaration(
        self, msg: msgs.TypeDeclaration, problem: Problem
    ) -> model.Type:
        # print("@handles(msgs.TypeDeclaration)")
        # print(msg)
        if msg.type_name == 'up:bool':
            return problem.env.type_manager.BoolType()
        elif msg.type_name.startswith('up:integer['):
            tmp = msg.type_name.split('[')[1].split(']')[0].split(', ')
            return problem.env.type_manager.IntType(
                lower_bound=int(tmp[0]) if tmp[0] != '-inf' else None,
                upper_bound=int(tmp[1]) if tmp[1] != 'inf' else None,
            )
        elif msg.type_name.startswith('up:real['):
            tmp = msg.type_name.split('[')[1].split(']')[0].split(', ')
            lower_bound = fractions.Fraction(tmp[0]) if tmp[0] != '-inf' else None
            upper_bound = fractions.Fraction(tmp[1]) if tmp[1] != 'inf' else None
            return problem.env.type_manager.RealType(
                lower_bound=lower_bound, upper_bound=upper_bound
            )
        else:
            father = (
                problem.user_type(msg.parent_type) if msg.parent_type != '' else None
            )
            return problem.env.type_manager.UserType(name=msg.type_name, father=father)

    @handles(msgs.Problem)
    def _convert_problem(
        self, msg: msgs.Problem, env: Optional[Environment] = None
    ) -> Problem:
        # print("@handles(msgs.Problem)")
        # print("=========================================================================1")
        # print(msg)
        # print("=========================================================================1")
        problem_name = str(msg.problem_name) if str(msg.problem_name) != '' else None
        if len(msg.hierarchy) > 0:
            problem = model.htn.HierarchicalProblem(name=problem_name, env=env)
        else:
            problem = Problem(name=problem_name, env=env)

        # print("=========================================================================2")
        # print(msg)
        # print("=========================================================================2")
        for t in msg.types:
            problem._add_user_type(self.convert(t, problem))
        # print("=========================================================================3")
        # print(msg)
        # print("=========================================================================3")
        for obj in msg.objects:
            problem.add_object(self.convert(obj, problem))
        # print("=========================================================================4")
        # print(msg)
        # print("=========================================================================4")
        for f in msg.fluents:
            # print("=========================================================================4.1")
            # print(msg)
            # print("=========================================================================4.1")
            # print("=========================================================================4.5")
            # print(f)
            # print("=========================================================================4.5")
            problem.add_fluent(
                self.convert(f, problem),
                default_initial_value=self.convert(f.default_value[0], problem)
                if len(f.default_value) > 0
                else None,
            )
        # print("=========================================================================5")
        # print(msg)
        # print("=========================================================================5")
        for f in msg.actions:
            print("===============================")
            print(f)
            print("===============================")
            problem.add_action(self.convert(f, problem))
        for eff in msg.timed_effects:
            ot = self.convert(eff.occurrence_time, problem)
            effect = self.convert(eff.effect, problem)
            problem.add_timed_effect(
                timing=ot,
                fluent=effect.fluent,
                value=effect.value,
                condition=effect.condition,
            )

        for assign in msg.initial_state:
            problem.set_initial_value(
                fluent=self.convert(assign.fluent, problem),
                value=self.convert(assign.value, problem),
            )

        for g in msg.goals:
            goal = self.convert(g.goal, problem)
            if len(g.timing) == 0:
                problem.add_goal(goal)
            else:
                timing = self.convert(g.timing[0])
                problem.add_timed_goal(interval=timing, goal=goal)

        for metric in msg.metrics:
            problem.add_quality_metric(self.convert(metric, problem))

        if len(msg.hierarchy) > 0:
            for task in msg.hierarchy[0].abstract_tasks:
                problem.add_task(self.convert(task, problem))
            for method in msg.hierarchy[0].methods:
                problem.add_method(self.convert(method, problem))
            problem._initial_task_network = self.convert(
                msg.hierarchy[0].initial_task_network, problem
            )

        return problem
 
    @handles(msgs.AbstractTaskDeclaration)
    def _convert_abstract_task(
        self, msg: msgs.AbstractTaskDeclaration, problem: Problem
    ):
        return model.htn.Task(
            msg.name, [self.convert(p, problem) for p in msg.parameters], problem.env
        )

    @handles(msgs.Task)
    def _convert_task(
        self, msg: msgs.Task, problem: model.htn.HierarchicalProblem
    ) -> model.htn.Subtask:
        if problem.has_task(msg.task_name):
            task = problem.get_task(msg.task_name)
        elif problem.has_action(msg.task_name):
            task = problem.action(msg.task_name)
        else:
            raise ValueError(f'Unknown task name: {msg.task_name}')
        parameters = [self.convert(p, problem) for p in msg.parameters]
        return model.htn.Subtask(task, *parameters, ident=msg.id, _env=problem.env)

    @handles(msgs.Method)
    def _convert_method(
        self, msg: msgs.Method, problem: model.htn.HierarchicalProblem
    ) -> model.htn.Method:
        method = model.htn.Method(
            msg.name,
            [self.convert(p, problem) for p in msg.parameters],
            problem.env,
        )
        achieved_task_params = []
        for p in msg.achieved_task.parameters:
            achieved_task_params.append(method.parameter(p.atom.symbol))
        method.set_task(
            problem.get_task(msg.achieved_task.task_name), *achieved_task_params
        )
        for st in msg.subtasks:
            method.add_subtask(self.convert(st, problem))
        for c in msg.constraints:
            method.add_constraint(self.convert(c, problem))
        for c in msg.conditions:
            assert not c.HasField('span'), 'Timed conditions are currently unsupported.'
            method.add_precondition(self.convert(c.cond, problem))
        return method

    @handles(msgs.TaskNetwork)
    def _convert_task_network(
        self, msg: msgs.TaskNetwork, problem: model.htn.HierarchicalProblem
    ) -> model.htn.TaskNetwork:
        tn = model.htn.TaskNetwork(problem.env)
        for v in msg.variables:
            tn.add_variable(v.name, convert_type_str(v.type, problem))
        for st in msg.subtasks:
            tn.add_subtask(self.convert(st, problem))
        for c in msg.constraints:
            tn.add_constraint(self.convert(c, problem))

        return tn

    @handles(msgs.Metric)
    def _convert_metric(
        self, msg: msgs.Metric, problem: Problem
    ) -> Union[
        metrics.MinimizeActionCosts,
        metrics.MinimizeSequentialPlanLength,
        metrics.MinimizeMakespan,
        metrics.MinimizeExpressionOnFinalState,
        metrics.MaximizeExpressionOnFinalState,
        metrics.Oversubscription,
    ]:
        if msg.kind == msgs.Metric.MINIMIZE_ACTION_COSTS:
            costs = {}
            for a, cost in msg.action_costs.items():
                costs[problem.action(a)] = self.convert(cost, problem)
            return metrics.MinimizeActionCosts(
                costs=costs,
                default=self.convert(msg.default_action_cost[0], problem)
                if len(msg.default_action_cost) > 0
                else None,
            )

        elif msg.kind == msgs.Metric.MINIMIZE_SEQUENTIAL_PLAN_LENGTH:
            return metrics.MinimizeSequentialPlanLength()

        elif msg.kind == msgs.Metric.MINIMIZE_MAKESPAN:
            return metrics.MinimizeMakespan()

        elif msg.kind == msgs.Metric.MINIMIZE_EXPRESSION_ON_FINAL_STATE:
            return metrics.MinimizeExpressionOnFinalState(
                expression=self.convert(msg.expression, problem)
            )

        elif msg.kind == msgs.Metric.MAXIMIZE_EXPRESSION_ON_FINAL_STATE:
            return metrics.MaximizeExpressionOnFinalState(
                expression=self.convert(msg.expression, problem)
            )
        elif msg.kind == msgs.Metric.OVERSUBSCRIPTION:
            goals = {}
            for g in msg.goals:
                goals[self.convert(g.goal, problem)] = self.convert(g.cost)
            return metrics.Oversubscription(goals)
        else:
            raise UPException(f'Unknown metric kind `{msg.kind}`')

    @handles(msgs.Action)
    def _convert_action(self, msg: msgs.Action, problem: Problem) -> model.Action:
        # print("@handles(msgs.Action)")
        # print("=================================================================1")
        # print(msg)
        # print("=================================================================1")
        action: model.Action

        parameters = OrderedDict()
        for param in msg.parameters:
            parameters[param.name] = convert_type_str(param.type, problem)

        if len(msg.duration) > 0:
            action = DurativeAction(msg.name, parameters)
            action.set_duration_constraint(self.convert(msg.duration[0], problem))
        else:
            action = InstantaneousAction(msg.name, parameters)

        conditions = []
        for condition in msg.conditions:
            print("++++++++++++++++++++++++++++++++")
            print(condition)
            print("++++++++++++++++++++++++++++++++")
            cond = self.convert(condition.cond, problem)
            span = self.convert(condition.span[0]) if len(condition.span) > 0 else None
            conditions.append((cond, span))

        effects = []
        for effect in msg.effects:
            eff = self.convert(effect.effect, problem)
            time = (
                self.convert(effect.occurrence_time)
                if len(effect.occurrence_time) > 0
                else None
            )
            effects.append((eff, time))

        if isinstance(action, DurativeAction):
            for c, span in conditions:
                action.add_condition(span, c)
            for e, ot in effects:
                if e.kind == EffectKind.ASSIGN:
                    action.add_effect(ot, e.fluent, e.value, e.condition)
                elif e.kind == EffectKind.DECREASE:
                    action.add_decrease_effect(ot, e.fluent, e.value, e.condition)
                elif e.kind == EffectKind.INCREASE:
                    action.add_increase_effect(ot, e.fluent, e.value, e.condition)
        elif isinstance(action, InstantaneousAction):
            for c, _ in conditions:
                action.add_precondition(c)
            for e, _ in effects:
                if e.kind == EffectKind.ASSIGN:
                    action.add_effect(e.fluent, e.value, e.condition)
                elif e.kind == EffectKind.DECREASE:
                    action.add_decrease_effect(e.fluent, e.value, e.condition)
                elif e.kind == EffectKind.INCREASE:
                    action.add_increase_effect(e.fluent, e.value, e.condition)

        return action

    @handles(msgs.EffectExpression)
    def _convert_effect(
        self, msg: msgs.EffectExpression, problem: Problem
    ) -> model.Effect:
        # EffectKind
        if msg.kind == msgs.EffectExpression.INCREASE:
            kind = EffectKind.INCREASE
        elif msg.kind == msgs.EffectExpression.DECREASE:
            kind = EffectKind.DECREASE
        else:
            kind = EffectKind.ASSIGN


        fluent=self.convert(msg.fluent, problem)

        msg.condition.expressions[0].atom[0].symbol_atom.append("hola")
        # print("+++++++++++++++++++++")
        # print(msg.condition)
        # print("+++++++++++++++++++++")
        # print("--------------------")
        # print(msg.value)
        # print("--------------------")
        condition=self.convert(msg.condition, problem)
        value=self.convert(msg.value, problem)


        return Effect(
            fluent=fluent,
            value=value,
            condition=condition,
            kind=kind,
        )

    @handles(msgs.Duration)
    def _convert_duration(
        self, msg: msgs.Duration, problem: Problem
    ) -> model.timing.DurationInterval:
        # print("@handles(msgs.Duration)")
        # print(msg)
        return model.timing.DurationInterval(
            lower=self.convert(msg.controllable_in_bounds.lower, problem),
            upper=self.convert(msg.controllable_in_bounds.upper, problem),
            is_left_open=bool(msg.controllable_in_bounds.is_left_open),
            is_right_open=bool(msg.controllable_in_bounds.is_right_open),
        )

    @handles(msgs.TimeInterval)
    def _convert_timed_interval(self, msg: msgs.TimeInterval) -> model.TimeInterval:
        # print("@handles(msgs.TimeInterval)")
        # print(msg)
        return model.TimeInterval(
            lower=self.convert(msg.lower),
            upper=self.convert(msg.upper),
            is_left_open=msg.is_left_open,
            is_right_open=msg.is_right_open,
        )

    @handles(msgs.Timing)
    def _convert_timing(self, msg: msgs.Timing) -> model.timing.Timing:
        # print("@handles(msgs.Timing)")
        # print(msg)
        return model.Timing(
            delay=self.convert(msg.delay)
            if msg.HasField('delay')
            else fractions.Fraction(0),
            timepoint=self.convert(msg.timepoint),
        )

    @handles(msgs.Real)
    def _convert_real(self, msg: msgs.Real) -> fractions.Fraction:
        # print("@handles(msgs.Real)")
        # print(msg)
        return fractions.Fraction(msg.numerator, msg.denominator)

    @handles(msgs.Timepoint)
    def _convert_timepoint(self, msg: msgs.Timepoint) -> model.timing.Timepoint:
        # print("@handles(msgs.Timepoint)")
        # print(msg)
        if msg.kind == msgs.Timepoint.GLOBAL_START:
            kind = model.timing.TimepointKind.GLOBAL_START
        elif msg.kind == msgs.Timepoint.GLOBAL_END:
            kind = model.timing.TimepointKind.GLOBAL_END
        elif msg.kind == msgs.Timepoint.START:
            kind = model.timing.TimepointKind.START
        elif msg.kind == msgs.Timepoint.END:
            kind = model.timing.TimepointKind.END
        else:
            raise UPException('Unknown timepoint kind: {}'.format(msg.kind))
        container = msg.container_id if msg.container_id != '' else None
        return model.timing.Timepoint(kind, container)

    @handles(msgs.Plan)
    def _convert_plan(
        self, msg: msgs.Plan, problem: Problem
    ) -> unified_planning.plans.Plan:
        # print("@handles(msgs.Plan)")
        # print(msg)
        actions = [self.convert(a, problem) for a in msg.actions]
        if all(isinstance(a, tuple) for a in actions):
            return unified_planning.plans.TimeTriggeredPlan(actions)
        else:
            return unified_planning.plans.SequentialPlan(actions=actions)

    @handles(msgs.ActionInstance)
    def _convert_action_instance(
        self, msg: msgs.ActionInstance, problem: Problem
    ) -> Union[
        Tuple[
            model.timing.Timing,
            unified_planning.plans.ActionInstance,
            model.timing.Duration,
        ],
        unified_planning.plans.ActionInstance,
    ]:
        # print("@handles(msgs.ActionInstance)")
        # print(msg)
        # action instance parameters are atoms but in UP they are FNodes
        # converting to up.model.FNode
        parameters = tuple([self.convert(param, problem) for param in msg.parameters])

        action_instance = unified_planning.plans.ActionInstance(
            problem.action(msg.action_name),
            parameters,
        )

        start_time = (
            self.convert(msg.start_time) if msg.time_triggered else None
        )
        end_time = self.convert(msg.end_time) if msg.time_triggered else None
        if start_time is not None:
            return (
                start_time,  # Absolute Start Time
                action_instance,
                end_time - start_time if end_time else None,  # Duration
            )
        else:
            return action_instance

    @handles(msgs.PlanGenerationResult)
    def _convert_plan_generation_result(
        self, result: msgs.PlanGenerationResult, problem: Problem
    ) -> unified_planning.engines.PlanGenerationResult:
        # print("@handles(msgs.PlanGenerationResult)")
        # print(msg)
        if result.status == msgs.PlanGenerationResult.SOLVED_SATISFICING:
            status = (
                unified_planning.engines.results.PlanGenerationResultStatus.SOLVED_SATISFICING
            )
        elif result.status == msgs.PlanGenerationResult.SOLVED_OPTIMALLY:
            status = (
                unified_planning.engines.results.PlanGenerationResultStatus.SOLVED_OPTIMALLY
            )
        elif result.status == msgs.PlanGenerationResult.UNSOLVABLE_PROVEN:
            status = (
                unified_planning.engines.results.PlanGenerationResultStatus.UNSOLVABLE_PROVEN
            )
        elif result.status == msgs.PlanGenerationResult.UNSOLVABLE_INCOMPLETELY:
            status = (
                unified_planning.engines.results.PlanGenerationResultStatus.UNSOLVABLE_INCOMPLETELY
            )
        elif result.status == msgs.PlanGenerationResult.TIMEOUT:
            status = unified_planning.engines.results.PlanGenerationResultStatus.TIMEOUT
        elif result.status == msgs.PlanGenerationResult.MEMOUT:
            status = unified_planning.engines.results.PlanGenerationResultStatus.MEMOUT
        elif result.status == msgs.PlanGenerationResult.INTERNAL_ERROR:
            status = (
                unified_planning.engines.results.PlanGenerationResultStatus.INTERNAL_ERROR
            )
        elif result.status == msgs.PlanGenerationResult.UNSUPPORTED_PROBLEM:
            status = (
                unified_planning.engines.results.PlanGenerationResultStatus.UNSUPPORTED_PROBLEM
            )
        else:
            raise UPException(f'Unknown Planner Status: {result.status}')

        log_messages = None
        metrics = None

        if len(result.metric_names) > 0:
            metrics = dict(zip(result.metric_names, result.metric_values))

        if len(result.log_messages) > 0:
            log_messages = [self.convert(log) for log in result.log_messages]

        return unified_planning.engines.PlanGenerationResult(
            status=status,
            plan=self.convert(result.plan, problem),
            engine_name=result.engine_name,
            metrics=metrics,
            log_messages=log_messages,
        )

    @handles(msgs.LogMessage)
    def _convert_log_message(
        self, log: msgs.LogMessage
    ) -> unified_planning.engines.LogMessage:
        # print("@handles(msgs.LogMessage)")
        # print(msg)
        if log.level == msgs.LogMessage.INFO:
            return unified_planning.engines.LogMessage(
                level=unified_planning.engines.LogLevel.INFO,
                message=log.message,
            )
        elif log.level == msgs.LogMessage.WARNING:
            return unified_planning.engines.LogMessage(
                level=unified_planning.engines.LogLevel.WARNING,
                message=log.message,
            )
        elif log.level == msgs.LogMessage.ERROR:
            return unified_planning.engines.LogMessage(
                level=unified_planning.engines.LogLevel.ERROR,
                message=log.message,
            )
        elif log.level == msgs.LogMessage.DEBUG:
            return unified_planning.engines.LogMessage(
                level=unified_planning.engines.LogLevel.DEBUG,
                message=log.message,
            )
        else:
            raise UPException(f'Unexpected Log Level: {log.level}')

    @handles(msgs.CompilerResult)
    def _convert_compiler_result(
        self,
        result: msgs.CompilerResult,
        lifted_problem: unified_planning.model.Problem,
    ) -> unified_planning.engines.CompilerResult:
        problem = self.convert(result.problem, lifted_problem.env)
        map: Dict[
            unified_planning.model.Action,
            Tuple[unified_planning.model.Action, List[unified_planning.model.FNode]],
        ] = {}
        for grounded_action in problem.actions:
            map_back_plan = dict(zip(
                    result.map_back_plan_keys, rresult.map_back_plan_keys_values))
            original_action_instance = self.convert(
                map_back_plan[grounded_action.name], lifted_problem
            )
            map[grounded_action] = (
                original_action_instance.action,
                original_action_instance.actual_parameters,
            )
        return unified_planning.engines.CompilerResult(
            problem=problem,
            map_back_action_instance=partial(
                unified_planning.engines.compilers.utils.lift_action_instance, map=map
            ),
            engine_name=result.engine,
            log_messages=[self.convert(log) for log in result.log_messages],
        )

    @handles(msgs.ValidationResult)
    def _convert_validation_result(
        self, result: msgs.ValidationResult
    ) -> unified_planning.engines.ValidationResult:
        if result.status == msgs.ValidationResult.VALID:
            r_status = unified_planning.engines.ValidationResultStatus.VALID
        elif result.status == msgs.ValidationResult.INVALID:
            r_status = unified_planning.engines.ValidationResultStatus.INVALID
        else:
            raise UPException(f'Unexpected ValidationResult status: {result.status}')
        return unified_planning.engines.ValidationResult(
            status=r_status,
            engine_name=result.engine,
            log_messages=[self.convert(log) for log in result.log_messages],
        )
