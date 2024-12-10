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


# This module started from the proto_writer.py module from the
# AIPlan4EU project, with the same license

# type: ignore[valid-type]
import fractions
from itertools import product
from typing import Dict, List

from unified_planning import model
from unified_planning.engines import PlanGenerationResult
from unified_planning.exceptions import UPException
import unified_planning.model.htn
from unified_planning.model.operators import (
    BOOL_OPERATORS,
    IRA_OPERATORS,
    OperatorKind,
    RELATIONS,
)
from unified_planning.model.timing import TimepointKind
from unified_planning.model.types import domain_item, domain_size
from unified_planning.environment import get_environment
import unified_planning.model.walkers as walkers
from upf4ros2.converter import Converter, handles
# from upf4ros2.ros2_utils import print_expr

from upf_msgs import msg as msgs


def map_operator(op: int) -> str:
    if op == OperatorKind.PLUS:
        return 'up:plus'
    elif op == OperatorKind.MINUS:
        return 'up:minus'
    elif op == OperatorKind.TIMES:
        return 'up:times'
    elif op == OperatorKind.DIV:
        return 'up:div'
    elif op == OperatorKind.LE:
        return 'up:le'
    elif op == OperatorKind.LT:
        return 'up:lt'
    elif op == OperatorKind.EQUALS:
        return 'up:equals'
    elif op == OperatorKind.AND:
        return 'up:and'
    elif op == OperatorKind.OR:
        return 'up:or'
    elif op == OperatorKind.NOT:
        return 'up:not'
    elif op == OperatorKind.IMPLIES:
        return 'up:implies'
    elif op == OperatorKind.IFF:
        return 'up:iff'
    elif op == OperatorKind.EXISTS:
        return 'up:exists'
    elif op == OperatorKind.FORALL:
        return 'up:forall'
    raise ValueError(f'Unknown operator `{op}`')


def interface_type(tpe: model.Type) -> str:
    if tpe.is_bool_type():
        return 'up:bool'
    elif tpe.is_time_type():
        return 'up:time'
    elif tpe.is_int_type() or tpe.is_real_type():
        return f'up:{tpe}'
    elif isinstance(tpe, model.types._UserType):
        return str(tpe.name)


class FNode2ROS2(walkers.DagWalker):

    def __init__(self, ros2_writer):
        super().__init__()
        self._ros2_writer = ros2_writer

    def convert(self, expression: model.FNode) -> msgs.Expression:
        return self.walk(expression)

    def walk_bool_constant(
        self, expression: model.FNode, args: List[msgs.Expression]
    ) -> msgs.Expression:
        item = msgs.ExpressionItem()
        item.atom.append(msgs.Atom())
        item.atom[0].boolean_atom.append(expression.bool_constant_value())
        item.type = 'up:bool'
        item.kind = msgs.ExpressionItem.CONSTANT
        ret = msgs.Expression()
        ret.expressions.append(item)
        ret.level.append(0)
        return ret

    def walk_int_constant(
        self, expression: model.FNode, args: List[msgs.Expression]
    ) -> msgs.Expression:
        item = msgs.ExpressionItem()
        item.atom.append(msgs.Atom())
        item.atom[0].int_atom.append(expression.int_constant_value())
        item.type = 'up:integer'
        item.kind = msgs.ExpressionItem.CONSTANT
        ret = msgs.Expression()
        ret.expressions.append(item)
        ret.level.append(0)
        return ret

    def walk_real_constant(
        self, expression: model.FNode, args: List[msgs.Expression]
    ) -> msgs.Expression:
        item = msgs.ExpressionItem()
        real = msgs.Real()
        real.numerator = expression.real_constant_value().numerator
        real.denominator = expression.real_constant_value().denominator
        item.atom.append(msgs.Atom())
        item.atom[0].real_atom.append(real)
        item.type = 'up:real'
        item.kind = msgs.ExpressionItem.CONSTANT
        ret = msgs.Expression()
        ret.expressions.append(item)
        ret.level.append(0)
        return ret

    def walk_param_exp(
        self, expression: model.FNode, args: List[msgs.Expression]
    ) -> msgs.Expression:
        item = msgs.ExpressionItem()
        item.atom.append(msgs.Atom())
        item.atom[0].symbol_atom.append(expression.parameter().name)
        item.type = interface_type(expression.parameter().type)
        item.kind = msgs.ExpressionItem.PARAMETER
        ret = msgs.Expression()
        ret.expressions.append(item)
        ret.level.append(0)
        return ret

    def walk_variable_exp(
        self, expression: model.FNode, args: List[msgs.Expression]
    ) -> msgs.Expression:
        item = msgs.ExpressionItem()
        s_atom = msgs.Atom()
        s_atom.symbol_atom.append(expression.variable().name)
        item.atom.append(s_atom)
        item.type = interface_type(expression.variable().type)
        item.kind = msgs.ExpressionItem.VARIABLE
        ret = msgs.Expression()
        ret.expressions.append(item)
        ret.level.append(0)
        return ret

    def walk_object_exp(
        self, expression: model.FNode, args: List[msgs.Expression]
    ) -> msgs.Expression:
        item = msgs.ExpressionItem()
        item.atom.append(msgs.Atom())
        item.atom[0].symbol_atom.append(expression.object().name)
        item.type = interface_type(expression.object().type)
        item.kind = msgs.ExpressionItem.CONSTANT
        ret = msgs.Expression()
        ret.expressions.append(item)
        ret.level.append(0)
        return ret

    def walk_timing_exp(
        self, expression: model.FNode, args: List[msgs.Expression]
    ) -> msgs.Expression:
        timing = expression.timing()
        tp = timing.timepoint
        if timing.timepoint.container is not None:
            item = msgs.ExpressionItem()
            atom = msgs.Atom()
            atom.symbol_atom.append(timing.timepoint.container)
            item.atom.append(atom)
            item.type = 'up:container'
            item.kind = msgs.ExpressionItem.CONTAINER_ID
            expr = msgs.Expression()
            expr.expressions.append(item)
            expr.level.append(0)
            args = [expr]
        else:
            args = []

        if tp.kind == TimepointKind.GLOBAL_START:
            fn = 'up:global_start'
        elif tp.kind == TimepointKind.GLOBAL_END:
            fn = 'up:global_end'
        elif tp.kind == TimepointKind.START:
            fn = 'up:start'
        elif tp.kind == TimepointKind.END:
            fn = 'up:end'
        else:
            raise ValueError(f'Unknown timepoint kind: {tp.kind}')

        fn_exp_item = msgs.ExpressionItem()
        fn_atom = msgs.Atom()
        fn_atom.symbol_atom.append(fn)
        fn_exp_item.atom.append(fn_atom)
        fn_exp_item.kind = msgs.ExpressionItem.FUNCTION_SYMBOL

        tp_exp = msgs.Expression()
        tp_exp.expressions.append(msgs.ExpressionItem())
        tp_exp.level.append(0)

        tp_exp.expressions[0].kind = msgs.ExpressionItem.FUNCTION_APPLICATION
        tp_exp.expressions[0].type = 'up:time'

        tp_exp.expressions.append(fn_exp_item)
        tp_exp.level.append(1)

        (other_expr, other_levels) = self.increase_level_expressions(args, 1)

        tp_exp.expressions.extend(other_expr)
        tp_exp.level.extend(other_levels)

        assert timing.delay == 0
        return tp_exp

    def increase_level_expressions(
        self, expressions: List[msgs.Expression], base_level: int
    ) -> (List[msgs.ExpressionItem], List[int]):
        ret_expr = []
        ret_levels = []

        for expr in expressions:
            ret_expr.extend(expr.expressions)
            for level in expr.level:
                ret_levels.append(level + base_level)
        return (ret_expr, ret_levels)

    def walk_fluent_exp(
        self, expression: model.FNode, args: List[msgs.Expression]
    ) -> msgs.Expression:
        ret = msgs.Expression()
        item_root = msgs.ExpressionItem()
        item_root.type = interface_type(expression.fluent().type)
        item_root.kind = msgs.ExpressionItem.STATE_VARIABLE
        ret.expressions.append(item_root)
        ret.level.append(0)

        item = msgs.ExpressionItem()
        item.atom.append(msgs.Atom())
        item.atom[0].symbol_atom.append(expression.fluent().name)
        item.type = interface_type(expression.fluent().type)
        item.kind = msgs.ExpressionItem.FLUENT_SYMBOL
        ret.expressions.append(item)
        ret.level.append(1)

        (extended_expr, extended_levels) = self.increase_level_expressions(args, 1)

        ret.expressions.extend(extended_expr)
        ret.level.extend(extended_levels)

        return ret

    @walkers.handles(BOOL_OPERATORS.union(IRA_OPERATORS).union(RELATIONS))
    def walk_operator(
        self, expression: model.FNode, args: List[msgs.Expression]
    ) -> msgs.Expression:
        sub_list = []
        expr_item = msgs.ExpressionItem()
        expr_item.atom.append(msgs.Atom())
        expr_item.atom[0].symbol_atom.append(
            map_operator(expression.node_type))
        expr_item.kind = msgs.ExpressionItem.FUNCTION_SYMBOL
        expr_item.type = 'up:operator'
        sub_list.append(expr_item)
        # forall/exists: add the declared variables from the payload to
        # the beginning of the parameter list.
        other_expr = []
        other_levels = []
        if expression.is_exists() or expression.is_forall():
            list_prev = [self._ros2_writer.convert(
                p) for p in expression.variables()]
            (other_expr, other_levels) = self.increase_level_expressions(list_prev, 1)

        (args_expr, args_level) = self.increase_level_expressions(args, 1)

        ret = msgs.Expression()
        ret.expressions.append(msgs.ExpressionItem())
        ret.level.append(0)
        ret.expressions[0].kind = msgs.ExpressionItem.FUNCTION_APPLICATION
        ret.expressions[0].type = ''

        ret.expressions.append(expr_item)
        ret.level.append(1)

        ret.expressions.extend(other_expr)
        ret.level.extend(other_levels)

        ret.expressions.extend(args_expr)
        ret.level.extend(args_level)

        return ret


map_features = {
    'ACTION_BASED': msgs.Problem.ACTION_BASED,
    'HIERARCHICAL': msgs.Problem.HIERARCHICAL,
    'SCHEDULING': msgs.Problem.SCHEDULING,
    'SIMPLE_NUMERIC_PLANNING': msgs.Problem.SIMPLE_NUMERIC_PLANNING,
    'GENERAL_NUMERIC_PLANNING': msgs.Problem.GENERAL_NUMERIC_PLANNING,
    'CONTINUOUS_TIME': msgs.Problem.CONTINUOUS_TIME,
    'DISCRETE_TIME': msgs.Problem.DISCRETE_TIME,
    'INTERMEDIATE_CONDITIONS_AND_EFFECTS': msgs.Problem.INTERMEDIATE_CONDITIONS_AND_EFFECTS,
    'EXTERNAL_CONDITIONS_AND_EFFECTS': msgs.Problem.EXTERNAL_CONDITIONS_AND_EFFECTS,
    'TIMED_EFFECTS': msgs.Problem.TIMED_EFFECTS,
    'TIMED_GOALS': msgs.Problem.TIMED_GOALS,
    'DURATION_INEQUALITIES': msgs.Problem.DURATION_INEQUALITIES,
    'SELF_OVERLAPPING': msgs.Problem.SELF_OVERLAPPING,
    'STATIC_FLUENTS_IN_DURATIONS': msgs.Problem.STATIC_FLUENTS_IN_DURATIONS,
    'FLUENTS_IN_DURATIONS': msgs.Problem.FLUENTS_IN_DURATIONS,
    'REAL_TYPE_DURATIONS': msgs.Problem.REAL_TYPE_DURATIONS,
    'INT_TYPE_DURATIONS': msgs.Problem.INT_TYPE_DURATIONS,
    'CONTINUOUS_NUMBERS': msgs.Problem.CONTINUOUS_NUMBERS,
    'DISCRETE_NUMBERS': msgs.Problem.DISCRETE_NUMBERS,
    'BOUNDED_TYPES': msgs.Problem.BOUNDED_TYPES,
    'NEGATIVE_CONDITIONS': msgs.Problem.NEGATIVE_CONDITIONS,
    'DISJUNCTIVE_CONDITIONS': msgs.Problem.DISJUNCTIVE_CONDITIONS,
    'EQUALITIES': msgs.Problem.EQUALITIES,
    'EXISTENTIAL_CONDITIONS': msgs.Problem.EXISTENTIAL_CONDITIONS,
    'UNIVERSAL_CONDITIONS': msgs.Problem.UNIVERSAL_CONDITIONS,
    'CONDITIONAL_EFFECTS': msgs.Problem.CONDITIONAL_EFFECTS,
    'INCREASE_EFFECTS': msgs.Problem.INCREASE_EFFECTS,
    'DECREASE_EFFECTS': msgs.Problem.DECREASE_EFFECTS,
    'STATIC_FLUENTS_IN_BOOLEAN_ASSIGNMENTS': msgs.Problem.STATIC_FLUENTS_IN_BOOLEAN_ASSIGNMENTS,
    'STATIC_FLUENTS_IN_NUMERIC_ASSIGNMENTS': msgs.Problem.STATIC_FLUENTS_IN_NUMERIC_ASSIGNMENTS,
    'STATIC_FLUENTS_IN_OBJECT_ASSIGNMENTS': msgs.Problem.STATIC_FLUENTS_IN_OBJECT_ASSIGNMENTS,
    'FLUENTS_IN_BOOLEAN_ASSIGNMENTS': msgs.Problem.FLUENTS_IN_BOOLEAN_ASSIGNMENTS,
    'FLUENTS_IN_NUMERIC_ASSIGNMENTS': msgs.Problem.FLUENTS_IN_NUMERIC_ASSIGNMENTS,
    'FLUENTS_IN_OBJECT_ASSIGNMENTS': msgs.Problem.FLUENTS_IN_OBJECT_ASSIGNMENTS,
    'FORALL_EFFECTS': msgs.Problem.FORALL_EFFECTS,
    'FLAT_TYPING': msgs.Problem.FLAT_TYPING,
    'HIERARCHICAL_TYPING': msgs.Problem.HIERARCHICAL_TYPING,
    'NUMERIC_FLUENTS': msgs.Problem.NUMERIC_FLUENTS,
    'OBJECT_FLUENTS': msgs.Problem.OBJECT_FLUENTS,
    'INT_FLUENTS': msgs.Problem.INT_FLUENTS,
    'REAL_FLUENTS': msgs.Problem.REAL_FLUENTS,
    'BOOL_FLUENT_PARAMETERS': msgs.Problem.BOOL_FLUENT_PARAMETERS,
    'BOUNDED_INT_FLUENT_PARAMETERS': msgs.Problem.BOUNDED_INT_ACTION_PARAMETERS,
    'BOOL_ACTION_PARAMETERS': msgs.Problem.BOOL_ACTION_PARAMETERS,
    'BOUNDED_INT_ACTION_PARAMETERS': msgs.Problem.BOUNDED_INT_ACTION_PARAMETERS,
    'UNBOUNDED_INT_ACTION_PARAMETERS': msgs.Problem.UNBOUNDED_INT_ACTION_PARAMETERS,
    'REAL_ACTION_PARAMETERS': msgs.Problem.REAL_ACTION_PARAMETERS,
    'ACTIONS_COST': msgs.Problem.ACTIONS_COST,
    'FINAL_VALUE': msgs.Problem.FINAL_VALUE,
    'MAKESPAN': msgs.Problem.MAKESPAN,
    'PLAN_LENGTH': msgs.Problem.PLAN_LENGTH,
    'OVERSUBSCRIPTION': msgs.Problem.OVERSUBSCRIPTION,
    'TEMPORAL_OVERSUBSCRIPTION': msgs.Problem.TEMPORAL_OVERSUBSCRIPTION,
    'STATIC_FLUENTS_IN_ACTIONS_COST': msgs.Problem.STATIC_FLUENTS_IN_ACTIONS_COST,
    'FLUENTS_IN_ACTIONS_COST': msgs.Problem.FLUENTS_IN_ACTIONS_COST,
    'REAL_NUMBERS_IN_ACTIONS_COST': msgs.Problem.REAL_NUMBERS_IN_ACTIONS_COST,
    'INT_NUMBERS_IN_ACTIONS_COST': msgs.Problem.INT_NUMBERS_IN_ACTIONS_COST,
    'REAL_NUMBERS_IN_OVERSUBSCRIPTION': msgs.Problem.REAL_NUMBERS_IN_OVERSUBSCRIPTION,
    'INT_NUMBERS_IN_OVERSUBSCRIPTION': msgs.Problem.INT_NUMBERS_IN_OVERSUBSCRIPTION,
    'SIMULATED_EFFECTS': msgs.Problem.SIMULATED_EFFECTS,
    'TRAJECTORY_CONSTRAINTS': msgs.Problem.TRAJECTORY_CONSTRAINTS,
    'STATE_INVARIANTS': msgs.Problem.STATE_INVARIANTS,
    'METHOD_PRECONDITIONS': msgs.Problem.METHOD_PRECONDITIONS,
    'TASK_NETWORK_CONSTRAINTS': msgs.Problem.TASK_NETWORK_CONSTRAINTS,
    'INITIAL_TASK_NETWORK_VARIABLES': msgs.Problem.INITIAL_TASK_NETWORK_VARIABLES,
    'TASK_ORDER_TOTAL': msgs.Problem.TASK_ORDER_TOTAL,
    'TASK_ORDER_PARTIAL': msgs.Problem.TASK_ORDER_PARTIAL,
    'TASK_ORDER_TEMPORAL': msgs.Problem.TASK_ORDER_TEMPORAL,
    'UNDEFINED_INITIAL_NUMERIC': msgs.Problem.UNDEFINED_INITIAL_NUMERIC,
    'UNDEFINED_INITIAL_SYMBOLIC': msgs.Problem.UNDEFINED_INITIAL_SYMBOLIC}


def map_feature(feature: str) -> int:
    pb_feature = map_features[feature]
    if pb_feature is None:
        raise ValueError(f'Cannot convert feature to protobuf {feature}')
    return pb_feature


class ROS2InterfaceWriter(Converter):
    """Class to convert from unified_planning Problem instance to ROS 2 Interfaces."""

    def __init__(self):
        super().__init__()
        self._fnode2ros2 = FNode2ROS2(self)

    @handles(model.Fluent)
    def _convert_fluent(
        self, fluent: model.Fluent, problem: model.Problem
    ) -> msgs.Fluent:
        name = fluent.name
        sig = [self.convert(t) for t in fluent.signature]

        ret = msgs.Fluent()
        ret.name = name
        ret.value_type = interface_type(fluent.type)
        ret.parameters = sig
        if fluent in problem.fluents_defaults:
            ret.default_value.append(self.convert(
                problem.fluents_defaults[fluent]))
        return ret

    @handles(model.Object)
    def _convert_object(self, obj: model.Object) -> msgs.ObjectDeclaration:
        ret = msgs.ObjectDeclaration()
        ret.name = obj.name
        ret.type = interface_type(obj.type)
        return ret

    @handles(model.FNode)
    def _convert_fnode(self, exp: model.FNode) -> msgs.Expression:
        return self._fnode2ros2.convert(exp)

    @handles(model.types._BoolType)
    def _convert_bool_type(
            self,
            tpe: model.types._BoolType) -> msgs.TypeDeclaration:
        ret = msgs.TypeDeclaration()
        ret.type_name = interface_type(tpe)
        return ret

    @handles(model.types._UserType)
    def _convert_user_type(
            self,
            t: model.types._UserType) -> msgs.TypeDeclaration:
        ret = msgs.TypeDeclaration()
        ret.type_name = interface_type(t)
        ret.parent_type = '' if t.father is None else interface_type(t.father)
        return ret

    @handles(model.types._IntType)
    def _convert_integer_type(
            self,
            t: model.types._IntType) -> msgs.TypeDeclaration:
        ret = msgs.TypeDeclaration()
        ret.type_name = interface_type(t)
        return ret

    @handles(model.types._RealType)
    def _convert_real(self, t: model.types._RealType) -> msgs.TypeDeclaration:
        ret = msgs.TypeDeclaration()
        ret.type_name = interface_type(t)
        return ret

    @handles(model.Effect)
    def _convert_effect(self, effect: model.Effect) -> msgs.EffectExpression:
        kind = None
        if effect.is_assignment():
            kind = msgs.EffectExpression.ASSIGN
        elif effect.is_increase():
            kind = msgs.EffectExpression.INCREASE
        elif effect.is_decrease():
            kind = msgs.EffectExpression.DECREASE
        else:
            raise ValueError(f'Unsupported effect: {effect}')

        ret = msgs.EffectExpression()
        ret.forall = []
        if effect.forall:
            for v in effect.forall:
                variable = msgs.Variable()
                variable.name = v.name
                variable.type = self.convert(v.type)
                ret.forall.append(variable)

        ret.kind = kind

        ret.fluent = self.convert(effect.fluent)
        ret.value = self.convert(effect.value)
        ret.condition = self.convert(effect.condition)
        return ret

    @handles(model.InstantaneousAction)
    def _convert_instantaneous_action(
        self, a: model.InstantaneousAction
    ) -> msgs.Action:
        effects = []
        conditions = []

        for cond in a.preconditions:
            r2cond = msgs.Condition()
            r2cond.cond = self.convert(cond)
            conditions.append(r2cond)

        for eff in a.effects:
            r2eff = msgs.Effect()
            r2eff.effect = self.convert(eff)
            effects.append(r2eff)

        ret = msgs.Action()
        ret.name = a.name
        ret.parameters = [self.convert(p) for p in a.parameters]
        ret.conditions = conditions
        ret.effects = effects

        return ret

    @handles(model.DurativeAction)
    def _convert_durative_action(self, a: model.DurativeAction) -> msgs.Action:
        effects = []
        conditions = []

        for span, cond in a.conditions.items():
            span = self.convert(span)
            for c in cond:
                new_cond = msgs.Condition()
                new_cond.cond = self.convert(c)
                new_cond.span.append(span)

                conditions.append(new_cond)
        for ot, eff in a.effects.items():
            ot = self.convert(ot)
            for e in eff:
                new_eff = msgs.Effect()
                new_eff.effect = self.convert(e)
                new_eff.occurrence_time.append(ot)

                effects.append(new_eff)
        ret = msgs.Action()
        ret.name = a.name
        ret.parameters = [self.convert(p) for p in a.parameters]
        ret.duration.append(self.convert(a.duration))
        ret.conditions = conditions
        ret.effects = effects

        return ret

    @handles(model.timing.Timepoint)
    def _convert_timepoint(self, tp: model.timing.Timepoint) -> msgs.Timepoint:
        if tp.kind == TimepointKind.START:
            kind = msgs.Timepoint.START
        elif tp.kind == TimepointKind.END:
            kind = msgs.Timepoint.END
        elif tp.kind == TimepointKind.GLOBAL_START:
            kind = msgs.Timepoint.GLOBAL_START
        elif tp.kind == TimepointKind.GLOBAL_END:
            kind = msgs.Timepoint.GLOBAL_END

        ret = msgs.Timepoint()
        ret.kind = kind

        if tp.container is not None:
            ret.container_id = tp.container
        return ret

    @handles(model.Timing)
    def _convert_timing(self, timing: model.Timing) -> msgs.Timing:
        ret = msgs.Timing()
        ret.timepoint = self.convert(timing._timepoint)
        ret.delay.append(self.convert(fractions.Fraction(timing.delay)))
        return ret

    @handles(fractions.Fraction)
    def _convert_fraction(self, fraction: fractions.Fraction) -> msgs.Real:
        ret = msgs.Real()
        ret.numerator = fraction.numerator
        ret.denominator = fraction.denominator
        return ret

    @handles(model.timing.Interval)
    def _convert_interval(
            self,
            interval: model.timing.Interval) -> msgs.Interval:
        ret = msgs.Interval()

        ret.is_left_open = interval.is_left_open()
        ret.lower = self.convert(interval.lower())
        ret.is_right_open = interval.is_right_open()
        ret.upper = self.convert(interval.lower())
        return ret

    @handles(model.TimeInterval)
    def _convert_time_interval(
        self, interval: model.TimeInterval
    ) -> msgs.TimeInterval:
        ret = msgs.TimeInterval()
        ret.is_left_open = interval.is_left_open()
        ret.lower = self.convert(interval.lower)
        ret.is_right_open = interval.is_right_open()
        ret.upper = self.convert(interval.upper)
        return ret

    @handles(model.DurationInterval)
    def _convert_duration_interval(
        self, interval: model.DurationInterval
    ) -> msgs.Duration:
        ret = msgs.Duration()
        ret.controllable_in_bounds.is_left_open = interval.is_left_open()
        ret.controllable_in_bounds.lower = self.convert(interval.lower)
        ret.controllable_in_bounds.is_right_open = interval.is_right_open()
        ret.controllable_in_bounds.upper = self.convert(interval.upper)

        return ret

    @handles(model.htn.Task)
    def _convert_abstract_task(
        self, task: model.htn.Task
    ) -> msgs.AbstractTaskDeclaration:
        ret = msgs.AbstractTaskDeclaration()
        ret.name = task.name
        ret.parameters = [self.convert(p) for p in task.parameters]
        return ret

    @handles(model.htn.ParameterizedTask)
    def _convert_parameterized_task(
        self, task: model.htn.ParameterizedTask
    ) -> msgs.Task:
        parameters = []
        for p in task.parameters:
            expr = msgs.Expression()
            aux = msgs.ExpressionItem()
            aux.atom.append(msgs.Atom())
            aux.atom[0].symbol_atom.append(p.name)
            aux.kind = msgs.ExpressionItem.PARAMETER
            aux.type = interface_type(p.type)
            expr.expressions.append(aux)
            expr.level.append(0)
            parameters.append(expr)

        ret = msgs.Task()
        ret.id = ''
        ret.task_name = task.task.name
        ret.parameters = parameters
        return ret

    @handles(model.htn.Subtask)
    def _convert_subtask(self, subtask: model.htn.Subtask) -> msgs.Task:
        ret = msgs.Task()
        ret.id = subtask.identifier
        ret.task_name = subtask.task.name
        ret.parameters = [self.convert(p) for p in subtask.parameters]
        return ret

    @handles(model.htn.Method)
    def _convert_method(self, method: model.htn.Method) -> msgs.Method:
        ret = msgs.Method()
        ret.name = method.name
        ret.parameters = [self.convert(p) for p in method.parameters]
        ret.achieved_task = self.convert(method.achieved_task)
        ret.subtasks = [self.convert(st) for st in method.subtasks]
        ret.constraints = [self.convert(c) for c in method.constraints]

        for c in method.preconditions:
            cond = msgs.Condition()
            cond.cond = self.convert(c)
            ret.conditions.append(cond)
        return ret

    @handles(model.htn.TaskNetwork)
    def _convert_task_network(
            self,
            tn: model.htn.TaskNetwork) -> msgs.TaskNetwork:
        ret = msgs.TaskNetwork()
        ret.variables = [self.convert(v) for v in tn.variables]
        ret.subtasks = [self.convert(st) for st in tn.subtasks]
        ret.constraints = [self.convert(c) for c in tn.constraints]
        return ret

    def build_hierarchy(
        self, problem: model.htn.HierarchicalProblem
    ) -> msgs.Hierarchy:
        ret = msgs.Hierarchy()
        ret.initial_task_network = self.convert(problem.task_network)
        ret.abstract_tasks = [self.convert(t) for t in problem.tasks]
        ret.methods = [self.convert(m) for m in problem.methods]
        return ret

    @handles(unified_planning.plans.Schedule)
    def _convert_schedule(
            self, schedule: unified_planning.plans.Schedule) -> msgs.Schedule:

        ret = msgs.Schedule()

        ret.activities_name = [a.name for a in schedule.activities]

        assignments = []

        for var, val in schedule.assignment.items():
            e = msgs.Expression()
            item = msgs.ExpressionItem()

            if isinstance(var, model.Timepoint):
                if var.kind == TimepointKind.START:
                    var = f"{var.container}.start"
                elif var.kind == TimepointKind.END:
                    var = f"{var.container}.end"
                else:
                    raise ValueError(f"Invalid timepoint in assignment: {var}")
            else:
                assert isinstance(var, model.Parameter)
                var = var.name

            item.type = var
            (val,) = get_environment().expression_manager.auto_promote(val)
            item.atom = self.convert(val).expressions[0].atom
            e.expressions.append(item)
            assignments.append(e)

        ret.variable_assignments = assignments
        return ret

    @handles(model.Problem, model.htn.HierarchicalProblem)
    def _convert_problem(self, problem: model.Problem) -> msgs.Problem:

        goals = [msgs.Goal(goal=self.convert(g)) for g in problem.goals]
        for t_goal in problem.timed_goals:
            for g in problem.timed_goals[t_goal]:
                goal = msgs.Goal()
                goal.goal = self.convert(g)
                goal.timing.append(self.convert(t_goal))
                goals += [goal]

        problem_name = str(problem.name) if problem.name is not None else ''
        hierarchy = []
        if isinstance(problem, model.htn.HierarchicalProblem):
            hierarchy.append(self.build_hierarchy(problem))

        ret = msgs.Problem()
        ret.domain_name = problem_name + '_domain'
        ret.problem_name = problem_name
        ret.types = [self.convert(t) for t in problem.user_types]
        ret.fluents = [self.convert(f, problem) for f in problem.fluents]
        ret.objects = [self.convert(o) for o in problem.all_objects]
        ret.actions = [self.convert(a) for a in problem.actions]

        for (x, v) in problem.initial_values.items():
            assignment = msgs.Assignment()
            assignment.fluent = self.convert(x)
            assignment.value = self.convert(v)
            ret.initial_state.append(assignment)

        if len(problem.trajectory_constraints) > 0:
            ret.features.append(msgs.Problem.TRAJECTORY_CONSTRAINTS)
            for constraint in problem.trajectory_constraints:
                condition = self.convert(constraint)
                ret.trajectory_constraints.append(condition)

        ret.timed_effects = [
            msgs.TimedEffect(
                effect=[self.convert(e) for e in problem.timed_effects[key]],
                occurrence_time=self.convert(key)
            )
            for key in problem.timed_effects
        ]
        ret.goals = goals
        ret.features = [map_feature(feature)
                        for feature in problem.kind.features]
        ret.metrics = [self.convert(m) for m in problem.quality_metrics]
        ret.hierarchy = hierarchy
        return ret

    @handles(model.metrics.MinimizeActionCosts)
    def _convert_minimize_action_costs(
        self, metric: model.metrics.MinimizeActionCosts
    ) -> msgs.Metric:
        action_costs = {}
        for action, cost in metric.costs.items():
            action_costs[action.name] = self.convert(cost)

        ret = msgs.Metric()
        ret.kind = msgs.Metric.MINIMIZE_ACTION_COSTS
        ret.action_cost_names = [
            msgs.Action(
                name=key) for key in action_costs.keys()]
        ret.action_cost_expr = list(action_costs.values())
        if metric.default is not None:
            ret.default_action_cost.append(self.convert(metric.default))
        return ret

    @handles(model.metrics.MinimizeSequentialPlanLength)
    def _convert_minimize_sequential_plan_length(self, _) -> msgs.Metric:
        ret = msgs.Metric()
        ret.kind = msgs.Metric.MINIMIZE_SEQUENTIAL_PLAN_LENGTH
        return ret

    @handles(model.metrics.MinimizeMakespan)
    def _convert_minimize_makespan(self, _) -> msgs.Metric:
        ret = msgs.Metric()
        ret.kind = msgs.Metric.MINIMIZE_MAKESPAN
        return ret

    @handles(model.metrics.MinimizeExpressionOnFinalState)
    def _convert_minimize_expression_on_final_state(
        self, metric: model.metrics.MinimizeExpressionOnFinalState
    ) -> msgs.Metric:
        ret = msgs.Metric()
        ret.kind = msgs.Metric.MINIMIZE_EXPRESSION_ON_FINAL_STATE
        ret.expression = self.convert(metric.expression)
        return ret

    @handles(model.metrics.MaximizeExpressionOnFinalState)
    def _convert_maximize_expression_on_final_state(
        self, metric: model.metrics.MaximizeExpressionOnFinalState
    ) -> msgs.Metric:
        ret = msgs.Metric()
        ret.kind = msgs.Metric.MAXIMIZE_EXPRESSION_ON_FINAL_STATE
        ret.expression = self.convert(metric.expression)
        return ret

    @handles(model.metrics.Oversubscription)
    def _convert_oversubscription_metric(
        self, metric: model.metrics.Oversubscription
    ) -> msgs.Metric:
        goals = []
        for g, c in metric.goals.items():
            goal = msgs.GoalWithCost()
            goal.goal = self.convert(g)
            goal.cost = self.convert(fractions.Fraction(c))
            goals.append(goal)
        ret = msgs.Metric()
        ret.kind = msgs.Metric.OVERSUBSCRIPTION
        ret.goals = goals
        return ret

    @handles(model.Parameter)
    def _convert_action_parameter(self, p: model.Parameter) -> msgs.Parameter:
        ret = msgs.Parameter()
        ret.name = p.name
        ret.type = interface_type(p.type)
        return ret

    @handles(model.Variable)
    def _convert_expression_variable(
        self, variable: model.Variable
    ) -> msgs.Expression:
        ret = msgs.Expression()
        ret.expressions.append(msgs.ExpressionItem())
        ret.level.append(0)
        ret.expressions[0].atom.append(msgs.Atom())
        ret.expressions[0].atom[0].symbol_atom.append(variable.name)
        ret.expressions[0].kind = msgs.ExpressionItem.VARIABLE
        ret.expressions[0].type = interface_type(variable.type)
        return ret

    @handles(unified_planning.plans.ActionInstance)
    def _convert_action_instance(
            self,
            a: unified_planning.plans.ActionInstance,
            start_time=None,
            end_time=None) -> msgs.ActionInstance:
        parameters = []
        for param in a.actual_parameters:
            # The parameters are atoms
            parameters.append(self.convert(param).expressions[0].atom[0])
        ret = msgs.ActionInstance()
        ret.action_name = a.action.name
        ret.parameters = parameters
        if bool(start_time) and bool(end_time):
            ret.start_time = start_time
            ret.end_time = end_time
            ret.time_triggered = True
        elif bool(start_time):
            ret.start_time = start_time
            ret.time_triggered = True
        else:
            ret.time_triggered = False
        return ret

    @handles(str)
    def _convert_str_atom(self, s: str) -> msgs.Atom:
        ret = msgs.Atom()
        ret.symbol_atom = [s]
        return ret

    @handles(unified_planning.plans.SequentialPlan)
    def _convert_sequential_plan(
        self, plan: unified_planning.plans.SequentialPlan
    ) -> msgs.Plan:
        ret = msgs.Plan()
        ret.kind = 1
        ret.actions = [self.convert(a) for a in plan.actions]
        return ret

    @handles(unified_planning.plans.HierarchicalPlan)
    def _convert_hierarchical_plan(
        self, plan: unified_planning.plans.HierarchicalPlan
    ) -> msgs.Plan:
        ret = msgs.Plan()

        if isinstance(plan._flat_plan, unified_planning.plans.SequentialPlan):
            ret = self._convert_sequential_plan(plan._flat_plan)
        elif isinstance(plan._flat_plan, unified_planning.plans.TimeTriggeredPlan):
            ret = self._convert_time_triggered_plan(plan._flat_plan)
        else:
            raise UPException(f"Unknown plan: {type(plan._flat_plan)}")

        ret.kind = 6
        return ret

    @handles(unified_planning.plans.TimeTriggeredPlan)
    def _convert_time_triggered_plan(
        self, plan: unified_planning.plans.TimeTriggeredPlan
    ) -> msgs.Plan:
        action_instances = []

        for a in plan.timed_actions:
            start_time = self.convert(a[0]) if a[0] is not None else None
            end_time = self.convert(a[0] + a[2]) if a[2] is not None else None
            instance = self._convert_action_instance(
                a[1], start_time=start_time, end_time=end_time
            )
            action_instances.append(instance)
        ret = msgs.Plan()
        ret.kind = 2
        ret.actions = action_instances
        return ret

    @handles(PlanGenerationResult)
    def _convert_plan_generation_result(
        self, result: PlanGenerationResult
    ) -> msgs.PlanGenerationResult:
        log_messages = None
        if result.log_messages is not None:
            log_messages = [self.convert(log) for log in result.log_messages]

        ret = msgs.PlanGenerationResult()
        ret.status = self.convert(result.status)
        ret.plan = self.convert(result.plan)
        ret.engine_name = result.engine_name
        if bool(result.metrics):
            ret.metric_names = list(result.metrics.keys())
            ret.metric_values = list(result.metrics.values())
        if bool(log_messages):
            ret.log_messages = log_messages
        return ret

    @handles(unified_planning.engines.PlanGenerationResultStatus)
    def _convert_plan_generation_status(
        self, status: unified_planning.engines.PlanGenerationResultStatus
    ) -> int:
        if (
            status
            == unified_planning.engines.PlanGenerationResultStatus.SOLVED_SATISFICING
        ):
            return msgs.PlanGenerationResult.SOLVED_SATISFICING

        elif (
            status
            == unified_planning.engines.PlanGenerationResultStatus.SOLVED_OPTIMALLY
        ):
            return msgs.PlanGenerationResult.SOLVED_OPTIMALLY
        elif (
            status
            == unified_planning.engines.PlanGenerationResultStatus.UNSOLVABLE_PROVEN
        ):
            return msgs.PlanGenerationResult.UNSOLVABLE_PROVEN
        elif (
            status
            == unified_planning.engines.PlanGenerationResultStatus.UNSOLVABLE_INCOMPLETELY
        ):
            return msgs.PlanGenerationResult.UNSOLVABLE_INCOMPLETELY
        elif status == unified_planning.engines.PlanGenerationResultStatus.TIMEOUT:
            return msgs.PlanGenerationResult.TIMEOUT
        elif status == unified_planning.engines.PlanGenerationResultStatus.MEMOUT:
            return msgs.PlanGenerationResult.MEMOUT
        elif (
            status == unified_planning.engines.PlanGenerationResultStatus.INTERNAL_ERROR
        ):
            return msgs.PlanGenerationResult.INTERNAL_ERROR
        elif (
            status
            == unified_planning.engines.PlanGenerationResultStatus.UNSUPPORTED_PROBLEM
        ):

            return msgs.PlanGenerationResult.UNSUPPORTED_PROBLEM
        elif status == unified_planning.engines.PlanGenerationResultStatus.INTERMEDIATE:
            return msgs.PlanGenerationResult.INTERMEDIATE
        else:
            raise ValueError('Unknown status: {}'.format(status))

    @handles(unified_planning.engines.LogMessage)
    def _convert_log_messages(
        self, log: unified_planning.engines.LogMessage
    ) -> msgs.LogMessage:
        if log.level == unified_planning.engines.LogLevel.INFO:
            level = msgs.LogMessage.INFO
        elif log.level == unified_planning.engines.LogLevel.WARNING:
            level = msgs.LogMessage.WARNING
        elif log.level == unified_planning.engines.LogLevel.ERROR:
            level = msgs.LogMessage.ERROR
        elif log.level == unified_planning.engines.LogLevel.DEBUG:
            level = msgs.LogMessage.DEBUG
        else:
            raise UPException(f'Unknown log level: {log.level}')

        ret = msgs.LogMessage()
        ret.level = level
        ret.message = str(log.message)
        return ret

    @handles(unified_planning.engines.CompilerResult)
    def _convert_compiler_result(
        self, result: unified_planning.engines.CompilerResult
    ) -> msgs.CompilerResult:
        mymap: Dict[str, msgs.ActionInstance] = {}
        log_messages = result.log_messages
        if log_messages is None:
            log_messages = []
        if result.map_back_action_instance is not None:
            for compiled_action in result.problem.actions:
                type_list = [
                    param.type for param in compiled_action.parameters]
                if len(type_list) == 0:
                    ai = unified_planning.plans.ActionInstance(compiled_action)
                    mymap[str(ai)] = self.convert(
                        result.map_back_action_instance(ai))
                    continue
                ground_size = 1
                domain_sizes = []
                for t in type_list:
                    ds = domain_size(result.problem, t)
                    domain_sizes.append(ds)
                    ground_size *= ds
                items_list: List[List[model.FNode]] = []
                for size, mtype in zip(domain_sizes, type_list):
                    items_list.append(
                        [domain_item(result.problem, mtype, j) for j in range(size)]
                    )
                grounded_params_list = product(*items_list)
                for grounded_params in grounded_params_list:
                    ai = unified_planning.plans.ActionInstance(
                        compiled_action, tuple(grounded_params)
                    )
                    mymap[str(ai)] = self.convert(
                        result.map_back_action_instance(ai))
        ret = msgs.CompilerResult()
        ret.problem = self.convert(result.problem)
        ret.map_back_plan_keys = mymap.keys()
        ret.map_back_plan_values = list(mymap.values())
        ret.log_messages = [self.convert(log) for log in log_messages]
        ret.engine = result.engine_name
        return ret

    @handles(unified_planning.engines.ValidationResult)
    def _convert_validation_result(
        self, result: unified_planning.engines.ValidationResult
    ) -> msgs.ValidationResult:
        ret = msgs.ValidationResult()
        ret.status = self.convert(result.status)
        ret.log_messages = [self.convert(log) for log in result.log_messages]
        ret.engine = result.engine_name
        ret.metrics = [
            msgs.ValidationMetric(
                key=key,
                value=value) for key,
            value in result.metrics.items()]
        return ret

    @handles(unified_planning.engines.ValidationResultStatus)
    def _convert_validation_result_status(
        self, status: unified_planning.engines.ValidationResultStatus
    ) -> int:
        if status == unified_planning.engines.ValidationResultStatus.VALID:
            return msgs.ValidationResult.VALID
        elif status == unified_planning.engines.ValidationResultStatus.INVALID:
            return msgs.ValidationResult.INVALID
        else:
            raise UPException(f'Unknown result status: {status}')
