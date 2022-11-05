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

# This module started from the proto_writer.py module from the 
# AIPlan4EU project, with the same license

# type: ignore[valid-type]
import fractions
from itertools import product
from typing import List

from unified_planning import model
import unified_planning.model.htn
import unified_planning.model.walkers as walkers
from unified_planning.model.types import domain_size, domain_item
from unified_planning.exceptions import UPException
from unified_planning.engines import PlanGenerationResult
from upf4ros2.converter import Converter, handles
from unified_planning.model.operators import (
    BOOL_OPERATORS,
    IRA_OPERATORS,
    RELATIONS,
    OperatorKind,
)
from unified_planning.model.timing import TimepointKind

from upf_msgs.msg import *


def map_operator(op: int) -> str:
    if op == OperatorKind.PLUS:
        return "up:plus"
    elif op == OperatorKind.MINUS:
        return "up:minus"
    elif op == OperatorKind.TIMES:
        return "up:times"
    elif op == OperatorKind.DIV:
        return "up:div"
    elif op == OperatorKind.LE:
        return "up:le"
    elif op == OperatorKind.LT:
        return "up:lt"
    elif op == OperatorKind.EQUALS:
        return "up:equals"
    elif op == OperatorKind.AND:
        return "up:and"
    elif op == OperatorKind.OR:
        return "up:or"
    elif op == OperatorKind.NOT:
        return "up:not"
    elif op == OperatorKind.IMPLIES:
        return "up:implies"
    elif op == OperatorKind.IFF:
        return "up:iff"
    elif op == OperatorKind.EXISTS:
        return "up:exists"
    elif op == OperatorKind.FORALL:
        return "up:forall"
    raise ValueError(f"Unknown operator `{op}`")


def interface_type(tpe: model.Type) -> str:
    if tpe.is_bool_type():
        return "up:bool"
    elif tpe.is_time_type():
        return "up:time"
    elif tpe.is_int_type() or tpe.is_real_type():
        return f"up:{tpe}"
    elif isinstance(tpe, model.types._UserType):
        return str(tpe.name)


class FNode2ROS2(walkers.DagWalker):
    def __init__(self, ros2_writer):
        super().__init__()
        self._ros2_writer = ros2_writer

    def convert(self, expression: model.FNode) -> Expression:
        return self.walk(expression)

    def walk_bool_constant(
        self, expression: model.FNode, args: List[Expression]
    ) -> Expression:
        print('walk_bool_constant')
        return Expression()
        # return Expression(
        #     atom=proto.Atom(boolean=expression.bool_constant_value()),
        #     list=[],
        #     kind=proto.ExpressionKind.Value("CONSTANT"),
        #     type="up:bool",
        # )

    def walk_int_constant(
        self, expression: model.FNode, args: List[Expression]
    ) -> Expression:
        print('walk_int_constant')
        return Expression()
        # return Expression(
        #    atom=proto.Atom(int=expression.int_constant_value()),
        #    list=[],
        #    kind=proto.ExpressionKind.Value("CONSTANT"),
        #    type="up:integer",
        # 

    def walk_real_constant(
        self, expression: model.FNode, args: List[Expression]
    ) -> Expression:
        print('walk_real_constant')
        item = ExpressionItem()
        real = Real()
        real.numerator = expression.real_constant_value().numerator
        real.denominator = expression.real_constant_value().denominator
        item.atom.real_atom.append(real)
        item.type = "up:real"
        item.kind = ExpressionItem.CONSTANT
        ret = Expression()
        ret.expressions.append(item)
        ret.level.append(0)
        return ret

    def walk_param_exp(
        self, expression: model.FNode, args: List[Expression]
    ) -> Expression:
        item = ExpressionItem()
        item.atom.symbol_atom.append(expression.parameter().name)
        item.type = interface_type(expression.parameter().type)
        item.kind = ExpressionItem.PARAMETER
        ret = Expression()
        ret.expressions.append(item)
        ret.level.append(0)
        return ret

    def walk_variable_exp(
        self, expression: model.FNode, args: List[Expression]
    ) -> Expression:
        print('walk_variable_exp')
        return Expression()
        # return Expression(
        #     atom=proto.Atom(symbol=expression.variable().name),
        #     list=[],
        #     kind=proto.ExpressionKind.Value("VARIABLE"),
        #     type=proto_type(expression.variable().type),
        # )

    def walk_object_exp(
        self, expression: model.FNode, args: List[Expression]
    ) -> Expression:
        item = ExpressionItem()
        item.atom.symbol_atom.append(expression.object().name)
        item.type = interface_type(expression.object().type)
        item.kind = ExpressionItem.CONSTANT
        ret = Expression()
        ret.expressions.append(item)
        ret.level.append(0)
        return ret

    def walk_timing_exp(
        self, expression: model.FNode, args: List[Expression]
    ) -> Expression:
        print('walk_object_exp')
        return Expression()
        # timing = expression.timing()
        # tp = timing.timepoint
        # if timing.timepoint.container is not None:
        #     args = [
        #         Expression(
        #             atom=proto.Atom(symbol=timing.timepoint.container),
        #             type="up:container",
        #             kind=proto.ExpressionKind.Value("CONTAINER_ID"),
        #         )
        #     ]
        # else:
        #     args = []
        # if tp.kind == TimepointKind.GLOBAL_START:
        #     fn = "up:global_start"
        # elif tp.kind == TimepointKind.GLOBAL_END:
        #     fn = "up:global_end"
        # elif tp.kind == TimepointKind.START:
        #     fn = "up:start"
        # elif tp.kind == TimepointKind.END:
        #     fn = "up:end"
        # else:
        #     raise ValueError(f"Unknown timepoint kind: {tp.kind}")
        # fn_exp = Expression(
        #     atom=proto.Atom(symbol=fn),
        #     kind=proto.ExpressionKind.Value("FUNCTION_SYMBOL"),
        # )
        # tp_exp = Expression(
        #     list=[fn_exp] + args,
        #     type="up:time",
        #     kind=proto.ExpressionKind.Value("FUNCTION_APPLICATION"),
        # )
        # assert timing.delay == 0
        # return tp_exp

    def walk_fluent_exp(
        self, expression: model.FNode, args: List[Expression]
    ) -> Expression:
        print('walk_fluent_exp')
        return Expression()
        # sub_list = []
        # sub_list.append(
        #     Expression(
        #         atom=proto.Atom(symbol=expression.fluent().name),
        #         kind=proto.ExpressionKind.Value("FLUENT_SYMBOL"),
        #         type=proto_type(expression.fluent().type),
        #     )
        # )
        # sub_list.extend(args)
        # return Expression(
        #     atom=None,
        #     list=sub_list,
        #     kind=proto.ExpressionKind.Value("STATE_VARIABLE"),
        #     type=proto_type(expression.fluent().type),
        # )

    @walkers.handles(BOOL_OPERATORS.union(IRA_OPERATORS).union(RELATIONS))
    def walk_operator(
        self, expression: model.FNode, args: List[Expression]
    ) -> Expression:
        print('walk_operator')
        return Expression()
        #sub_list = []
        #sub_list.append(
        #    Expression(
        #        atom=proto.Atom(symbol=map_operator(expression.node_type)),
        #        list=[],
        #        kind=proto.ExpressionKind.Value("FUNCTION_SYMBOL"),
        #        type="up:operator",
        #    )
        #)
        ## forall/exists: add the declared variables from the payload to the beginning of the parameter list.
        #if expression.is_exists() or expression.is_forall():
        #    sub_list.extend(
        #        [self._protobuf_writer.convert(p) for p in expression.variables()]
        #    )
        #
        #sub_list.extend(args)
        #return Expression(
        #    atom=None,
        #    list=sub_list,
        #    kind=proto.ExpressionKind.Value("FUNCTION_APPLICATION"),
        #    type="",
        #)


# def map_feature(feature: str) -> Feature:
#     pb_feature = proto.Feature.Value(feature)
#     if pb_feature is None:
#         raise ValueError(f"Cannot convert feature to protobuf {feature}")
#     return pb_feature


class ROS2InterfaceWriter(Converter):
    """
    ROS2InterfaceWriter: This class uses the convert method to take a unified_planning Problem instance
    and return the equivalent ROS 2 Interfaces representation.
    """

    def __init__(self):
        super().__init__()
        self._fnode2ros2 = FNode2ROS2(self)

    @handles(model.Fluent)
    def _convert_fluent(
        self, fluent: model.Fluent, problem: model.Problem
    ) -> Fluent:
        name = fluent.name
        sig = [self.convert(t) for t in fluent.signature]

        ret = Fluent()
        ret.name = name
        ret.value_type=interface_type(fluent.type)
        ret.parameters = sig
        ret.default_value = self.convert(problem.fluents_defaults[fluent]) if fluent in problem.fluents_defaults else Expression()
        return ret
 
    @handles(model.Object)
    def _convert_object(self, obj: model.Object) -> ObjectDeclaration:
        ret = ObjectDeclaration()
        ret.name = obj.name
        ret.type = interface_type(obj.type)
        return ret

    @handles(model.FNode)
    def _convert_fnode(self, exp: model.FNode) -> Expression:
        return self._fnode2ros2.convert(exp)
 
    @handles(model.types._BoolType)
    def _convert_bool_type(self, tpe: model.types._BoolType) -> TypeDeclaration:
        ret = TypeDeclaration()
        ret.type_name = interface_type(tpe)
        return ret

    @handles(model.types._UserType)
    def _convert_user_type(self, t: model.types._UserType) -> TypeDeclaration:
        ret = TypeDeclaration()
        ret.type_name = interface_type(t)
        ret.parent_type="" if t.father is None else interface_type(t.father)
        return ret

    @handles(model.types._IntType)
    def _convert_integer_type(self, t: model.types._IntType) -> TypeDeclaration:
        ret = TypeDeclaration()
        ret.type_name = interface_type(t)
        return ret

    @handles(model.types._RealType)
    def _convert_real(self, t: model.types._RealType) -> TypeDeclaration:
        ret = TypeDeclaration()
        ret.type_name = interface_type(t)     
        return ret

    @handles(model.Effect)
    def _convert_effect(self, effect: model.Effect) -> Effect:
        kind = None
        if effect.is_assignment():
            kind = EffectExpression.ASSIGN
        elif effect.is_increase():
            kind = EffectExpression.INCREASE
        elif effect.is_decrease():
            kind = EffectExpression.DECREASE
        else:
            raise ValueError(f"Unsupported effect: {effect}")

        ret = Effect()
        ret.effect.kind = kind
        ret.effect.fluent = self.convert(effect.fluent)
        ret.effect.value = self.convert(effect.value)
        ret.effect.condition = self.convert(effect.condition)
        return ret

#     @handles(model.InstantaneousAction)
#     def _convert_instantaneous_action(
#         self, a: model.InstantaneousAction
#     ) -> Action:
#         effects = []
#         conditions = []
# 
#         for cond in a.preconditions:
#             conditions.append(
#                 proto.Condition(
#                     cond=self.convert(cond),
#                     span=None,
#                 )
#             )
# 
#         for eff in a.effects:
#             effects.append(proto.Effect(effect=self.convert(eff), occurrence_time=None))
# 
#         ret = Action()
#         ret.name=a.name
#         ret.parameters=[self.convert(p) for p in a.parameters]
#         ret.duration=None
#         ret.conditions=conditions
#         ret.effects=effects
#         
#         return ret
# 
#     @handles(model.DurativeAction)
#     def _convert_durative_action(self, a: model.DurativeAction) -> Action:
#         effects = []
#         conditions = []
# 
#         for span, cond in a.conditions.items():
#             span = self.convert(span)
#             for c in cond:
#                 conditions.append(
#                     proto.Condition(
#                         cond=self.convert(c),
#                         span=span,
#                     )
#                 )
#         for ot, eff in a.effects.items():
#             ot = self.convert(ot)
#             for e in eff:
#                 effects.append(
#                     proto.Effect(
#                         effect=self.convert(e),
#                         occurrence_time=ot,
#                     )
#                 )
#     
#         ret = Action()
#         ret.name=a.name
#         ret.parameters=[self.convert(p) for p in a.parameters]
#         ret.duration=self.convert(a.duration)
#         ret.conditions=conditions
#         ret.effects=effects
#         
#         return ret
# 
    @handles(model.timing.Timepoint)
    def _convert_timepoint(self, tp: model.timing.Timepoint) -> Timepoint:
        if tp.kind == TimepointKind.START:
            kind = TimepointKind.START
        elif tp.kind == TimepointKind.END:
            kind = Timepoint.END
        elif tp.kind == TimepointKind.GLOBAL_START:
            kind = Timepoint.GLOBAL_START
        elif tp.kind == TimepointKind.GLOBAL_END:
            kind = Timepoint.GLOBAL_END
        
        ret = Timepoint()
        ret.kind = kind
        ret.container_id=tp.container
        return ret

    @handles(model.Timing)
    def _convert_timing(self, timing: model.Timing) -> Timing:
        ret = Timing()
        ret.timepoint=self.convert(timing._timepoint)
        ret.delay=self.convert(fractions.Fraction(timing.delay))
        return ret
 
    @handles(fractions.Fraction)
    def _convert_fraction(self, fraction: fractions.Fraction) -> Real:
        ret = Real()
        ret.numerator=fraction.numerator
        ret.denominator=fraction.denominator
        return ret

    @handles(model.timing.Interval)
    def _convert_interval(self, interval: model.timing.Interval) -> Interval:
        ret = Interval()

        ret.is_left_open=interval.is_left_open()
        ret.lower=self.convert(interval.lower())
        ret.is_right_open=interval.is_right_open()
        ret.upper=self.convert(interval.lower())
        return ret

    @handles(model.TimeInterval)
    def _convert_time_interval(
        self, interval: model.TimeInterval
    ) -> TimeInterval:
        ret = TimeInterval()
        ret.is_left_open=interval.is_left_open()
        ret.lower=self.convert(interval.lower)
        ret.is_right_open=interval.is_right_open()
        ret.upper=self.convert(interval.upper)
        return ret

    @handles(model.DurationInterval)
    def _convert_duration_interval(
        self, interval: model.DurationInterval
    ) -> Interval:
        ret = Interval()
        ret.controllable_in_bounds.is_left_open=interval.is_left_open()
        ret.controllable_in_bounds.lower=self.convert(interval.lower)
        ret.controllable_in_bounds.is_right_open=interval.is_right_open()
        ret.controllable_in_bounds.upper=self.convert(interval.upper)
        
        return ret
# 
#     @handles(model.htn.Task)
#     def _convert_abstract_task(
#         self, task: model.htn.Task
#     ) -> AbstractTaskDeclaration:
#         ret = AbstractTaskDeclaration()
#         ret.name=task.name
#         ret.parameters=[self.convert(p) for p in task.parameters]
# 
#         return ret
# 
#     @handles(model.htn.ParameterizedTask)
#     def _convert_parameterized_task(
#         self, task: model.htn.ParameterizedTask
#     ) -> Task:
#         parameters = []
#         for p in task.parameters:
#             aux = Expression()
#             aux.atom.symbol = p.name
#             aux.list = []
#             aux.kind = ExpressionKind.PARAMETER
#             aux.type = interface_type(p.type)
#             parameters.append(aux)
# 
#         ret.Task()
#         ret.id = ""
#         ret.task_name = task.task.name
#         ret.parameters = parameters
# 
#         return ret
# 
#     @handles(model.htn.Subtask)
#     def _convert_subtask(self, subtask: model.htn.Subtask) -> proto.Task:
#         return proto.Task(
#             id=subtask.identifier,
#             task_name=subtask.task.name,
#             parameters=[self.convert(p) for p in subtask.parameters],
#         )
# 
#     @handles(model.htn.Method)
#     def _convert_method(self, method: model.htn.Method) -> proto.Method:
#         return proto.Method(
#             name=method.name,
#             parameters=[self.convert(p) for p in method.parameters],
#             achieved_task=self.convert(method.achieved_task),
#             subtasks=[self.convert(st) for st in method.subtasks],
#             constraints=[self.convert(c) for c in method.constraints],
#             conditions=[
#                 proto.Condition(cond=self.convert(c)) for c in method.preconditions
#             ],
#         )
# 
#     @handles(model.htn.TaskNetwork)
#     def _convert_task_network(self, tn: model.htn.TaskNetwork) -> proto.TaskNetwork:
#         return proto.TaskNetwork(
#             variables=[self.convert(v) for v in tn.variables],
#             subtasks=[self.convert(st) for st in tn.subtasks],
#             constraints=[self.convert(c) for c in tn.constraints],
#         )
# 
#     def build_hierarchy(
#         self, problem: model.htn.HierarchicalProblem
#     ) -> proto.Hierarchy:
#         return proto.Hierarchy(
#             initial_task_network=self.convert(problem.task_network),
#             abstract_tasks=[self.convert(t) for t in problem.tasks],
#             methods=[self.convert(m) for m in problem.methods],
#         )
# 
#     @handles(model.Problem, model.htn.HierarchicalProblem)
#     def _convert_problem(self, problem: model.Problem) -> proto.Problem:
#         goals = [proto.Goal(goal=self.convert(g)) for g in problem.goals]
#         for (t, gs) in problem.timed_goals:
#             goals += [
#                 proto.Goal(goal=self.convert(g), timing=self.convert(t)) for g in gs
#             ]
# 
#         problem_name = str(problem.name) if problem.name is not None else ""
#         hierarchy = None
#         if isinstance(problem, model.htn.HierarchicalProblem):
#             hierarchy = self.build_hierarchy(problem)
# 
#         return proto.Problem(
#             domain_name=problem_name + "_domain",
#             problem_name=problem_name,
#             types=[self.convert(t) for t in problem.user_types],
#             fluents=[self.convert(f, problem) for f in problem.fluents],
#             objects=[self.convert(o) for o in problem.all_objects],
#             actions=[self.convert(a) for a in problem.actions],
#             initial_state=[
#                 proto.Assignment(fluent=self.convert(x), value=self.convert(v))
#                 for (x, v) in problem.initial_values.items()
#             ],
#             timed_effects=[self.convert(e) for e in problem.timed_effects],
#             goals=goals,
#             features=[map_feature(feature) for feature in problem.kind.features],
#             metrics=[self.convert(m) for m in problem.quality_metrics],
#             hierarchy=hierarchy,
#         )
# 
#     @handles(model.metrics.MinimizeActionCosts)
#     def _convert_minimize_action_costs(
#         self, metric: model.metrics.MinimizeActionCosts
#     ) -> proto.Metric:
#         action_costs = {}
#         for action, cost in metric.costs.items():
#             action_costs[action.name] = self.convert(cost)
# 
#         return proto.Metric(
#             kind=proto.Metric.MINIMIZE_ACTION_COSTS,
#             action_costs=action_costs,
#             default_action_cost=self.convert(metric.default)
#             if metric.default is not None
#             else None,
#         )
# 
#     @handles(model.metrics.MinimizeSequentialPlanLength)
#     def _convert_minimize_sequential_plan_length(self, _) -> proto.Metric:
#         return proto.Metric(
#             kind=proto.Metric.MINIMIZE_SEQUENTIAL_PLAN_LENGTH,
#         )
# 
#     @handles(model.metrics.MinimizeMakespan)
#     def _convert_minimize_makespan(self, _) -> proto.Metric:
#         return proto.Metric(
#             kind=proto.Metric.MINIMIZE_MAKESPAN,
#         )
# 
#     @handles(model.metrics.MinimizeExpressionOnFinalState)
#     def _convert_minimize_expression_on_final_state(
#         self, metric: model.metrics.MinimizeExpressionOnFinalState
#     ) -> proto.Metric:
#         return proto.Metric(
#             kind=proto.Metric.MINIMIZE_EXPRESSION_ON_FINAL_STATE,
#             expression=self.convert(metric.expression),
#         )
# 
#     @handles(model.metrics.MaximizeExpressionOnFinalState)
#     def _convert_maximize_expression_on_final_state(
#         self, metric: model.metrics.MaximizeExpressionOnFinalState
#     ) -> proto.Metric:
#         return proto.Metric(
#             kind=proto.Metric.MAXIMIZE_EXPRESSION_ON_FINAL_STATE,
#             expression=self.convert(metric.expression),
#         )
# 
#     @handles(model.metrics.Oversubscription)
#     def _convert_oversubscription_metric(
#         self, metric: model.metrics.Oversubscription
#     ) -> proto.Metric:
#         goals = []
#         for g, c in metric.goals.items():
#             goals.append(
#                 proto.GoalWithCost(
#                     goal=self.convert(g), cost=self.convert(fractions.Fraction(c))
#                 )
#             )
#         return proto.Metric(
#             kind=proto.Metric.OVERSUBSCRIPTION,
#             goals=goals,
#         )
 
    @handles(model.Parameter)
    def _convert_action_parameter(self, p: model.Parameter) -> Parameter:
        ret = Parameter()
        ret.name = p.name
        ret.type = interface_type(p.type)
        return ret

#     @handles(model.Variable)
#     def _convert_expression_variable(
#         self, variable: model.Variable
#     ) -> Expression:
#         # a variable declaration (in forall/exists) is converted directly as an expression
#         return Expression(
#             atom=proto.Atom(symbol=variable.name),
#             list=[],
#             kind=proto.ExpressionKind.Value("VARIABLE"),
#             type=proto_type(variable.type),
#         )
# 
    @handles(unified_planning.plans.ActionInstance)
    def _convert_action_instance(
        self, a: unified_planning.plans.ActionInstance, start_time=None, end_time=None
    ) -> ActionInstance:
        parameters = []
        for param in a.actual_parameters:
            # The parameters are atoms
            parameters.append(self.convert(param).expressions[0].atom)
        ret = ActionInstance()
        ret.action_name = a.action.name
        ret.parameters = parameters
        if bool(start_time) and bool(end_time):
            ret.start_time = start_time
            ret.end_time=end_time
            ret.time_triggered = True
        else:
            ret.time_triggered = False
        return ret

    @handles(str)
    def _convert_str_atom(self, s: str) -> Atom:
        ret = Atom()
        ret.symbol_atom = [s]
        return ret

    @handles(unified_planning.plans.SequentialPlan)
    def _convert_sequential_plan(
        self, plan: unified_planning.plans.SequentialPlan
    ) -> Plan:
        ret = Plan()
        ret.actions = [self.convert(a) for a in plan.actions]
        return ret

    @handles(unified_planning.plans.TimeTriggeredPlan)
    def _convert_time_triggered_plan(
        self, plan: unified_planning.plans.TimeTriggeredPlan
    ) -> Plan:
        action_instances = []

        for a in plan.timed_actions:
            start_time = self.convert(a[0])
            end_time = self.convert(a[0] + a[2])
            instance = self._convert_action_instance(
                a[1], start_time=start_time, end_time=end_time
            )
            action_instances.append(instance)
        ret = Plan()
        ret.actions = action_instances
        return ret


    @handles(unified_planning.engines.PlanGenerationResult)
    def _convert_plan_generation_result(
        self, result: unified_planning.engines.PlanGenerationResult
    ) -> PlanGenerationResult:
        log_messages = None
        if result.log_messages is not None:
            log_messages = [self.convert(log) for log in result.log_messages]
        
        ret = PlanGenerationResult()
        ret.status = self.convert(result.status)
        ret.plan=self.convert(result.plan)
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
            return PlanGenerationResult.SOLVED_SATISFICING

        elif (
            status
            == unified_planning.engines.PlanGenerationResultStatus.SOLVED_OPTIMALLY
        ):
            return PlanGenerationResult.SOLVED_OPTIMALLY
        elif (
            status
            == unified_planning.engines.PlanGenerationResultStatus.UNSOLVABLE_PROVEN
        ):
            return PlanGenerationResult.UNSOLVABLE_PROVEN
        elif (
            status
            == unified_planning.engines.PlanGenerationResultStatus.UNSOLVABLE_INCOMPLETELY
        ):
            return PlanGenerationResult.UNSOLVABLE_INCOMPLETELY
        elif status == unified_planning.engines.PlanGenerationResultStatus.TIMEOUT:
            return PlanGenerationResult.TIMEOUT
        elif status == unified_planning.engines.PlanGenerationResultStatus.MEMOUT:
            return PlanGenerationResult.MEMOUT
        elif (
            status == unified_planning.engines.PlanGenerationResultStatus.INTERNAL_ERROR
        ):
            return PlanGenerationResult.INTERNAL_ERROR
        elif (
            status
            == unified_planning.engines.PlanGenerationResultStatus.UNSUPPORTED_PROBLEM
        ):

            return PlanGenerationResult.UNSUPPORTED_PROBLEM
        elif status == unified_planning.engines.PlanGenerationResultStatus.INTERMEDIATE:
            return PlanGenerationResult.INTERMEDIATE
        else:
            raise ValueError("Unknown status: {}".format(status))

    @handles(unified_planning.engines.LogMessage)
    def _convert_log_messages(
        self, log: unified_planning.engines.LogMessage
    ) -> LogMessage:
        if log.level == unified_planning.engines.LogLevel.INFO:
            level = LogMessage.INFO
        elif log.level == unified_planning.engines.LogLevel.WARNING:
            level = LogLevel.WARNING
        elif log.level == unified_planning.engines.LogLevel.ERROR:
            level = LogMessage.ERROR
        elif log.level == unified_planning.engines.LogLevel.DEBUG:
            level = LogMessage.DEBUG
        else:
            raise UPException(f"Unknown log level: {log.level}")

        ret = LogMessage()
        ret.level = level
        ret.message = str(log.message)
        return ret
# 
#     @handles(unified_planning.engines.CompilerResult)
#     def _convert_compiler_result(
#         self, result: unified_planning.engines.CompilerResult
#     ) -> proto.CompilerResult:
#         map: Dict[str, proto.ActionInstance] = {}
#         log_messages = result.log_messages
#         if log_messages is None:
#             log_messages = []
#         if result.map_back_action_instance is not None:
#             for compiled_action in result.problem.actions:
#                 type_list = [param.type for param in compiled_action.parameters]
#                 if len(type_list) == 0:
#                     ai = unified_planning.plans.ActionInstance(compiled_action)
#                     map[str(ai)] = self.convert(result.map_back_action_instance(ai))
#                     continue
#                 ground_size = 1
#                 domain_sizes = []
#                 for t in type_list:
#                     ds = domain_size(result.problem, t)
#                     domain_sizes.append(ds)
#                     ground_size *= ds
#                 items_list: List[List[FNode]] = []
#                 for size, type in zip(domain_sizes, type_list):
#                     items_list.append(
#                         [domain_item(result.problem, type, j) for j in range(size)]
#                     )
#                 grounded_params_list = product(*items_list)
#                 for grounded_params in grounded_params_list:
#                     ai = unified_planning.plans.ActionInstance(
#                         compiled_action, tuple(grounded_params)
#                     )
#                     map[str(ai)] = self.convert(result.map_back_action_instance(ai))
#         return proto.CompilerResult(
#             problem=self.convert(result.problem),
#             map_back_plan=map,
#             log_messages=[self.convert(log) for log in log_messages],
#             engine=proto.Engine(name=result.engine_name),
#         )
# 
#     @handles(unified_planning.engines.ValidationResult)
#     def _convert_validation_result(
#         self, result: unified_planning.engines.ValidationResult
#     ) -> proto.ValidationResult:
#         return proto.ValidationResult(
#             status=self.convert(result.status),
#             log_messages=[self.convert(log) for log in result.log_messages],
#             engine=proto.Engine(name=result.engine_name),
#         )
# 
#     @handles(unified_planning.engines.ValidationResultStatus)
#     def _convert_validation_result_status(
#         self, status: unified_planning.engines.ValidationResultStatus
#     ) -> proto.ValidationResult.ValidationResultStatus:
#         if status == unified_planning.engines.ValidationResultStatus.VALID:
#             return proto.ValidationResult.ValidationResultStatus.Value("VALID")
#         elif status == unified_planning.engines.ValidationResultStatus.INVALID:
#             return proto.ValidationResult.ValidationResultStatus.Value("INVALID")
#         else:
#             raise UPException(f"Unknown result status: {status}")
