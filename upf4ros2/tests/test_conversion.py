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

# This module started from the test_protobuf_io.py module from the
# AIPlan4EU project, with the same license

from itertools import islice
import unittest

import pytest

from unified_planning import engines, model, shortcuts
from unified_planning.model import metrics
from unified_planning.plans import ActionInstance
from unified_planning.shortcuts import (
    CompilationKind,
    Compiler,
    OneshotPlanner,
    PlanValidator,
    Problem,
    UserType
)

from unified_planning.test.examples import get_example_problems


@pytest.mark.rostest
class TestROS2Interfaces(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        shortcuts.get_environment().credits_stream = None
        pass

    @classmethod
    def tearDownClass(cls):
        pass

    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)

        self.problems = get_example_problems()
        from upf4ros2.ros2_interface_reader import ROS2InterfaceReader
        from upf4ros2.ros2_interface_writer import ROS2InterfaceWriter

        self.pb_writer = ROS2InterfaceWriter()
        self.pb_reader = ROS2InterfaceReader()

    def test_fluent(self):
        problem = shortcuts.Problem('test')
        x = shortcuts.Fluent('x')

        x_pb = self.pb_writer.convert(x, problem)

        self.assertEqual(x_pb.name, 'x')
        self.assertEqual(x_pb.value_type, 'up:bool')

        x_up = self.pb_reader.convert(x_pb, problem)

        self.assertEqual(x_up.name, 'x')
        self.assertEqual(x_up.type, shortcuts.BoolType())

    def test_fluent_2(self):
        problem = self.problems['robot'].problem

        for f in problem.fluents:
            f_pb = self.pb_writer.convert(f, problem)
            f_up = self.pb_reader.convert(f_pb, problem)
            self.assertEqual(f, f_up)

    def test_fluent_3(self):
        """Test to handle subtypes of usertypes of Fluent Expression."""
        problem = self.problems['hierarchical_blocks_world'].problem

        for f in problem.fluents:
            f_pb = self.pb_writer.convert(f, problem)
            f_up = self.pb_reader.convert(f_pb, problem)
            self.assertEqual(f, f_up)

    def test_objects(self):
        """Test to handle subtypes of usertypes of Fluent Expression."""
        problem = self.problems['hierarchical_blocks_world'].problem

        for o in problem.all_objects:
            o_pb = self.pb_writer.convert(o)
            o_up = self.pb_reader.convert(o_pb, problem)

            self.assertEqual(o, o_up)

    def test_expression(self):
        problem = Problem('test')
        ex = problem.environment.expression_manager.true_expression

        ex_pb = self.pb_writer.convert(ex)
        ex_up = self.pb_reader.convert(ex_pb, problem)
        self.assertEqual(ex, ex_up)

        ex = problem.environment.expression_manager.Int(10)

        ex_pb = self.pb_writer.convert(ex)
        ex_up = self.pb_reader.convert(ex_pb, problem)
        self.assertEqual(ex, ex_up)

    def test_fluent_expressions(self):
        problem = self.problems['hierarchical_blocks_world'].problem

        problem_pb = self.pb_writer.convert(problem)
        problem_up = self.pb_reader.convert(problem_pb)

        self.assertEqual(problem, problem_up)

    def test_type_declaration(self):
        problem = Problem('test')
        ex = UserType('object')
        ex_pb = self.pb_writer.convert(ex)
        ex_up = self.pb_reader.convert(ex_pb, problem)
        self.assertEqual(ex, ex_up)

        o = model.Object('o', ex)
        problem.add_object(o)

        ex = UserType('location', ex)
        ex_pb = self.pb_writer.convert(ex)
        ex_up = self.pb_reader.convert(ex_pb, problem)
        self.assertEqual(ex, ex_up)

    def test_object_declaration(self):
        problem = Problem('test')
        loc_type = UserType('location')
        obj = problem.add_object('l1', loc_type)
        obj_pb = self.pb_writer.convert(obj)
        obj_up = self.pb_reader.convert(obj_pb, problem)
        self.assertEqual(obj, obj_up)

    def test_problem(self):
        import unified_planning.grpc.generated.unified_planning_pb2 as up_pb2

        problem = self.problems['robot'].problem

        problem_pb = self.pb_writer.convert(problem)
        problem_up = self.pb_reader.convert(problem_pb)

        pb_features = {up_pb2.Feature.Name(feature)
                       for feature in problem_pb.features}
        self.assertEqual(set(problem.kind.features), pb_features)
        self.assertEqual(problem, problem_up)

    def test_action(self):
        problem = self.problems['robot'].problem
        action = problem.action('move')

        action_pb = self.pb_writer.convert(action)
        action_up = self.pb_reader.convert(action_pb, problem)

        self.assertEqual(action, action_up)

    def test_durative_action(self):
        problem = self.problems['matchcellar'].problem
        action = problem.action('mend_fuse')

        action_pb = self.pb_writer.convert(action)
        action_up = self.pb_reader.convert(action_pb, problem)

        self.assertEqual(action, action_up)

    def test_action_instance(self):
        problem = self.problems['robot'].problem
        plan = self.problems['robot'].valid_plans[0]
        action_instance = plan.actions[0]

        action_instance_pb = self.pb_writer.convert(action_instance)
        action_instance_up = self.pb_reader.convert(
            action_instance_pb, problem)

        self.assertEqual(action_instance.action, action_instance_up.action)
        self.assertEqual(
            action_instance.actual_parameters,
            action_instance_up.actual_parameters)

    def test_plan(self):
        problem = self.problems['robot'].problem
        plan = self.problems['robot'].valid_plans[0]

        plan_pb = self.pb_writer.convert(plan)
        plan_up = self.pb_reader.convert(plan_pb, problem)

        self.assertEqual(plan, plan_up)

    def test_time_triggered_plan(self):
        problem = self.problems['temporal_conditional'].problem
        plan = self.problems['temporal_conditional'].valid_plans[0]

        plan_pb = self.pb_writer.convert(plan)
        plan_up = self.pb_reader.convert(plan_pb, problem)

        self.assertEqual(plan, plan_up)

    def test_metric(self):
        problem = Problem('test')
        problem.add_quality_metric(
            metric=metrics.MinimizeSequentialPlanLength())
        problem.add_quality_metric(metric=metrics.MinimizeMakespan())
        problem.add_quality_metric(
            metric=metrics.MinimizeExpressionOnFinalState(6)
        )
        problem.add_quality_metric(
            metric=metrics.MaximizeExpressionOnFinalState(3.5)
        )

        for metric in problem.quality_metrics:
            metric_pb = self.pb_writer.convert(metric)
            metric_up = self.pb_reader.convert(metric_pb, problem)

            self.assertEqual(str(metric), str(metric_up))

    def test_log_message(self):
        def assert_log(log):
            logger_pb = self.pb_writer.convert(log)
            logger_up = self.pb_reader.convert(logger_pb)
            self.assertEqual(log, logger_up)

        log = engines.LogMessage(engines.LogLevel.DEBUG, 'test message')
        assert_log(log)
        log = engines.LogMessage(engines.LogLevel.INFO, 'test message')
        assert_log(log)
        log = engines.LogMessage(engines.LogLevel.WARNING, 'test message')
        assert_log(log)
        log = engines.LogMessage(engines.LogLevel.ERROR, 'test message')
        assert_log(log)

    def test_plan_generation(self):
        problem = self.problems['robot'].problem

        with OneshotPlanner(name='tamer', params={'weight': 0.8}) as planner:
            self.assertNotEqual(planner, None)
            final_report = planner.solve(problem)

            final_report_pb = self.pb_writer.convert(final_report)
            final_report_up = self.pb_reader.convert(final_report_pb, problem)

            self.assertEqual(final_report, final_report_up)

    def test_compiler_result(self):
        problem = self.problems['hierarchical_blocks_world'].problem
        with Compiler(name='up_grounder') as grounder:
            ground_result = grounder.compile(
                problem, CompilationKind.GROUNDING)

            ground_result_pb = self.pb_writer.convert(ground_result)
            ground_result_up = self.pb_reader.convert(
                ground_result_pb, problem)

            self.assertEqual(ground_result.problem, ground_result_up.problem)
            for grounded_action in ground_result.problem.actions:
                # Test both callable 'map_back_action_instance' act
                # the same on every action of the grounded_problem
                grounded_action_instance = ActionInstance(grounded_action)
                original_action_instance_up = ground_result.map_back_action_instance(
                    grounded_action_instance)
                original_action_instance_pb = ground_result_up.map_back_action_instance(
                    grounded_action_instance)
                self.assertEqual(
                    original_action_instance_pb.action,
                    original_action_instance_up.action,
                )
                self.assertEqual(
                    original_action_instance_pb.actual_parameters,
                    original_action_instance_up.actual_parameters,
                )

    def test_validation_result(self):
        problem = self.problems['robot'].problem

        with OneshotPlanner(name='tamer', params={'weight': 0.8}) as planner:
            self.assertNotEqual(planner, None)
            final_report = planner.solve(problem)
            with PlanValidator(name='tamer') as validator:
                validation_result = validator.validate(
                    problem, final_report.plan)
                # prueba = self.pb_writer._convert_validation_result(validation_result)
                validation_result_pb = self.pb_writer.convert(
                    validation_result)
                validation_result_up = self.pb_reader.convert(
                    validation_result_pb)

                self.assertEqual(
                    validation_result.metrics,
                    validation_result_up.metrics)
                self.assertEqual(
                    validation_result.trace,
                    validation_result_up.trace)
                self.assertEqual(validation_result, validation_result_up)


class TestROS2InterfacesProblems(unittest.TestCase):

    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)

        self.problems = get_example_problems()
        from upf4ros2.ros2_interface_reader import ROS2InterfaceReader
        from upf4ros2.ros2_interface_writer import ROS2InterfaceWriter

        self.pb_writer = ROS2InterfaceWriter()
        self.pb_reader = ROS2InterfaceReader()

    def test_all_problems(self):
        for name, example in islice(self.problems.items(), 10):
            problem = example.problem

            problem_pb = self.pb_writer.convert(problem)
            problem_up = self.pb_reader.convert(problem_pb)
            # self.assertEqual(problem, problem_up)
            self.assertEqual(hash(problem), hash(problem_up))

    def test_all_plans(self):
        for name, example in self.problems.items():
            problem = example.problem
            if example.valid_plans:
                plan = example.valid_plans[0]
                plan_pb = self.pb_writer.convert(plan)
                plan_up = self.pb_reader.convert(plan_pb, problem)
                self.assertEqual(hash(plan), hash(plan_up))

    def test_some_plan_generations(self):
        problems = [
            'basic',
            'basic_without_negative_preconditions',
            'basic_nested_conjunctions',
            'robot_loader',
            'robot_loader_mod',
            'robot_loader_adv',
            'robot_real_constants',
            'robot_int_battery',
            'robot',
        ]
        for name in problems:
            problem = self.problems[name].problem

            with shortcuts.OneshotPlanner(name='tamer') as planner:
                self.assertNotEqual(planner, None)
                final_report = planner.solve(problem)

                final_report_pb = self.pb_writer.convert(final_report)
                final_report_up = self.pb_reader.convert(
                    final_report_pb, problem)

                self.assertEqual(final_report.status, final_report_up.status)
                self.assertEqual(final_report.plan, final_report_up.plan)
                self.assertEqual(
                    final_report.engine_name,
                    final_report_up.engine_name)


if __name__ == '__main__':
    unittest.main()
