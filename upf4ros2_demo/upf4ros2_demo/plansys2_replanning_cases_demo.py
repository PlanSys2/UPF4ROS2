import json
import time

import rclpy

from upf4ros2_demo.plansys2_replanning_demo import (
    DOMAIN,
    EDGES,
    GOALS,
    INITIAL_LOCATION,
    OBJECTS,
    PlanSys2ReplanningDemo,
)


BLOCKED_EDGE_CASE = 'blocked_edge'
GOAL_CHANGE_CASE = 'goal_change'
BOTH_CASE = 'both'


def make_problem(current_location, visited, edges, goals, blocked_locations=None):
    visited = set(visited or [INITIAL_LOCATION])
    blocked_locations = set(blocked_locations or [])
    edges = list(edges or EDGES)
    init_lines = [f"    (robot_at {current_location})"]
    init_lines.extend(f"    (visited {loc})" for loc in sorted(visited))
    init_lines.extend(f"    (connected {source} {target})" for source, target in edges)
    active_goals = [goal for goal in goals if goal not in blocked_locations]
    goal_lines = " ".join(f"(visited {goal})" for goal in active_goals) or f"(visited {current_location})"
    return f"""
(define (problem plansys2_replanning_problem)
  (:domain plansys2_replanning_demo)
  (:objects {' '.join(OBJECTS)} - location)
  (:init
{chr(10).join(init_lines)}
  )
  (:goal (and {goal_lines}))
)
""".strip()


class PlanSys2ReplanningCasesDemo(PlanSys2ReplanningDemo):

    def __init__(self):
        super().__init__()
        self.declare_parameter('replanning_case', BOTH_CASE)
        self.declare_parameter('initial_goals', ['bedroom', 'gym'])
        self.declare_parameter('changed_goals', ['room'])
        self.declare_parameter('goal_change_after_successes', 1)

        self.replanning_case = self.get_parameter('replanning_case').value
        if self.replanning_case not in (BLOCKED_EDGE_CASE, GOAL_CHANGE_CASE, BOTH_CASE):
            raise ValueError(
                f"replanning_case must be '{BLOCKED_EDGE_CASE}', '{GOAL_CHANGE_CASE}', or '{BOTH_CASE}'")

        self.initial_goals = self._parameter_list('initial_goals')
        self.changed_goals = self._parameter_list('changed_goals')
        self.current_goals = list(self.initial_goals)
        self.goal_change_after_successes = int(self.get_parameter('goal_change_after_successes').value)
        self.goal_change_applied = False
        self.actions_at_last_replan = 0

        if self.configured_blocked_edge == ('livingroom', 'entrance'):
            self.configured_blocked_edge = ('kitchen', 'gym')

    def _parameter_list(self, name):
        value = self.get_parameter(name).value
        if isinstance(value, str):
            return [value]
        return list(value)

    def _scenario_name(self):
        if self.replanning_case == BLOCKED_EDGE_CASE:
            return 'plansys2_blocked_edge_replanning'
        if self.replanning_case == GOAL_CHANGE_CASE:
            return 'plansys2_goal_change_replanning'
        return 'plansys2_combined_replanning'

    def _write_event(self, event, **data):
        now_ns = time.monotonic_ns()
        payload = {
            'run_id': self.run_id,
            'pipeline': self.pipeline,
            'scenario': self._scenario_name(),
            'replanning_case': self.replanning_case,
            'event': event,
            'timestamp_ns': now_ns,
            'relative_time_s': (now_ns - self.start_ns) / 1e9,
        }
        payload.update(data)
        self.events.write(json.dumps(payload, sort_keys=True) + '\n')
        self.events.flush()

    def _active_goals(self):
        return [goal for goal in self.current_goals if goal not in self.blocked_locations]

    def _goals_reached(self, visited):
        return all(goal in visited for goal in self._active_goals())

    def _maybe_trigger_goal_change(self, current_location, visited):
        if self.replanning_case not in (GOAL_CHANGE_CASE, BOTH_CASE) or self.goal_change_applied:
            return False
        executed_in_current_plan = len(self.successful_actions) - self.actions_at_last_replan
        if executed_in_current_plan < self.goal_change_after_successes:
            return False
        if self.replanning_case == BOTH_CASE and self.replans == 0:
            return False

        previous_goals = list(self.current_goals)
        self.current_goals = list(self.changed_goals)
        self.goal_change_applied = True
        self.disruption_action = self.successful_actions[-1] if self.successful_actions else None
        self.disruption_action_index = len(self.attempted_actions)
        self.state_at_disruption = {
            'current_location': current_location,
            'visited': sorted(visited),
        }
        self._write_event(
            'execution_disruption_detected',
            action=self.disruption_action,
            action_index=self.disruption_action_index,
            failure_reason='goal_changed',
            previous_goals=previous_goals,
            changed_goals=self.current_goals,
            state_at_disruption=self.state_at_disruption)
        self._write_event(
            'execution_stopped_for_replanning',
            reason='goal_changed',
            current_location=current_location,
            active_goals=self._active_goals())
        return True

    def execute_until_failure_or_goal(self, actions, current_location, visited, edges):
        executed = 0
        for action in actions:
            parts = action.strip('()').split()
            if len(parts) != 3 or parts[0] != 'move':
                self.failed_actions.append(action)
                return current_location, visited, edges, executed, True, 'invalid_action'
            _, source, target = parts
            self.attempted_actions.append(action)
            self._write_event('action_dispatch_started', action=action, active_goals=self._active_goals())
            if source != current_location:
                self.failed_actions.append(action)
                self._write_event('action_failed', action=action, failure_reason='state_mismatch')
                return current_location, visited, edges, executed, True, 'state_mismatch'
            if self.replanning_case in (BLOCKED_EDGE_CASE, BOTH_CASE):
                edges, disturbed = self._trigger_disturbance(action, source, target, edges, current_location, visited)
                if disturbed:
                    self._write_event(
                        'execution_stopped_for_replanning',
                        reason='blocked_edge',
                        current_location=current_location,
                        active_goals=self._active_goals())
                    return current_location, visited, edges, executed, True, 'blocked_edge'
            if bool(self.get_parameter('execute_actions').value) and not self._call_action('move', [source, target]):
                self.blocked_edge = (source, target)
                self.blocked_locations.add(target)
                edges = self._remove_location_edges(edges, target)
                self.failed_actions.append(action)
                self._write_event(
                    'action_failed',
                    action=action,
                    failure_reason='action_execution_failed',
                    blocked_location=target)
                self.get_logger().warn(
                    f'Action {action} failed or goal was rejected; marking {target} inaccessible '
                    f'and replanning without any connections to or from it')
                return current_location, visited, edges, executed, True, 'action_execution_failed'
            current_location = target
            visited.add(target)
            executed += 1
            self.successful_actions.append(action)
            self._write_event('action_succeeded', action=action, current_location=current_location)
            if self._maybe_trigger_goal_change(current_location, visited):
                return current_location, visited, edges, executed, True, 'goal_changed'
            if self._goals_reached(visited):
                return current_location, visited, edges, executed, False, ''
        return current_location, visited, edges, executed, not self._goals_reached(visited), 'goals_not_reached'

    def run_demo(self):
        self.configure_planner()
        mission_start_ns = time.monotonic_ns()
        self._write_event('mission_started', active_goals=self._active_goals())
        current_location = INITIAL_LOCATION
        visited = {INITIAL_LOCATION}
        edges = list(EDGES)
        max_replans = int(self.get_parameter('max_replans').value)
        total_executed = 0
        failure_reason = ''

        problem = make_problem(current_location, visited, edges, self.current_goals, self.blocked_locations)
        plan = self.get_plan(problem, 'initial')
        self.first_plan_length = len(plan)

        while rclpy.ok():
            current_location, visited, edges, executed, failed, failure_reason = self.execute_until_failure_or_goal(
                plan, current_location, visited, edges)
            total_executed += executed
            active_goals = self._active_goals()
            if not failed and all(goal in visited for goal in active_goals):
                self.final_plan_length = len(plan)
                break
            if self.replans >= max_replans:
                break
            self.replans += 1
            replanning_problem = make_problem(current_location, visited, edges, self.current_goals, self.blocked_locations)
            self._write_event(
                'replanning_triggered',
                replan_index=self.replans,
                current_location=current_location,
                visited=sorted(visited),
                removed_edge=list(self.blocked_edge) if self.blocked_edge else None,
                blocked_locations=sorted(self.blocked_locations),
                active_goals=active_goals,
                failure_reason=failure_reason,
                problem=replanning_problem)
            self.get_logger().warn(
                f'Replanning #{self.replans} after {failure_reason}; current_location={current_location}, '
                f'visited={sorted(visited)}, active_goals={active_goals}, removed_edge={self.blocked_edge}, '
                f'blocked_locations={sorted(self.blocked_locations)}')
            plan = self.get_plan(replanning_problem, f'replan_{self.replans}')
            self.actions_at_last_replan = len(self.successful_actions)

        active_goals = self._active_goals()
        goals_achieved = [goal for goal in self.current_goals if goal in visited]
        goals_unachieved = [goal for goal in active_goals if goal not in visited]
        success = all(goal in visited for goal in active_goals)
        if not self.final_plan_length and plan:
            self.final_plan_length = len(plan)
        mission_time_s = (time.monotonic_ns() - mission_start_ns) / 1e9
        execution_disruption_detected = self.disruption_action is not None
        replanning_triggered = self.replans > 0
        valid_replanning_trial = success and execution_disruption_detected and replanning_triggered
        recovery_success_after_disruption = success and execution_disruption_detected
        summary = {
            'run_id': self.run_id,
            'scenario': self._scenario_name(),
            'replanning_case': self.replanning_case,
            'pipeline': self.pipeline,
            'success': success,
            'execution_disruption_detected': execution_disruption_detected,
            'replanning_triggered': replanning_triggered,
            'valid_replanning_trial': valid_replanning_trial,
            'recovery_success_after_disruption': recovery_success_after_disruption,
            'planning_time_total_s': sum(self.planning_times),
            'planning_time_initial_s': self.planning_times[0] if self.planning_times else None,
            'replanning_latency_s': self.replanning_times[0] if self.replanning_times else None,
            'replanning_time_mean_s': sum(self.replanning_times) / len(self.replanning_times) if self.replanning_times else 0.0,
            'initial_plan_length': self.first_plan_length,
            'final_plan_length': self.final_plan_length,
            'replan_plan_lengths': [plan['plan_length'] for plan in self.plan_history if plan['reason'].startswith('replan')],
            'initial_plan_unit_action_cost': self.first_plan_length,
            'final_plan_unit_action_cost': self.final_plan_length,
            'replan_plan_unit_action_costs': [plan['plan_length'] for plan in self.plan_history if plan['reason'].startswith('replan')],
            'total_generated_plan_actions': sum(plan['plan_length'] for plan in self.plan_history),
            'plan_history': self.plan_history,
            'mission_time_s': mission_time_s,
            'replans': self.replans,
            'attempted_actions': len(self.attempted_actions),
            'attempted_action_trace': self.attempted_actions,
            'successful_actions': total_executed,
            'successful_action_trace': self.successful_actions,
            'failed_actions': len(self.failed_actions),
            'failed_action_trace': self.failed_actions,
            'configured_blocked_edge': list(self.configured_blocked_edge),
            'configured_blocked_edge_used': self.disturbance_trigger == 'on_edge_attempt',
            'disturbance_trigger': self.disturbance_trigger,
            'goal_change_applied': self.goal_change_applied,
            'initial_goals': self.initial_goals,
            'changed_goals': self.changed_goals,
            'disruption_action': self.disruption_action,
            'disruption_action_index': self.disruption_action_index,
            'state_at_disruption': self.state_at_disruption,
            'blocked_edge': list(self.blocked_edge) if self.blocked_edge else None,
            'blocked_locations': sorted(self.blocked_locations),
            'active_goals': active_goals,
            'goals': self.current_goals,
            'goals_achieved': goals_achieved,
            'goals_unachieved': goals_unachieved,
            'goal_achievement_ratio': len(goals_achieved) / len(self.current_goals) if self.current_goals else 1.0,
            'plan_quality_metric': 'unit_action_count',
            'failure_reason': '' if success else failure_reason,
            'events_file': str(self.events_path),
        }
        self.summary_path.write_text(json.dumps(summary, indent=2, sort_keys=True) + '\n')
        self._write_event('mission_succeeded' if success else 'mission_failed', **summary)
        self.get_logger().info(f'Wrote replanning summary to {self.summary_path}')
        return success


def main(args=None):
    rclpy.init(args=args)
    node = PlanSys2ReplanningCasesDemo()
    try:
        success = node.run_demo()
    finally:
        node.destroy_node()
        rclpy.shutdown()
    return 0 if success else 1


if __name__ == '__main__':
    raise SystemExit(main())
