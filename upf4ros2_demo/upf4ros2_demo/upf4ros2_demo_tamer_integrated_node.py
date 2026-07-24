import argparse
import csv
from collections import OrderedDict
from fractions import Fraction
import os
from pathlib import Path
import subprocess
import time

import rclpy
from rclpy.node import Node

from unified_planning import model
from unified_planning.shortcuts import (
    And,
    BoolType,
    ClosedTimeInterval,
    Div,
    EndTiming,
    Equals,
    GE,
    Int,
    LE,
    Not,
    OneshotPlanner,
    Plus,
    Real,
    RealType,
    StartTiming,
    Times,
    UserType,
    get_environment,
)

from upf4ros2.ros2_interface_writer import ROS2InterfaceWriter
from upf_msgs.srv import PlanOneShot as PlanOneShotSrv


CLK_TCK = os.sysconf(os.sysconf_names['SC_CLK_TCK'])


def now():
    return time.perf_counter()


def read_self_resource():
    try:
        import resource
        usage = resource.getrusage(resource.RUSAGE_SELF)
        return usage.ru_utime + usage.ru_stime, usage.ru_maxrss
    except Exception:
        return None, None


def read_proc_stat(pid):
    try:
        stat = Path(f'/proc/{pid}/stat').read_text().split()
        return (int(stat[13]) + int(stat[14])) / CLK_TCK
    except Exception:
        return None


def read_proc_rss_kb(pid):
    try:
        for line in Path(f'/proc/{pid}/status').read_text().splitlines():
            if line.startswith('VmRSS:'):
                return int(line.split()[1])
    except Exception:
        return None
    return None


def find_upf4ros2_server_pids():
    try:
        result = subprocess.run(
            ['pgrep', '-f', 'upf4ros2_main'],
            check=False,
            capture_output=True,
            text=True)
    except Exception:
        return []
    self_pid = os.getpid()
    pids = []
    for token in result.stdout.split():
        try:
            pid = int(token)
        except ValueError:
            continue
        if pid != self_pid:
            pids.append(pid)
    return pids


def sum_proc_cpu(pids):
    return sum(value for value in (read_proc_stat(pid) for pid in pids) if value is not None)


def sum_proc_rss_kb(pids):
    return sum(value for value in (read_proc_rss_kb(pid) for pid in pids) if value is not None)


class Metrics:
    def __init__(self):
        self.values = {}

    def add(self, key, value):
        self.values[key] = self.values.get(key, 0.0) + value

    def inc(self, key, value=1):
        self.values[key] = self.values.get(key, 0) + value

    def measure(self, key, func, *args, **kwargs):
        start = now()
        result = func(*args, **kwargs)
        self.add(key, now() - start)
        return result


def real(value):
    return Real(Fraction(str(value)))


def signature(**kwargs):
    return OrderedDict(kwargs)


def build_tamer_complex_integrated_problem(metrics=None):
    metrics = metrics or Metrics()
    problem = metrics.measure(
        'build_create_problem_s', model.Problem, 'social-care-tamer-complex-integrated')

    start_types = now()
    location = UserType('location')
    robot = UserType('robot')
    person = UserType('person')
    item = UserType('item')
    request = UserType('request')
    resource = UserType('resource')
    capability = UserType('capability')
    metrics.add('build_create_types_s', now() - start_types)

    start_fluents = now()
    connected = model.Fluent('connected', BoolType(), _signature=signature(**{'from': location, 'to': location}))
    corridor_link = model.Fluent('corridor-link', BoolType(), _signature=signature(**{'from': location, 'to': location}))
    blocked = model.Fluent('blocked', BoolType(), _signature=signature(**{'from': location, 'to': location}))
    can_enter = model.Fluent('can-enter', BoolType(), r=robot, l=location)
    can_move = model.Fluent('can-move', BoolType(), _signature=signature(r=robot, **{'from': location, 'to': location}))
    robot_at = model.Fluent('robot-at', BoolType(), r=robot, l=location)
    person_at = model.Fluent('person-at', BoolType(), p=person, l=location)
    item_at = model.Fluent('item-at', BoolType(), i=item, l=location)
    robot_free = model.Fluent('robot-free', BoolType(), r=robot)
    available = model.Fluent('available', BoolType(), x=resource)
    charger_for = model.Fluent('charger-for', BoolType(), dock=resource, l=location)
    uses_corridor = model.Fluent('uses-corridor', BoolType(), _signature=signature(**{'from': location, 'to': location, 'c': resource}))
    uses_terminal = model.Fluent('uses-terminal', BoolType(), q=request, terminal=resource)
    robot_has = model.Fluent('robot-has', BoolType(), r=robot, c=capability)
    can_pick = model.Fluent('can-pick', BoolType(), r=robot, i=item)
    can_deliver = model.Fluent('can-deliver', BoolType(), r=robot, q=request)
    can_guide = model.Fluent('can-guide', BoolType(), r=robot, q=request)
    can_comfort = model.Fluent('can-comfort', BoolType(), r=robot, q=request)
    can_assist = model.Fluent('can-assist', BoolType(), monitor=robot, actor=robot, q=request)
    can_handover = model.Fluent('can-handover', BoolType(), _signature=signature(**{'from': robot, 'to': robot, 'i': item, 'l': location}))
    can_cooperate = model.Fluent('can-cooperate', BoolType(), left=robot, right=robot, i=item)
    request_person = model.Fluent('request-person', BoolType(), q=request, p=person)
    request_item = model.Fluent('request-item', BoolType(), q=request, i=item)
    request_origin = model.Fluent('request-origin', BoolType(), q=request, l=location)
    request_destination = model.Fluent('request-destination', BoolType(), q=request, l=location)
    request_capability = model.Fluent('request-capability', BoolType(), q=request, c=capability)
    carrying = model.Fluent('carrying', BoolType(), r=robot, i=item)
    co_carrying = model.Fluent('co-carrying', BoolType(), left=robot, right=robot, i=item)
    handover_done = model.Fluent('handover-done', BoolType(), i=item, _from=robot, to=robot)
    delivered = model.Fluent('delivered', BoolType(), i=item, p=person)
    comforted = model.Fluent('comforted', BoolType(), p=person)
    assisted = model.Fluent('assisted', BoolType(), p=person)
    checked_in = model.Fluent('checked-in', BoolType(), p=person)
    guided = model.Fluent('guided', BoolType(), p=person, l=location)
    service_completed = model.Fluent('service-completed', BoolType(), q=request)

    battery = model.Fluent('battery', RealType(), r=robot)
    battery_max = model.Fluent('battery-max', RealType(), r=robot)
    speed = model.Fluent('speed', RealType(), r=robot)
    energy_rate = model.Fluent('energy-rate', RealType(), r=robot)
    payload_capacity = model.Fluent('payload-capacity', RealType(), r=robot)
    current_load = model.Fluent('current-load', RealType(), r=robot)
    item_weight = model.Fluent('item-weight', RealType(), i=item)
    item_size = model.Fluent('item-size', RealType(), i=item)
    request_priority = model.Fluent('request-priority', RealType(), q=request)
    task_duration = model.Fluent('task-duration', RealType(), q=request)
    total_cost = model.Fluent('total-cost', RealType())
    metrics.add('build_create_fluents_s', now() - start_fluents)

    for fluent in [
        connected, corridor_link, blocked, can_enter, can_move, robot_at, person_at, item_at,
        robot_free, available, charger_for, uses_corridor, uses_terminal, robot_has, can_pick,
        can_deliver, can_guide, can_comfort, can_assist, can_handover, can_cooperate,
        request_person, request_item, request_origin, request_destination, request_capability,
        carrying, co_carrying, handover_done, delivered, comforted, assisted, checked_in, guided,
        service_completed,
    ]:
        metrics.measure('build_add_fluents_s', problem.add_fluent, fluent, default_initial_value=False)
        metrics.inc('build_add_fluent_calls')
    for fluent in [
        battery, battery_max, speed, energy_rate, payload_capacity, current_load, item_weight,
        item_size, request_priority, task_duration, total_cost,
    ]:
        metrics.measure('build_add_fluents_s', problem.add_fluent, fluent, default_initial_value=real(0))
        metrics.inc('build_add_fluent_calls')

    objects = {}
    for type_, names in [
        (capability, 'nav-cap delivery-cap social-cap monitor-cap lift-left-cap lift-right-cap admin-cap guide-cap'.split()),
        (location, 'reception pharmacy central-corridor ward-1 ward-2 laboratory social-room charge-a charge-b'.split()),
        (robot, 'r-admin r-pharmacy r-ward r-guide r-social r-remote r-hybrid'.split()),
        (person, 'visitor-a visitor-b resident-a resident-b resident-c resident-e resident-f'.split()),
        (item, 'med-a med-b oxygen-unit meal-tray'.split()),
        (request, 'q-checkin q-guide q-wellbeing q-remote q-med-normal q-med-handover q-heavy q-social-urgent'.split()),
        (resource, 'terminal-1 elevator-1 dock-a dock-b'.split()),
    ]:
        for name in names:
            objects[name] = metrics.measure('build_create_objects_s', model.Object, name, type_)
            metrics.measure('build_add_objects_s', problem.add_object, objects[name])
            metrics.inc('build_add_object_calls')

    o = objects
    start = StartTiming()
    end = EndTiming()
    overall = ClosedTimeInterval(start, end)

    def da(name, params, duration):
        action = metrics.measure('build_create_actions_s', model.DurativeAction, name, _parameters=OrderedDict(params))
        metrics.measure('build_set_durations_s', action.set_fixed_duration, duration)
        metrics.inc('build_create_action_calls')
        return action

    def add_action(action):
        metrics.measure('build_add_actions_s', problem.add_action, action)
        metrics.inc('build_add_action_calls')

    def p(action, name):
        return action.parameter(name)

    def add_start_conditions(action, conditions):
        for condition in conditions:
            metrics.measure('build_add_conditions_s', action.add_condition, start, condition)
            metrics.inc('build_add_condition_calls')

    def add_overall_conditions(action, conditions):
        for condition in conditions:
            metrics.measure('build_add_conditions_s', action.add_condition, overall, condition)
            metrics.inc('build_add_condition_calls')

    def add_start_effects(action, effects):
        for fluent_exp, value in effects:
            metrics.measure('build_add_effects_s', action.add_effect, start, fluent_exp, value)
            metrics.inc('build_add_effect_calls')

    def add_end_effects(action, effects):
        for fluent_exp, value in effects:
            metrics.measure('build_add_effects_s', action.add_effect, end, fluent_exp, value)
            metrics.inc('build_add_effect_calls')

    def add_start_decrease(action, fluent_exp, value):
        metrics.measure('build_add_effects_s', action.add_decrease_effect, start, fluent_exp, value)
        metrics.inc('build_add_effect_calls')

    def add_end_decrease(action, fluent_exp, value):
        metrics.measure('build_add_effects_s', action.add_decrease_effect, end, fluent_exp, value)
        metrics.inc('build_add_effect_calls')

    def add_end_increase(action, fluent_exp, value):
        metrics.measure('build_add_effects_s', action.add_increase_effect, end, fluent_exp, value)
        metrics.inc('build_add_effect_calls')

    r_from_to = [('r', robot), ('from', location), ('to', location)]

    navigate = da('navigate', r_from_to, Int(1))
    r, fr, to = p(navigate, 'r'), p(navigate, 'from'), p(navigate, 'to')
    metrics.measure('build_set_durations_s', navigate.set_fixed_duration, Plus(Int(2), Div(Int(3), speed(r))))
    add_start_conditions(navigate, [robot_at(r, fr), connected(fr, to), can_move(r, fr, to), Not(blocked(fr, to)), can_enter(r, to), robot_free(r), robot_has(r, o['nav-cap']), GE(battery(r), Times(Int(3), energy_rate(r)))])
    add_overall_conditions(navigate, [Not(blocked(fr, to))])
    add_start_effects(navigate, [(robot_free(r), False), (robot_at(r, fr), False)])
    add_start_decrease(navigate, battery(r), Times(Int(3), energy_rate(r)))
    add_end_effects(navigate, [(robot_at(r, to), True), (robot_free(r), True)])
    add_end_increase(navigate, total_cost(), Plus(Int(2), Times(Int(3), energy_rate(r))))
    add_action(navigate)

    traverse = da('traverse-shared-corridor', [('r', robot), ('from', location), ('to', location), ('corridor', resource)], Int(1))
    r, fr, to, corridor = p(traverse, 'r'), p(traverse, 'from'), p(traverse, 'to'), p(traverse, 'corridor')
    metrics.measure('build_set_durations_s', traverse.set_fixed_duration, Plus(Int(4), Div(Int(3), speed(r))))
    add_start_conditions(traverse, [robot_at(r, fr), connected(fr, to), can_move(r, fr, to), corridor_link(fr, to), uses_corridor(fr, to, corridor), available(corridor), can_enter(r, to), robot_free(r), robot_has(r, o['nav-cap']), GE(battery(r), Times(Int(4), energy_rate(r)))])
    add_overall_conditions(traverse, [Not(blocked(fr, to))])
    add_start_effects(traverse, [(available(corridor), False), (robot_free(r), False), (robot_at(r, fr), False)])
    add_start_decrease(traverse, battery(r), Times(Int(4), energy_rate(r)))
    add_end_effects(traverse, [(robot_at(r, to), True), (available(corridor), True), (robot_free(r), True)])
    add_end_increase(traverse, total_cost(), Plus(Int(5), Times(Int(4), energy_rate(r))))
    add_action(traverse)

    charge = da('charge-robot', [('r', robot), ('l', location), ('dock', resource)], Int(10))
    r, l, dock = p(charge, 'r'), p(charge, 'l'), p(charge, 'dock')
    add_overall_conditions(charge, [robot_at(r, l)])
    add_start_conditions(charge, [charger_for(dock, l), available(dock), robot_free(r)])
    add_start_effects(charge, [(available(dock), False), (robot_free(r), False)])
    add_end_effects(charge, [(battery(r), battery_max(r)), (available(dock), True), (robot_free(r), True)])
    add_end_increase(charge, total_cost(), Int(1))
    add_action(charge)

    pick_item = da('pick-item', [('r', robot), ('i', item), ('l', location)], Int(1))
    r, i, l = p(pick_item, 'r'), p(pick_item, 'i'), p(pick_item, 'l')
    metrics.measure('build_set_durations_s', pick_item.set_fixed_duration, Plus(Int(1), item_size(i)))
    add_overall_conditions(pick_item, [robot_at(r, l)])
    add_start_conditions(pick_item, [item_at(i, l), robot_free(r), robot_has(r, o['delivery-cap']), can_pick(r, i), LE(Plus(current_load(r), item_weight(i)), payload_capacity(r))])
    add_start_effects(pick_item, [(robot_free(r), False), (item_at(i, l), False)])
    add_end_effects(pick_item, [(carrying(r, i), True), (robot_free(r), True)])
    add_end_increase(pick_item, current_load(r), item_weight(i))
    add_end_increase(pick_item, total_cost(), item_size(i))
    add_action(pick_item)

    deliver = da('deliver-item', [('r', robot), ('i', item), ('p', person), ('l', location), ('q', request)], Int(1))
    r, i, pe, l, q = p(deliver, 'r'), p(deliver, 'i'), p(deliver, 'p'), p(deliver, 'l'), p(deliver, 'q')
    metrics.measure('build_set_durations_s', deliver.set_fixed_duration, Plus(Int(2), task_duration(q)))
    add_overall_conditions(deliver, [robot_at(r, l), person_at(pe, l)])
    add_start_conditions(deliver, [carrying(r, i), robot_free(r), robot_has(r, o['delivery-cap']), can_deliver(r, q), request_person(q, pe), request_item(q, i), request_destination(q, l), request_capability(q, o['delivery-cap'])])
    add_start_effects(deliver, [(robot_free(r), False), (carrying(r, i), False)])
    add_end_effects(deliver, [(delivered(i, pe), True), (service_completed(q), True), (robot_free(r), True)])
    add_end_decrease(deliver, current_load(r), item_weight(i))
    add_end_increase(deliver, total_cost(), request_priority(q))
    add_action(deliver)

    handover = da('handover-item', [('from', robot), ('to', robot), ('i', item), ('l', location)], Int(3))
    rf, rt, i, l = p(handover, 'from'), p(handover, 'to'), p(handover, 'i'), p(handover, 'l')
    add_overall_conditions(handover, [robot_at(rf, l), robot_at(rt, l)])
    add_start_conditions(handover, [carrying(rf, i), can_handover(rf, rt, i, l), robot_free(rf), robot_free(rt), robot_has(rt, o['delivery-cap']), LE(Plus(current_load(rt), item_weight(i)), payload_capacity(rt))])
    add_start_effects(handover, [(robot_free(rf), False), (robot_free(rt), False), (carrying(rf, i), False)])
    add_end_effects(handover, [(carrying(rt, i), True), (handover_done(i, rf, rt), True), (robot_free(rf), True), (robot_free(rt), True)])
    add_end_decrease(handover, current_load(rf), item_weight(i))
    add_end_increase(handover, current_load(rt), item_weight(i))
    add_end_increase(handover, total_cost(), Int(4))
    add_action(handover)

    handover_deliver = da('handover-deliver-item', [('from', robot), ('to', robot), ('i', item), ('p', person), ('l', location), ('q', request)], Int(1))
    rf, rt, i, pe, l, q = [p(handover_deliver, n) for n in ['from', 'to', 'i', 'p', 'l', 'q']]
    metrics.measure('build_set_durations_s', handover_deliver.set_fixed_duration, Plus(Int(4), task_duration(q)))
    add_overall_conditions(handover_deliver, [robot_at(rf, l), robot_at(rt, l), person_at(pe, l)])
    add_start_conditions(handover_deliver, [carrying(rf, i), can_handover(rf, rt, i, l), can_deliver(rt, q), robot_free(rf), robot_free(rt), robot_has(rt, o['delivery-cap']), request_person(q, pe), request_item(q, i), request_destination(q, l), request_capability(q, o['delivery-cap'])])
    add_start_effects(handover_deliver, [(robot_free(rf), False), (robot_free(rt), False), (carrying(rf, i), False)])
    add_end_effects(handover_deliver, [(handover_done(i, rf, rt), True), (delivered(i, pe), True), (service_completed(q), True), (robot_free(rf), True), (robot_free(rt), True)])
    add_end_decrease(handover_deliver, current_load(rf), item_weight(i))
    add_end_increase(handover_deliver, total_cost(), Plus(Int(4), request_priority(q)))
    add_action(handover_deliver)

    coop_pick = da('cooperative-pick-heavy', [('left', robot), ('right', robot), ('i', item), ('l', location)], Int(1))
    left, right, i, l = [p(coop_pick, n) for n in ['left', 'right', 'i', 'l']]
    metrics.measure('build_set_durations_s', coop_pick.set_fixed_duration, Plus(Int(4), item_size(i)))
    add_overall_conditions(coop_pick, [robot_at(left, l), robot_at(right, l)])
    add_start_conditions(coop_pick, [item_at(i, l), can_cooperate(left, right, i), robot_free(left), robot_free(right), robot_has(left, o['lift-left-cap']), robot_has(right, o['lift-right-cap'])])
    add_start_effects(coop_pick, [(robot_free(left), False), (robot_free(right), False), (item_at(i, l), False)])
    add_end_effects(coop_pick, [(co_carrying(left, right, i), True), (robot_free(left), True), (robot_free(right), True)])
    add_end_increase(coop_pick, total_cost(), Plus(Int(8), item_size(i)))
    add_action(coop_pick)

    coop_transport = da('cooperative-transport-heavy', [('left', robot), ('right', robot), ('i', item), ('p', person), ('from', location), ('to', location), ('q', request)], Int(1))
    left, right, i, pe, fr, to, q = [p(coop_transport, n) for n in ['left', 'right', 'i', 'p', 'from', 'to', 'q']]
    metrics.measure('build_set_durations_s', coop_transport.set_fixed_duration, Plus(Int(9), task_duration(q)))
    add_overall_conditions(coop_transport, [person_at(pe, to)])
    add_start_conditions(coop_transport, [robot_at(left, fr), robot_at(right, fr), item_at(i, fr), connected(fr, to), can_move(left, fr, to), can_move(right, fr, to), can_cooperate(left, right, i), robot_free(left), robot_free(right), robot_has(left, o['lift-left-cap']), robot_has(right, o['lift-right-cap']), GE(battery(left), Times(Int(5), energy_rate(left))), GE(battery(right), Times(Int(5), energy_rate(right))), request_person(q, pe), request_item(q, i), request_origin(q, fr), request_destination(q, to), request_capability(q, o['lift-left-cap'])])
    add_start_effects(coop_transport, [(robot_free(left), False), (robot_free(right), False), (robot_at(left, fr), False), (robot_at(right, fr), False), (item_at(i, fr), False)])
    add_start_decrease(coop_transport, battery(left), Times(Int(5), energy_rate(left)))
    add_start_decrease(coop_transport, battery(right), Times(Int(5), energy_rate(right)))
    add_end_effects(coop_transport, [(robot_at(left, to), True), (robot_at(right, to), True), (delivered(i, pe), True), (service_completed(q), True), (robot_free(left), True), (robot_free(right), True)])
    add_end_increase(coop_transport, total_cost(), Plus(Int(12), request_priority(q)))
    add_action(coop_transport)

    coop_carry = da('cooperative-carry-heavy', [('left', robot), ('right', robot), ('i', item), ('from', location), ('to', location)], Int(1))
    left, right, i, fr, to = [p(coop_carry, n) for n in ['left', 'right', 'i', 'from', 'to']]
    metrics.measure('build_set_durations_s', coop_carry.set_fixed_duration, Plus(Int(5), Div(Int(3), speed(left))))
    add_start_conditions(coop_carry, [co_carrying(left, right, i), robot_at(left, fr), robot_at(right, fr), connected(fr, to), can_move(left, fr, to), can_move(right, fr, to), Not(blocked(fr, to)), can_enter(left, to), can_enter(right, to), robot_free(left), robot_free(right), GE(battery(left), Times(Int(5), energy_rate(left))), GE(battery(right), Times(Int(5), energy_rate(right)))])
    add_overall_conditions(coop_carry, [Not(blocked(fr, to))])
    add_start_effects(coop_carry, [(robot_free(left), False), (robot_free(right), False), (robot_at(left, fr), False), (robot_at(right, fr), False)])
    add_start_decrease(coop_carry, battery(left), Times(Int(5), energy_rate(left)))
    add_start_decrease(coop_carry, battery(right), Times(Int(5), energy_rate(right)))
    add_end_effects(coop_carry, [(robot_at(left, to), True), (robot_at(right, to), True), (robot_free(left), True), (robot_free(right), True)])
    add_end_increase(coop_carry, total_cost(), Int(10))
    add_action(coop_carry)

    coop_deliver = da('cooperative-deliver-heavy', [('left', robot), ('right', robot), ('i', item), ('p', person), ('l', location), ('q', request)], Int(1))
    left, right, i, pe, l, q = [p(coop_deliver, n) for n in ['left', 'right', 'i', 'p', 'l', 'q']]
    metrics.measure('build_set_durations_s', coop_deliver.set_fixed_duration, Plus(Int(5), task_duration(q)))
    add_overall_conditions(coop_deliver, [robot_at(left, l), robot_at(right, l), person_at(pe, l)])
    add_start_conditions(coop_deliver, [co_carrying(left, right, i), robot_free(left), robot_free(right), request_person(q, pe), request_item(q, i), request_destination(q, l), request_capability(q, o['lift-left-cap'])])
    add_start_effects(coop_deliver, [(robot_free(left), False), (robot_free(right), False), (co_carrying(left, right, i), False)])
    add_end_effects(coop_deliver, [(delivered(i, pe), True), (service_completed(q), True), (robot_free(left), True), (robot_free(right), True)])
    add_end_increase(coop_deliver, total_cost(), request_priority(q))
    add_action(coop_deliver)

    check_in = da('check-in-visitor', [('r', robot), ('p', person), ('l', location), ('q', request), ('terminal', resource)], Int(1))
    r, pe, l, q, terminal = [p(check_in, n) for n in ['r', 'p', 'l', 'q', 'terminal']]
    metrics.measure('build_set_durations_s', check_in.set_fixed_duration, Plus(Int(2), task_duration(q)))
    add_overall_conditions(check_in, [robot_at(r, l), person_at(pe, l)])
    add_start_conditions(check_in, [available(terminal), uses_terminal(q, terminal), robot_free(r), robot_has(r, o['admin-cap']), request_person(q, pe), request_origin(q, l), request_destination(q, l), request_capability(q, o['admin-cap'])])
    add_start_effects(check_in, [(available(terminal), False), (robot_free(r), False)])
    add_end_effects(check_in, [(checked_in(pe), True), (service_completed(q), True), (available(terminal), True), (robot_free(r), True)])
    add_end_increase(check_in, total_cost(), request_priority(q))
    add_action(check_in)

    guide = da('guide-person', [('r', robot), ('p', person), ('from', location), ('to', location), ('q', request)], Int(1))
    r, pe, fr, to, q = [p(guide, n) for n in ['r', 'p', 'from', 'to', 'q']]
    metrics.measure('build_set_durations_s', guide.set_fixed_duration, Plus(Int(3), Div(Int(3), speed(r))))
    add_start_conditions(guide, [robot_at(r, fr), person_at(pe, fr), connected(fr, to), can_move(r, fr, to), Not(blocked(fr, to)), can_enter(r, to), robot_free(r), robot_has(r, o['guide-cap']), can_guide(r, q), request_person(q, pe), request_origin(q, fr), request_destination(q, to), request_capability(q, o['guide-cap']), GE(battery(r), Times(Int(3), energy_rate(r)))])
    add_overall_conditions(guide, [Not(blocked(fr, to))])
    add_start_effects(guide, [(robot_free(r), False), (robot_at(r, fr), False), (person_at(pe, fr), False)])
    add_start_decrease(guide, battery(r), Times(Int(3), energy_rate(r)))
    add_end_effects(guide, [(robot_at(r, to), True), (person_at(pe, to), True), (guided(pe, to), True), (service_completed(q), True), (robot_free(r), True)])
    add_end_increase(guide, total_cost(), request_priority(q))
    add_action(guide)

    comfort = da('comfort-person', [('r', robot), ('p', person), ('l', location), ('q', request)], Int(1))
    r, pe, l, q = [p(comfort, n) for n in ['r', 'p', 'l', 'q']]
    metrics.measure('build_set_durations_s', comfort.set_fixed_duration, Plus(Int(3), task_duration(q)))
    add_overall_conditions(comfort, [robot_at(r, l), person_at(pe, l)])
    add_start_conditions(comfort, [robot_free(r), robot_has(r, o['social-cap']), can_comfort(r, q), request_person(q, pe), request_origin(q, l), request_destination(q, l), request_capability(q, o['social-cap'])])
    add_start_effects(comfort, [(robot_free(r), False)])
    add_end_effects(comfort, [(comforted(pe), True), (service_completed(q), True), (robot_free(r), True)])
    add_end_increase(comfort, total_cost(), request_priority(q))
    add_action(comfort)

    remote = da('remote-assist', [('monitor', robot), ('actor', robot), ('p', person), ('station', location), ('l', location), ('q', request), ('terminal', resource)], Int(1))
    monitor, actor, pe, station, l, q, terminal = [p(remote, n) for n in ['monitor', 'actor', 'p', 'station', 'l', 'q', 'terminal']]
    metrics.measure('build_set_durations_s', remote.set_fixed_duration, Plus(Int(4), task_duration(q)))
    add_overall_conditions(remote, [robot_at(monitor, station), robot_at(actor, l), person_at(pe, l)])
    add_start_conditions(remote, [available(terminal), uses_terminal(q, terminal), robot_free(monitor), robot_free(actor), robot_has(monitor, o['monitor-cap']), robot_has(actor, o['social-cap']), can_assist(monitor, actor, q), request_person(q, pe), request_origin(q, station), request_destination(q, l), request_capability(q, o['monitor-cap']), GE(battery(monitor), Int(4))])
    add_start_effects(remote, [(available(terminal), False), (robot_free(monitor), False), (robot_free(actor), False)])
    add_start_decrease(remote, battery(monitor), Int(4))
    add_end_effects(remote, [(assisted(pe), True), (service_completed(q), True), (available(terminal), True), (robot_free(monitor), True), (robot_free(actor), True)])
    add_end_increase(remote, total_cost(), request_priority(q))
    add_action(remote)

    def set_true(expr):
        metrics.measure('build_set_initial_values_s', problem.set_initial_value, expr, True)
        metrics.inc('build_set_initial_value_calls')

    def set_num(expr, value):
        metrics.measure('build_set_initial_values_s', problem.set_initial_value, expr, real(value))
        metrics.inc('build_set_initial_value_calls')

    set_num(total_cost(), 0)

    for r_name, l_name in [('r-admin', 'reception'), ('r-pharmacy', 'pharmacy'), ('r-ward', 'pharmacy'), ('r-guide', 'reception'), ('r-social', 'laboratory'), ('r-remote', 'charge-a'), ('r-hybrid', 'laboratory')]:
        set_true(robot_at(o[r_name], o[l_name]))
        set_true(robot_free(o[r_name]))

    for p_name, l_name in [('visitor-a', 'reception'), ('visitor-b', 'reception'), ('resident-a', 'laboratory'), ('resident-b', 'pharmacy'), ('resident-c', 'social-room'), ('resident-e', 'laboratory'), ('resident-f', 'ward-2')]:
        set_true(person_at(o[p_name], o[l_name]))

    set_true(carrying(o['r-hybrid'], o['med-a']))
    for i_name, l_name in [('med-b', 'pharmacy'), ('oxygen-unit', 'laboratory'), ('meal-tray', 'pharmacy')]:
        set_true(item_at(o[i_name], o[l_name]))

    for res in ['terminal-1', 'elevator-1', 'dock-a', 'dock-b']:
        set_true(available(o[res]))
    set_true(charger_for(o['dock-a'], o['charge-a']))
    set_true(charger_for(o['dock-b'], o['charge-b']))

    for a, b in [('reception', 'ward-1'), ('ward-1', 'reception'), ('pharmacy', 'central-corridor'), ('central-corridor', 'pharmacy'), ('central-corridor', 'ward-2'), ('ward-2', 'central-corridor'), ('pharmacy', 'ward-1'), ('ward-1', 'pharmacy'), ('laboratory', 'social-room'), ('social-room', 'laboratory')]:
        set_true(connected(o[a], o[b]))
    for a, b in [('central-corridor', 'ward-2'), ('ward-2', 'central-corridor')]:
        set_true(corridor_link(o[a], o[b]))
        set_true(uses_corridor(o[a], o[b], o['elevator-1']))

    for r_name, caps in {
        'r-admin': ['admin-cap'],
        'r-pharmacy': ['nav-cap', 'delivery-cap'],
        'r-ward': ['nav-cap', 'delivery-cap', 'social-cap'],
        'r-guide': ['nav-cap', 'guide-cap', 'social-cap'],
        'r-social': ['nav-cap', 'social-cap', 'lift-right-cap'],
        'r-remote': ['nav-cap', 'monitor-cap'],
        'r-hybrid': ['nav-cap', 'delivery-cap', 'social-cap', 'lift-left-cap'],
    }.items():
        for cap in caps:
            set_true(robot_has(o[r_name], o[cap]))

    for r_name, locations in {
        'r-admin': ['reception', 'central-corridor'],
        'r-pharmacy': ['pharmacy', 'ward-1', 'charge-a'],
        'r-ward': ['pharmacy', 'central-corridor', 'ward-2', 'charge-b'],
        'r-guide': ['reception', 'central-corridor', 'ward-1', 'social-room', 'charge-a'],
        'r-social': ['laboratory', 'social-room', 'central-corridor', 'ward-2', 'charge-b'],
        'r-remote': ['charge-a'],
        'r-hybrid': ['laboratory', 'social-room', 'charge-a'],
    }.items():
        for loc in locations:
            set_true(can_enter(o[r_name], o[loc]))

    for r_name, a, b in [
        ('r-pharmacy', 'pharmacy', 'ward-1'), ('r-ward', 'pharmacy', 'central-corridor'),
        ('r-ward', 'central-corridor', 'ward-2'), ('r-guide', 'reception', 'ward-1'),
        ('r-social', 'laboratory', 'social-room'), ('r-hybrid', 'laboratory', 'social-room'),
    ]:
        set_true(can_move(o[r_name], o[a], o[b]))

    for item_name in ['med-a', 'med-b']:
        set_true(can_pick(o['r-pharmacy'], o[item_name]))
    set_true(can_deliver(o['r-ward'], o['q-med-handover']))
    set_true(can_guide(o['r-guide'], o['q-guide']))
    set_true(can_comfort(o['r-social'], o['q-wellbeing']))
    set_true(can_comfort(o['r-hybrid'], o['q-med-normal']))
    set_true(can_comfort(o['r-hybrid'], o['q-social-urgent']))
    set_true(can_assist(o['r-remote'], o['r-ward'], o['q-remote']))
    set_true(can_handover(o['r-pharmacy'], o['r-ward'], o['med-b'], o['pharmacy']))
    set_true(can_cooperate(o['r-hybrid'], o['r-social'], o['oxygen-unit']))
    set_true(uses_terminal(o['q-checkin'], o['terminal-1']))
    set_true(uses_terminal(o['q-remote'], o['terminal-1']))

    for q_name, p_name, origin, dest, cap in [
        ('q-checkin', 'visitor-a', 'reception', 'reception', 'admin-cap'),
        ('q-guide', 'visitor-b', 'reception', 'ward-1', 'guide-cap'),
        ('q-wellbeing', 'resident-c', 'social-room', 'social-room', 'social-cap'),
        ('q-remote', 'resident-b', 'charge-a', 'pharmacy', 'monitor-cap'),
        ('q-med-normal', 'resident-a', 'laboratory', 'laboratory', 'social-cap'),
        ('q-med-handover', 'resident-b', 'pharmacy', 'pharmacy', 'delivery-cap'),
        ('q-heavy', 'resident-c', 'laboratory', 'social-room', 'lift-left-cap'),
        ('q-social-urgent', 'resident-e', 'laboratory', 'laboratory', 'social-cap'),
    ]:
        set_true(request_person(o[q_name], o[p_name]))
        set_true(request_origin(o[q_name], o[origin]))
        set_true(request_destination(o[q_name], o[dest]))
        set_true(request_capability(o[q_name], o[cap]))
    set_true(request_item(o['q-med-handover'], o['med-b']))
    set_true(request_item(o['q-heavy'], o['oxygen-unit']))

    for r_name, values in {
        'r-admin': (45, 70, 1.1, 0.8, 1, 0),
        'r-pharmacy': (42, 80, 0.9, 1.0, 3, 0),
        'r-ward': (55, 90, 0.8, 1.1, 4, 0),
        'r-guide': (50, 70, 1.0, 0.9, 1, 0),
        'r-social': (60, 85, 0.9, 1.1, 4, 0),
        'r-remote': (2, 80, 1.0, 1.0, 1, 0),
        'r-hybrid': (65, 90, 1.1, 1.0, 5, 1),
    }.items():
        b, bmax, spd, erate, capacity, load = values
        set_num(battery(o[r_name]), b)
        set_num(battery_max(o[r_name]), bmax)
        set_num(speed(o[r_name]), spd)
        set_num(energy_rate(o[r_name]), erate)
        set_num(payload_capacity(o[r_name]), capacity)
        set_num(current_load(o[r_name]), load)

    for i_name, weight, size in [('med-a', 1, 1), ('med-b', 1, 1), ('oxygen-unit', 9, 5), ('meal-tray', 2, 2)]:
        set_num(item_weight(o[i_name]), weight)
        set_num(item_size(o[i_name]), size)

    for q_name, priority, duration in [
        ('q-checkin', 2, 1), ('q-guide', 6, 2), ('q-wellbeing', 8, 2),
        ('q-remote', 25, 2), ('q-med-normal', 10, 1), ('q-med-handover', 18, 1),
        ('q-heavy', 14, 2), ('q-social-urgent', 20, 1),
    ]:
        set_num(request_priority(o[q_name]), priority)
        set_num(task_duration(o[q_name]), duration)

    goals = [
        checked_in(o['visitor-a']), guided(o['visitor-b'], o['ward-1']), comforted(o['resident-c']),
        assisted(o['resident-b']), comforted(o['resident-a']), handover_done(o['med-b'], o['r-pharmacy'], o['r-ward']),
        delivered(o['med-b'], o['resident-b']), delivered(o['oxygen-unit'], o['resident-c']), comforted(o['resident-e']),
        service_completed(o['q-checkin']), service_completed(o['q-guide']), service_completed(o['q-wellbeing']),
        service_completed(o['q-remote']), service_completed(o['q-med-normal']), service_completed(o['q-med-handover']),
        service_completed(o['q-heavy']), service_completed(o['q-social-urgent']),
    ]
    metrics.measure('build_add_goals_s', problem.add_goal, And(*goals))
    metrics.inc('build_add_goal_calls')

    return problem


class TamerIntegratedNodeDemo(Node):
    def __init__(self):
        super().__init__('upf4ros2_demo_tamer_integrated_node')
        self._writer = ROS2InterfaceWriter()
        self._plan_client = self.create_client(PlanOneShotSrv, 'upf4ros2/srv/planOneShot')

    def solve(self, problem, wait_service_timeout_s):
        row = {}
        start = now()
        service_ready = self._plan_client.wait_for_service(timeout_sec=wait_service_timeout_s)
        row['wait_service_s'] = now() - start
        if not service_ready:
            raise RuntimeError('upf4ros2/srv/planOneShot is not available')

        request = PlanOneShotSrv.Request()
        start = now()
        request.problem = self._writer.convert(problem)
        row['serialize_problem_s'] = now() - start

        self.get_logger().info('Planning with a node-built UPF Problem through upf4ros2/srv/planOneShot...')
        start = now()
        future = self._plan_client.call_async(request)
        row['service_call_submit_s'] = now() - start

        start = now()
        rclpy.spin_until_future_complete(self, future)
        row['service_wait_response_s'] = now() - start

        start = now()
        response = future.result()
        row['response_extract_s'] = now() - start
        self.get_logger().info(f'success={response.success}')
        start = now()
        for action in response.plan_result.plan.actions:
            params = [x.symbol_atom[0] for x in action.parameters]
            self.get_logger().info(action.action_name + '(' + ', '.join(params) + ')')
        row['plan_log_s'] = now() - start
        row['success'] = response.success
        row['plan_action_count'] = len(response.plan_result.plan.actions)
        return response, row


def build_row(run_id, node, args):
    row = {'run_id': run_id, 'configuration': 'upf4ros2_node_problem'}
    total_start = now()
    cpu_before, maxrss_before = read_self_resource()
    server_pids = find_upf4ros2_server_pids()
    server_cpu_before = sum_proc_cpu(server_pids)
    server_rss_before = sum_proc_rss_kb(server_pids)

    metrics = Metrics()
    start = now()
    problem = build_tamer_complex_integrated_problem(metrics)
    row['build_total_s'] = now() - start
    row.update(metrics.values)
    row['problem_kind'] = str(problem.kind).replace('\n', ' | ')
    row['actions'] = len(list(problem.actions))
    row['objects'] = len(list(problem.all_objects))
    row['fluents'] = len(list(problem.fluents))
    row['goals'] = len(problem.goals)

    try:
        _, solve_row = node.solve(problem, args.wait_service_timeout_s)
        row.update(solve_row)
    except Exception as error:
        row['success'] = False
        row['error'] = repr(error)

    cpu_after, maxrss_after = read_self_resource()
    server_cpu_after = sum_proc_cpu(server_pids)
    server_rss_after = sum_proc_rss_kb(server_pids)
    row['cpu_s_delta'] = None if cpu_before is None or cpu_after is None else cpu_after - cpu_before
    row['server_cpu_s_delta'] = server_cpu_after - server_cpu_before
    row['server_rss_kb_before'] = server_rss_before
    row['server_rss_kb_after'] = server_rss_after
    row['server_pids'] = ' '.join(str(pid) for pid in server_pids)
    row['maxrss_kb_before'] = maxrss_before
    row['maxrss_kb_after'] = maxrss_after
    row['loadavg_1m'] = os.getloadavg()[0]
    row['total_s'] = now() - total_start
    return row


def write_csv(rows, output):
    fieldnames = sorted({key for row in rows for key in row.keys()})
    with open(output, 'w', newline='') as stream:
        writer = csv.DictWriter(stream, fieldnames=fieldnames)
        writer.writeheader()
        writer.writerows(rows)


def main(args=None):
    parser = argparse.ArgumentParser(description='UPF4ROS2 benchmark with the integrated Tamer problem built in the node')
    parser.add_argument('--runs', type=int, default=50)
    parser.add_argument('--output', default='tamer_integrated_node_upf4ros2.csv')
    parser.add_argument('--wait-service-timeout-s', type=float, default=20.0)
    parsed_args, ros_args = parser.parse_known_args(args)

    get_environment().credits_stream = None
    rclpy.init(args=ros_args)
    node = TamerIntegratedNodeDemo()
    rows = []
    try:
        for run_id in range(1, parsed_args.runs + 1):
            node.get_logger().info(f'Benchmark run {run_id}/{parsed_args.runs}')
            row = build_row(run_id, node, parsed_args)
            rows.append(row)
            node.get_logger().info(
                f'run={run_id} success={row.get("success")} '
                f'total_s={row.get("total_s", 0):.3f} '
                f'build_s={row.get("build_total_s", 0):.3f} '
                f'service_wait_s={row.get("service_wait_response_s", 0):.3f}')
        write_csv(rows, parsed_args.output)
        node.get_logger().info(f'wrote {parsed_args.output}')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
