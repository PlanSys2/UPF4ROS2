from unified_planning.shortcuts import *
from unified_planning.io import PDDLReader, PDDLWriter
import itertools
from ament_index_python.packages import get_package_share_directory


class ProblemMSG:


    def __init__(self,monitorsg,horizon):
        self.monsg = monitorsg
        self.policy=self.monsg.sg.backwardRecursion(horizon)
        reader = PDDLReader()
        self.domain=reader.parse_problem(get_package_share_directory('upf4ros2_demo')+'/pddl/monitor_domain.pddl')



    def problem_gen(self,state,horizon):
        drone_type = self.domain.user_type("drone")
        waypoint_type = self.domain.user_type("waypoint")

        at_pred = self.domain.fluent("at")
        landed_pred = self.domain.fluent("landed")
        takoff_pred = self.domain.fluent("taken_off")
        inspected_pred = self.domain.fluent("inspected")
        # self.domain.set_initial_value(self.domain.fluent("inspected"), False)
        # self.domain.set_initial_value(self.domain.fluent("at"), False)
        # self.domain.set_initial_value(self.domain.at("inspected"), False)
        # for n in range(self.monsg.n_players):
        state_i=list(itertools.product(
            *[range(2) for m in range(self.monsg.m_sites)])).index(state)
        state_policy=self.policy[horizon][state_i]
        problems=[]
        for n in range(self.monsg.n_players):
            strat_n=state_policy[n]
            played_act=strat_n.index(1)
            problem = self.domain.clone()

            # problem=unified_planning.model.Problem('monitor1')
            # problem.add_fluent(at_pred, default_initial_value=False)
            # problem.add_fluent(landed_pred, default_initial_value=False)
            # problem.add_fluent(takoff_pred, default_initial_value=False)
            # problem.add_fluent(inspected_pred, default_initial_value=False)
            # problem.add_actions(self.domain.actions)

            # drone_homes=[unified_planning.model.Object(f"h{n}", waypoint_type) for n in range(self.monsg.n_players)]
            # problem.add_objects(drone_homes)
            # drones=[unified_planning.model.Object(f"d{n}", drone_type) for n in range(self.monsg.n_players)]
            # problem.add_objects(drones)
            home = problem.add_object(f"home{n}", waypoint_type)
            drone = problem.add_object(f"d{n}", drone_type)
            waypoints = [unified_planning.model.Object(f"w{m}", waypoint_type) for m
                         in range(self.monsg.m_sites)]
            problem.add_objects(waypoints)



            problem.set_initial_value(at_pred(drone, home), True)
            problem.set_initial_value(landed_pred(drone), True)
            # problem.set_initial_value(at_pred(d1), False)

            problem.add_goal(at_pred(drone, home))
            problem.add_goal(landed_pred(drone))
            problem.add_goal(inspected_pred(waypoints[played_act]))

            problems+=[problem]
            w=PDDLWriter(problem)
            print(w.get_domain())
            print(w.get_problem())
            w.write_problem(get_package_share_directory('upf4ros2_demo')+ f"/pddl/monitorproblem_{n}.pddl")
            print("-------------------------------")
        w = PDDLWriter(problems[0])
        w.write_domain(get_package_share_directory('upf4ros2_demo') + "/pddl/monitor_domain_upf.pddl")

        # with OneshotPlanner(problem_kind=problem.kind) as planner:
        #     result = planner.solve(problem)
        #     print("%s returned: %s" % (planner.name, result.plan))
        # print(problems)
