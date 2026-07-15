from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, EmitEvent, RegisterEventHandler
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    pkg_name = 'upf4ros2_demo'
    replanning_case = LaunchConfiguration('replanning_case')
    planner = LaunchConfiguration('planner')
    output_dir = LaunchConfiguration('output_dir')
    execute_actions = LaunchConfiguration('execute_actions')

    use_native = PythonExpression(["'", planner, "' == 'native'"])
    use_upf4ros2 = PythonExpression(["'", planner, "' == 'upf4ros2'"])

    demo = Node(
        package=pkg_name,
        executable='plansys2_replanning_cases_demo',
        name='plansys2_replanning_cases_demo',
        output='screen',
        parameters=[{
            'pipeline': PythonExpression([
                "'plansys2_upf4ros2' if '", planner,
                "' == 'upf4ros2' else 'plansys2_native_popf'"
            ]),
            'output_dir': output_dir,
            'execute_actions': execute_actions,
            'disturbance_trigger': 'on_edge_attempt',
            'blocked_edge': ['kitchen', 'gym'],
            'replanning_case': replanning_case,
            'initial_goals': ['bedroom', 'gym'],
            'changed_goals': ['room'],
            'goal_change_after_successes': 1,
        }])

    navigation_action = Node(
        package=pkg_name,
        executable='upf4ros2_navigation_action',
        name='upf4ros2_navigation_action',
        parameters=[get_package_share_directory(pkg_name) + '/params/tiago.yaml'],
        output='screen')

    return LaunchDescription([
        DeclareLaunchArgument(
            'replanning_case',
            default_value='both',
            description="Replanning case to run: 'both', 'blocked_edge', or 'goal_change'"),
        DeclareLaunchArgument(
            'planner',
            default_value='native',
            description="Planner backend: 'native' or 'upf4ros2'"),
        DeclareLaunchArgument(
            'output_dir',
            default_value='/tmp/plansys2_replanning_cases_demo',
            description='Directory for replanning event and summary files'),
        DeclareLaunchArgument(
            'execute_actions',
            default_value='true',
            description='Whether to call the navigation action service while executing the plan'),
        Node(
            package='upf4ros2',
            executable='upf4ros2_main',
            name='upf4ros2',
            output='screen',
            condition=IfCondition(use_upf4ros2)),
        Node(
            package='plansys2_planner',
            executable='planner_node',
            name='planner',
            output='screen',
            parameters=[{
                'plan_solver_plugins': ['POPF'],
                'POPF.plugin': 'plansys2/POPFPlanSolver',
            }],
            condition=IfCondition(use_native)),
        Node(
            package='plansys2_planner',
            executable='planner_node',
            name='planner',
            output='screen',
            parameters=[{
                'plan_solver_plugins': ['UPF'],
                'UPF.plugin': 'plansys2/UPFPlanSolver',
            }],
            condition=IfCondition(use_upf4ros2)),
        navigation_action,
        demo,
        RegisterEventHandler(
            OnProcessExit(
                target_action=demo,
                on_exit=[EmitEvent(event=Shutdown(reason='replanning cases demo finished'))])),
    ])
