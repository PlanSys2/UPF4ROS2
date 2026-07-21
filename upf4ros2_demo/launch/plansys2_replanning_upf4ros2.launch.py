from launch import LaunchDescription
from launch.actions import EmitEvent, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    pkg_name = 'upf4ros2_demo'

    demo = Node(
        package='upf4ros2_demo',
        executable='plansys2_replanning_demo',
        name='plansys2_replanning_demo',
        output='screen',
        parameters=[{
            'pipeline': 'plansys2_upf4ros2',
            'output_dir': '/tmp/plansys2_replanning_demo/upf4ros2',
            'execute_actions': True,
            'disturbance_trigger': 'on_first_action_attempt',
        }])

    navigation_action = Node(
        package=pkg_name,
        executable='upf4ros2_navigation_action',
        name='upf4ros2_navigation_action',
        parameters=[get_package_share_directory(pkg_name) + '/params/tiago.yaml'],
        output='screen')

    return LaunchDescription([
        Node(
            package='upf4ros2',
            executable='upf4ros2_main',
            name='upf4ros2',
            output='screen'),
        Node(
            package='plansys2_planner',
            executable='planner_node',
            name='planner',
            output='screen',
            parameters=[{
                'plan_solver_plugins': ['UPF'],
                'UPF.plugin': 'plansys2/UPFPlanSolver',
            }]),
        navigation_action,
        demo,
        RegisterEventHandler(
            OnProcessExit(
                target_action=demo,
                on_exit=[EmitEvent(event=Shutdown(reason='replanning demo finished'))])),
    ])
