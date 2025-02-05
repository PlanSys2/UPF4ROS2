from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():

    upf4ros2_pddl_cmd = Node(package='upf4ros2_demo',
                             executable='upf4ros2_demo1_pddl',
                             output='screen',
                             parameters=[
                                 {'domain': '/pddl/domain.pddl'},
                                 {'problem': '/pddl/problem.pddl'}
                             ])

    ld = LaunchDescription()
    ld.add_action(upf4ros2_pddl_cmd)

    return ld
