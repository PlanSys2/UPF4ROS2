from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():

    upf4ros2_demo_cmd = Node(package='upf4ros2_demo',
                        executable='upf4ros2_demo_navigate',
                        output='screen')

    ld = LaunchDescription()
    ld.add_action(upf4ros2_demo_cmd)

    return ld
