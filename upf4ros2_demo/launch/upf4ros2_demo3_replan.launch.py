from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    pkg_name = 'upf4ros2_demo'

    points = LaunchConfiguration('wps')
    execute_actions = LaunchConfiguration('execute_actions')
    simulate_block_first_move = LaunchConfiguration('simulate_block_first_move')
    max_replans = LaunchConfiguration('max_replans')

    return LaunchDescription([
        DeclareLaunchArgument(
            'wps',
            default_value=get_package_share_directory(pkg_name) + '/params/house.yaml',
            description='YAML waypoints file'),
        DeclareLaunchArgument(
            'execute_actions',
            default_value='true',
            description='Call move/check_wp action services instead of dry-running effects'),
        DeclareLaunchArgument(
            'simulate_block_first_move',
            default_value='true',
            description='Force the first move to fail once and trigger replanning'),
        DeclareLaunchArgument(
            'max_replans',
            default_value='3',
            description='Maximum number of replanning attempts'),
        Node(
            package='upf4ros2',
            executable='upf4ros2_main',
            name='upf4ros2',
            output='screen'),
        Node(
            package=pkg_name,
            executable='upf4ros2_navigation_action',
            name='upf4ros2_navigation_action',
            parameters=[points],
            output='screen'),
        Node(
            package=pkg_name,
            executable='upf4ros2_check_wp_action',
            name='upf4ros2_check_wp_action',
            output='screen'),
        Node(
            package=pkg_name,
            executable='upf4ros2_demo3_replan',
            name='upf4ros2_demo3_replan',
            parameters=[{
                'execute_actions': execute_actions,
                'simulate_block_first_move': simulate_block_first_move,
                'max_replans': max_replans,
            }],
            output='screen'),
    ])
