from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    mission_file = get_package_share_directory('behaviour_trees_pkg') + '/config/parallel_mission_02.yaml'

    return LaunchDescription([
        Node(
            package='behaviour_trees_pkg',
            executable='arming_server',
            name='arming',
            output='screen',
            prefix='gnome-terminal --title="Arming Server" -- '
        ),

        Node(
            package='behaviour_trees_pkg',
            executable='offboard_server',
            name='offboarding',
            output='screen',
            prefix='gnome-terminal --title="Offboard Server" -- '
        ),

        Node(
            package='behaviour_trees_pkg',
            executable='takeoff_server',
            name='take_off',
            output='screen',
            prefix='gnome-terminal --title="Take Off Server" -- '
        ),

        Node(
            package='behaviour_trees_pkg',
            executable='follow_traj_server',
            name='follow_traj',
            output='screen',
            prefix='gnome-terminal --title="Follow Trajectory Server" -- '
        ),

        Node(
            package='behaviour_trees_pkg',
            executable='land_server',
            name='land',
            output='screen',
            prefix='gnome-terminal --title="Land Server" -- '
        ),

        Node(
            package='as2_behavior_tree',
            executable='as2_behavior_tree_node',
            name='bt_manager',
            output='screen',
            prefix='gnome-terminal --title="Behavior Tree Manager" -- ',
            parameters=[{
                'tree': mission_file,
                'use_groot': False,                 # Set to True if you want to use Groot
                'groot_client_port': 1666,
                'groot_server_port': 1667,
                'server_timeout': 10000,
                'bt_loop_duration': 10,
                'wait_for_service_timeout': 5000
            }]
        )
    ])
