from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    config_file = get_package_share_directory('motion_controller_pkg') + '/config/controller_config.yaml'

    return LaunchDescription([
        Node(
            package='motion_controller_pkg',
            executable='motion_controller',
            name='motion_controller',
            output='screen',
            parameters=[config_file]
        )
    ])
