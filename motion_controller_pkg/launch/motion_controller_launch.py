from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    config_file = get_package_share_directory('motion_controller_pkg') + '/config/controller_config.yaml'

    return LaunchDescription([
        Node(
            package='motion_controller_pkg',
            executable='motion_controller',
            name='motion_controller',
            output='screen',
            parameters=[config_file],
            prefix='gnome-terminal --title="Motion Controller Server" -- '
        ),

        Node(
            package='motion_controller_pkg',
            executable='motion_controller_client',
            name='motion_controller_client',
            output='screen',
            prefix='gnome-terminal --title="Motion Controller Client" -- '
        )
    ])
