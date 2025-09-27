from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    config = os.path.join(
        get_package_share_directory('raptor_dbw_interface'),
        'config',
        'vehicle_interface_params.yaml'
    )

    return LaunchDescription([
        Node(
            package='raptor_dbw_interface',
            executable='raptor_dbw_interface_node',
            name='raptor_dbw_interface',
            parameters=[config],
            output='screen'
        )
    ])
