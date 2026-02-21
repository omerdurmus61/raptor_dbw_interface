from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
import os

def generate_launch_description():
    config = os.path.join(
        get_package_share_directory('raptor_dbw_interface'),
        'config',
        'vehicle_interface_params.yaml'
    )

    use_encoder_odometry = LaunchConfiguration('use_encoder_odometry')

    return LaunchDescription([
        
        DeclareLaunchArgument(
                'use_encoder_odometry',
                default_value= 'true',
                description = 'Run encoder odometry node'
            ),

        Node(
            package='raptor_dbw_interface',
            executable='raptor_dbw_interface_node',
            name='raptor_dbw_interface',
            parameters=[config],
            output='screen'
        ),
        Node(
            package='raptor_dbw_interface',
            executable='raptor_encoder_odometry_node',
            name='raptor_encoder_odometry',
            parameters=[config],
            output='screen',
            condition=IfCondition(use_encoder_odometry)
        ),

    ])
