from launch import LaunchDescription
import launch_ros.actions
import os
import yaml
from launch.substitutions import EnvironmentVariable
import pathlib
import launch.actions
from launch.actions import DeclareLaunchArgument
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    return LaunchDescription([
        launch_ros.actions.Node(
                package='nav2_gps_waypoint_follower_demo', 
                executable='gps_waypoint_collector', 
                name='gps_waypoint_collector_node',
                output='screen',
                remappings=[('/gps', '/gps/fix'),
                            ('/imu', '/imu/data')]
            )               
])
