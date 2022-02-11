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
    nav2_gps_waypoint_follower_demo_dir = get_package_share_directory('nav2_gps_waypoint_follower_demo')
    parameters_file_dir = os.path.join(nav2_gps_waypoint_follower_demo_dir, 'params')
    parameters_file_path = os.path.join(parameters_file_dir, 'dual_ekf_navsat_localization.yaml')
    return LaunchDescription([
        launch_ros.actions.Node(
                package='robot_localization', 
                executable='ekf_node', 
                name='ekf_local_filter_node',
                output='screen',
                parameters=[parameters_file_path],
                remappings=[('odometry/filtered', 'odometry/local')]           
            ),
        launch_ros.actions.Node(
                package='robot_localization', 
                executable='ekf_node', 
                name='ekf_global_filter_node',
                output='screen',
                parameters=[parameters_file_path],
                remappings=[('odometry/filtered', 'odometry/global')]
            ),           
        launch_ros.actions.Node(
                package='robot_localization', 
                executable='navsat_transform_node', 
                name='navsat_transform_node',
                output='screen',
                parameters=[parameters_file_path],
                remappings=[('imu/data', 'imu/data'),
                            ('gps/fix', 'gps/fix'), 
                            ('gps/filtered', 'gps/filtered'),
                            ('odometry/gps', 'odometry/gps'),
                            ('odometry/filtered', 'odometry/global')]           
            )           
])