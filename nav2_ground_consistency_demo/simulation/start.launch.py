"""
Launch file for nav2_ground_consistency_costmap_plugin demo.

Launches Gazebo simulation with Husky robot and ROS 2 bridge for pointcloud.

Usage:
  ros2 launch nav2_ground_consistency_demo start.launch.py
"""

import yaml
import os
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch_ros.actions import Node  
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution
from launch.actions import OpaqueFunction, DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable
from launch.substitutions import Command, FindExecutable, LaunchConfiguration
from launch.conditions import IfCondition  
from launch.launch_description_sources import PythonLaunchDescriptionSource

def load_yaml_file(yaml_file_path):
    try:
        with open(yaml_file_path, 'r') as file:
            return yaml.load(file)
    except EnvironmentError as e: 
        print(str(e))
        return None    

def launch_setup(context, *args, **kwargs):
  """Setup and launch Gazebo simulation with Husky robot and ROS 2 bridge."""
  
  # Get configuration parameters
  world_file_name = str(LaunchConfiguration('world_file_name').perform(context))
  robot_name = str(LaunchConfiguration('robot_name').perform(context))

  # Get package share directory using ROS2 mechanisms
  pkg_share = get_package_share_directory('nav2_ground_consistency_demo')
  
  # Construct Gazebo arguments: load the selected world SDF file with verbose logging
  ign_args = '-v 4 -r ' + os.path.join(pkg_share, 'simulation', 'models', world_file_name + '.sdf')
  
  # Add GUI config
  gui_config_path = os.path.join(pkg_share, 'simulation', 'config', 'gazebo_gui.config')
  if os.path.exists(gui_config_path):
    ign_args += ' --gui-config ' + gui_config_path

  # Launch Gazebo with the selected world
  gazebo_launch_description = IncludeLaunchDescription(
	PythonLaunchDescriptionSource(
		os.path.join(
			get_package_share_directory('ros_gz_sim'),
			'launch',
			'gz_sim.launch.py'
		)
	),
	launch_arguments={
		'gz_args': ign_args
	}.items()
  )

  # Launch ROS 2 <-> Gazebo bridge for topics (cmd_vel, lidar, IMU, clock)
  # Bridge configuration based on world name and sensor paths
  bridge_args = [
    '/model/husky/cmd_vel@geometry_msgs/msg/Twist@gz.msgs.Twist',
    f'/world/{world_file_name}/model/husky/link/base_link/sensor/front_laser/scan/points@sensor_msgs/msg/PointCloud2@gz.msgs.PointCloudPacked',
    f'/world/{world_file_name}/clock@rosgraph_msgs/msg/Clock@gz.msgs.Clock'
  ]
  
  ign_ros2_bridge = Node(
    package='ros_gz_bridge',
    executable='parameter_bridge',
    arguments=bridge_args,
    remappings=[
      ('/model/husky/cmd_vel', '/cmd_vel'),
      (f'/world/{world_file_name}/clock', '/clock'),
      (f'/world/{world_file_name}/model/husky/link/base_link/sensor/front_laser/scan/points', '/husky/scan/points')
    ],
    output='both'
  )
  
  ign_ros2_bridge_description = [ign_ros2_bridge]

  # Static transform publisher: husky/base_link -> husky/base_link/front_laser
  # Position from model.sdf: x=0.0012, y=0, z=0.716 (lidar mounted 0.716m in base_link)
  static_tf_front_laser = Node(
    package='tf2_ros',
    executable='static_transform_publisher',
    arguments=['0.0012', '0', '0.716', '0', '0', '0', 'husky/base_link', 'husky/base_link/front_laser'],
    parameters=[{'use_sim_time': True}],
  )
  
  return [gazebo_launch_description, ign_ros2_bridge, static_tf_front_laser]   
  
def generate_launch_description(): 
  """Generate launch description with environment variable setup."""
  
  # Get package share directory using ROS2 mechanisms
  pkg_share = get_package_share_directory('nav2_ground_consistency_demo')
  
  # Set GZ_SIM_RESOURCE_PATH so Gazebo can find robot models and assets
  models_dir = os.path.join(pkg_share, 'simulation', 'models')
  resource_dir = os.path.join(pkg_share, 'resource')
  
  set_gz_resource_path = SetEnvironmentVariable(
    'GZ_SIM_RESOURCE_PATH',
    models_dir + ':' + resource_dir + ':$GZ_SIM_RESOURCE_PATH'
  )
       
  return LaunchDescription([
    set_gz_resource_path,

    DeclareLaunchArgument(
        "robot_name",
        default_value="husky",
        description="Robot model. Options: husky"
    ),
        
    DeclareLaunchArgument(
        "world_file_name",
        default_value="baylands_terrain",
        description="Gazebo world to load. Options: baylands_terrain"
    ),

    OpaqueFunction(function = launch_setup)
    
    ])
  