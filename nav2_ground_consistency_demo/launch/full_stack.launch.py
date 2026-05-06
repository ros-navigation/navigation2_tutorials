"""
Nav2 navigation stack launch for nav2_ground_consistency_demo.

Usage:
  ros2 launch nav2_ground_consistency_demo full_stack.launch.py
"""

import os
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, OpaqueFunction, SetEnvironmentVariable
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def gazebo_launch_setup(context, *args, **kwargs):
    """Setup and launch Gazebo simulation with Husky robot and terrain."""
    
    world_file_name = str(LaunchConfiguration('world_file_name').perform(context))
    pkg_share = get_package_share_directory('nav2_ground_consistency_demo')
    
    # Gazebo world and GUI config paths
    world_sdf_path = os.path.join(pkg_share, 'models', world_file_name + '.sdf')
    gui_config_path = os.path.join(pkg_share, 'config', 'gazebo_gui.config')
    
    # Construct Gazebo launch arguments
    ign_args = '-v 4 -r ' + world_sdf_path
    if os.path.exists(gui_config_path):
        ign_args += ' --gui-config ' + gui_config_path
    
    # Launch Gazebo
    gazebo_launch = IncludeLaunchDescription(
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
    
    # ROS 2 <-> Gazebo bridge
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
    
    # Static transform: base_link -> front_laser
    static_tf_front_laser = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=['0.0012', '0', '0.716', '0', '0', '0', 'husky/base_link', 'husky/base_link/front_laser'],
        parameters=[{'use_sim_time': True}],
    )
    
    return [gazebo_launch, ign_ros2_bridge, static_tf_front_laser]

def generate_launch_description():
    """Generate complete launch description."""
    
    # Get package directories
    nav2_demo_dir = FindPackageShare("nav2_ground_consistency_demo")
    nav2_bringup_dir = FindPackageShare("nav2_bringup")
    kiss_icp_dir = FindPackageShare("kiss_icp")
    ground_seg_dir = FindPackageShare("ground_segmentation_ros2")
    
    # Get package share for resource path
    pkg_share = get_package_share_directory('nav2_ground_consistency_demo')
    models_dir = os.path.join(pkg_share, 'models')
    
    # Set Gazebo resource path
    set_gz_resource_path = SetEnvironmentVariable(
        'GZ_SIM_RESOURCE_PATH',
        models_dir + ':$GZ_SIM_RESOURCE_PATH'
    )
    
    # KISS-ICP odometry
    kiss_icp_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [kiss_icp_dir, "/launch/odometry.launch.py"]
        ),
        launch_arguments={
            "topic": "/husky/scan/points",
            "base_frame": "husky/base_link",
            "lidar_odom_frame": "odom",
            "invert_odom_tf": "False", 
            "visualize": "False",
            "config_file": PathJoinSubstitution(
                [nav2_demo_dir, "config/kiss_icp_config.yaml"]
            ),
            "use_sim_time": "True",
        }.items()
    )
    
    # Ground segmentation
    ground_seg_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [ground_seg_dir, "/launch/ground_segmentation.launch.py"]
        ),
        launch_arguments={
            "pointcloud_topic": "/husky/scan/points",
            "params_file": PathJoinSubstitution(
                [nav2_demo_dir, "config/gseg3d_config.yaml"]
            ),
            "use_sim_time": "True",
        }.items()
    )
    
    # Nav2 bringup
    nav2_bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([nav2_bringup_dir, "launch", "bringup_launch.py"])
        ),
        launch_arguments={
            "use_sim_time": "True",
            "slam": "False",
            "use_localization": "False",
            "autostart": "True",
            "use_composition": "False",
            "use_respawn": "False",
            "params_file": PathJoinSubstitution([nav2_demo_dir, "config", "nav2_config.yaml"]),
        }.items(),
    )
    
    # Static map -> odom transform
    map_to_odom_tf = Node(
        package="tf2_ros",
        output="screen",
        executable="static_transform_publisher",
        arguments=["0", "0", "0", "0", "0", "0", "map", "odom"],
        parameters=[{"use_sim_time": True}],
    )
    
    # RViz2 visualization
    rviz_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([nav2_bringup_dir, "launch", "rviz_launch.py"])
        ),
        launch_arguments={
            "use_sim_time": "True",
        }.items()
    )
    
    return LaunchDescription([
        set_gz_resource_path,
        
        DeclareLaunchArgument(
            "world_file_name",
            default_value="baylands_terrain",
            description="Gazebo world to load"
        ),
        
        OpaqueFunction(function=gazebo_launch_setup),
        kiss_icp_launch,
        ground_seg_launch,
        map_to_odom_tf,
        nav2_bringup,
        rviz_launch,
    ])
