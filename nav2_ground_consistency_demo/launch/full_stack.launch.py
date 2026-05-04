"""
Nav2 navigation stack launch for nav2_ground_consistency_demo.

Usage:
  ros2 launch nav2_ground_consistency_demo full_stack.launch.py
"""

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.conditions import IfCondition
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    
    # Get package directories
    nav2_demo_dir = FindPackageShare("nav2_ground_consistency_demo")
    nav2_bringup_dir = FindPackageShare("nav2_bringup")
    kiss_icp_dir = FindPackageShare("kiss_icp")
    ground_seg_dir = FindPackageShare("ground_segmentation_ros2")
    
    # Gazebo simulation launch (includes Husky robot and terrain)
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([nav2_demo_dir, "simulation/start.launch.py"])
        ),
        launch_arguments={
            "world_file_name": "baylands_terrain",
        }.items()
    )
    
    # KISS-ICP odometry (provides odom frame and TF transforms)
    # This replaces standard Nav2 localization for this demo
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
    
    # Ground segmentation (labels points as ground or obstacle)
    ground_seg_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [ground_seg_dir, "/launch/ground_segmentation.launch.py"]
        ),
        launch_arguments={
            "pointcloud_topic": "/husky/scan/points",
            "imu_topic": "/husky/imu",
            "params_file": PathJoinSubstitution(
                [nav2_demo_dir, "config/gseg3d_config.yaml"]
            ),
            "use_sim_time": "True",
        }.items()
    )
    
    # Nav2 bringup launch with ground consistency configuration
    nav2_bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([nav2_bringup_dir, "launch", "bringup_launch.py"])
        ),
        launch_arguments={
            "use_sim_time": "True",
            "slam": "False",
            "autostart": "True",
            "use_composition": "False",
            "use_respawn": "False",
            "params_file": PathJoinSubstitution([nav2_demo_dir, "config", "nav2_config.yaml"]),
        }.items(),
    )
    
    # Static map -> odom transform (since we don't have a map provider)
    map_to_odom_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        arguments=["0", "0", "0", "0", "0", "0", "map", "odom"],
        parameters=[{"use_sim_time": True}],
        output="screen",
    )
    
    # RViz2 visualization (optional, controlled by launch parameter)
    rviz = Node(
        package="rviz2",
        executable="rviz2",
        arguments=[
            "-d", PathJoinSubstitution([nav2_demo_dir, "config", "config.rviz"])
        ],
        parameters=[{"use_sim_time": True}],
        condition=IfCondition(LaunchConfiguration("rviz", default="true")),
        output="screen"
    )
    
    return LaunchDescription([
        DeclareLaunchArgument(
            "rviz",
            default_value="true",
            description="Start RViz2"
        ),
        
        gazebo_launch,
        kiss_icp_launch,
        ground_seg_launch,
        map_to_odom_tf,
        nav2_bringup,
        rviz,
    ])
