"""
Full navigation stack launch for nav2_ground_consistency_demo.

Launches all required components:
1. KISS-ICP odometry node (subscribes to lidar, provides odom->base_link TF)
2. Ground segmentation node (classifies points into ground/obstacle)
3. NAV2 controller_server with ground consistency costmap layer

Usage:
  ros2 launch nav2_ground_consistency_demo full_stack.launch.py

The data flow:
  /husky/scan/points (from simulation)
    ├── KISS-ICP → /tf (odom->base_link), /nav_msgs/Odometry
    └── Ground Seg → /ground_points, /obstacle_points
         └── NAV2 Ground Consistency Layer → local costmap → controller_server
"""

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, ExecuteProcess, TimerAction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.conditions import IfCondition
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    
    # Get package directories
    nav2_demo_dir = FindPackageShare("nav2_ground_consistency_demo")
    kiss_icp_dir = FindPackageShare("kiss_icp")
    ground_seg_dir = FindPackageShare("ground_segmentation_ros2")
    
    # KISS-ICP odometry (provides odom frame and TF transforms)
    kiss_icp_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [kiss_icp_dir, "/launch/odometry.launch.py"]
        ),
        launch_arguments={
            "topic": "/husky/scan/points",
            "base_frame": "husky/base_link",
            "lidar_odom_frame": "odom",
            "invert_odom_tf": "false", 
            "visualize": "false",
            "config_file": PathJoinSubstitution(
                [nav2_demo_dir, "config/kiss_icp_config.yaml"]
            ),
            "use_sim_time": "true",
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
            "use_sim_time": "true",
        }.items()
    )
    
    # Controller server with ground consistency costmap layer
    controller_server = Node(
        package="nav2_controller",
        executable="controller_server",
        output="screen",
        parameters=[
            PathJoinSubstitution(
                [nav2_demo_dir, "config/nav2_config.yaml"]
            ),
            {"use_sim_time": True}
        ],
    )
    
    # RViz2 visualization (optional, controlled by launch parameter)
    rviz = Node(
        package="rviz2",
        executable="rviz2",
        arguments=[
            "-d", PathJoinSubstitution([nav2_demo_dir, "config/config.rviz"])
        ],
        parameters=[{"use_sim_time": True}],
        condition=IfCondition(LaunchConfiguration("rviz", default="true")),
        output="screen"
    )
    
    return LaunchDescription([
        DeclareLaunchArgument(
            "rviz",
            default_value="true",
            description="Start RViz2 visualization"
        ),
        
        kiss_icp_launch,
        ground_seg_launch,
        controller_server,
        rviz,
    ])
