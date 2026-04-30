"""
Nav2 navigation stack launch for nav2_ground_consistency_demo.

Usage:
  ros2 launch nav2_ground_consistency_demo full_stack.launch.py
"""

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, TimerAction
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
    
    # Delay nav2 stack start to ensure localization and ground segmentation are ready
    nav2_launch_delayed = TimerAction(
        period=3.0,
        actions=[
            # Local Costmap Server (with ground consistency layer)
            Node(
                package="nav2_costmap_2d",
                executable="costmap_2d_markers_server",
                name="local_costmap_markers",
                output="screen",
                parameters=[
                    PathJoinSubstitution([nav2_demo_dir, "config/nav2_config.yaml"]),
                    {"use_sim_time": True}
                ]
            ),
            Node(
                package="nav2_costmap_2d",
                executable="costmap_2d_node",
                name="local_costmap_server",
                output="screen",
                remappings=[
                    ("costmap/costmap_raw", "local_costmap/costmap_raw"),
                    ("costmap/footprint", "local_costmap/footprint"),
                    ("costmap/published_footprint", "local_costmap/published_footprint"),
                ],
                parameters=[
                    PathJoinSubstitution([nav2_demo_dir, "config/nav2_config.yaml"]),
                    {"use_sim_time": True}
                ]
            ),
            # Planner Server
            Node(
                package="nav2_planner",
                executable="planner_server",
                name="planner_server",
                output="screen",
                parameters=[
                    PathJoinSubstitution([nav2_demo_dir, "config/nav2_config.yaml"]),
                    {"use_sim_time": True}
                ]
            ),
            # Controller Server
            Node(
                package="nav2_controller",
                executable="controller_server",
                name="controller_server",
                output="screen",
                parameters=[
                    PathJoinSubstitution([nav2_demo_dir, "config/nav2_config.yaml"]),
                    {"use_sim_time": True}
                ]
            ),
            # Behavior Server
            Node(
                package="nav2_behaviors",
                executable="behavior_server",
                name="behavior_server",
                output="screen",
                parameters=[
                    PathJoinSubstitution([nav2_demo_dir, "config/nav2_config.yaml"]),
                    {"use_sim_time": True}
                ]
            ),
            # Navigation to Pose Server
            Node(
                package="nav2_bt_navigator",
                executable="bt_navigator",
                name="bt_navigator",
                output="screen",
                parameters=[
                    PathJoinSubstitution([nav2_demo_dir, "config/nav2_config.yaml"]),
                    {"use_sim_time": True}
                ]
            ),
        ]
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
        
        gazebo_launch,
        kiss_icp_launch,
        ground_seg_launch,
        nav2_launch_delayed,
        rviz,
    ])
