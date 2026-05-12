"""
Nav2 navigation stack launch for nav2_lidar_ground_segmentation_demo

Usage:
  ros2 launch nav2_lidar_ground_segmentation_demo full_stack.launch.py
"""

import os
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, SetEnvironmentVariable
from launch.substitutions import PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

DEMO_PKG_SHARE = get_package_share_directory('nav2_lidar_ground_segmentation_demo')
WORLD_NAME = "baylands_terrain"

def generate_gazebo_launch():
    """Generate Gazebo simulation with Husky robot and terrain."""
        
    # Gazebo world and GUI config paths'
    world_sdf_path = os.path.join(DEMO_PKG_SHARE, 'models', WORLD_NAME + '.sdf')
    gui_config_path = os.path.join(DEMO_PKG_SHARE, 'config', 'gazebo_gui.config')
    
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
        f'/world/{WORLD_NAME}/model/husky/link/base_link/sensor/front_laser/scan/points@sensor_msgs/msg/PointCloud2@gz.msgs.PointCloudPacked',
        f'/world/{WORLD_NAME}/clock@rosgraph_msgs/msg/Clock@gz.msgs.Clock'
    ]
    
    bridge_config_path = os.path.join(DEMO_PKG_SHARE, 'config', 'bridge_config.yaml')

    ign_ros2_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        parameters=[{'config_file': bridge_config_path}],
        output='both'
    )
    
    # Static transform: base_link -> front_laser
    # We don't use tf from Gazebo because it interferes with kiss_icp's odometry frame associations. 
    # Instead, we publish a static transform with the same parameters as the one in Gazebo.
    static_tf_front_laser = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=[
            '--x', '0.0012',
            '--y', '0',
            '--z', '0.716',
            '--yaw', '0',
            '--pitch', '0',
            '--roll', '0',
            '--frame-id', 'husky/base_link',
            '--child-frame-id', 'husky/base_link/front_laser'
        ],
        parameters=[{'use_sim_time': True}]
    )
    
    return [gazebo_launch, ign_ros2_bridge, static_tf_front_laser]

def generate_launch_description():
    """Generate complete launch description."""
    
    # Get package directories
    nav2_bringup_dir = FindPackageShare("nav2_bringup")
    kiss_icp_dir = FindPackageShare("kiss_icp")
    ground_seg_dir = FindPackageShare("ground_segmentation_ros2")
    
    # Set Gazebo resource path
    models_dir = os.path.join(DEMO_PKG_SHARE, 'models')
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
                [DEMO_PKG_SHARE, "config/kiss_icp_config.yaml"]
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
                [DEMO_PKG_SHARE, "config/gseg3d_config.yaml"]
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
            "params_file": PathJoinSubstitution([DEMO_PKG_SHARE, "config", "nav2_config.yaml"]),
        }.items(),
    )
    
    # Static map -> odom transform
    # We don't use a mapper in the demo so provide a static transform between map and odom.
    map_to_odom_tf = Node(
            package="tf2_ros",
            output="screen",
            executable="static_transform_publisher",
            arguments=[
                '--x', '0',
                '--y', '0',
                '--z', '0',
                '--qx', '0',
                '--qy', '0',
                '--qz', '0',
                '--qw', '1',
                '--frame-id', 'map',
                '--child-frame-id', 'odom'
            ],
            parameters=[{'use_sim_time': True}],
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
        
        *generate_gazebo_launch(),
        kiss_icp_launch,
        ground_seg_launch,
        map_to_odom_tf,
        nav2_bringup,
        rviz_launch,
    ])
