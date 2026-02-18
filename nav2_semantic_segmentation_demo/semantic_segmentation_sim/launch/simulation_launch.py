"""Launch Gazebo simulation with TurtleBot 4 in Baylands world."""

import os
import tempfile

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    AppendEnvironmentVariable,
    DeclareLaunchArgument,
    ExecuteProcess,
    IncludeLaunchDescription,
    OpaqueFunction,
    RegisterEventHandler,
)
from launch.conditions import IfCondition
from launch.event_handlers import OnShutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration, PythonExpression
from launch_ros.actions import Node


def generate_launch_description():
    # Get package directories
    sim_dir = get_package_share_directory('nav2_minimal_tb4_sim')
    desc_dir = get_package_share_directory('nav2_minimal_tb4_description')
    pkg_dir = get_package_share_directory('semantic_segmentation_sim')

    # Launch configuration variables
    headless = LaunchConfiguration('headless')
    use_sim_time = LaunchConfiguration('use_sim_time')

    # Robot pose in world
    pose = {
        'x': LaunchConfiguration('x_pose', default='-13.34'),
        'y': LaunchConfiguration('y_pose', default='-6.35'),
        'z': LaunchConfiguration('z_pose', default='0.3'),
        'R': LaunchConfiguration('roll', default='0.00'),
        'P': LaunchConfiguration('pitch', default='0.00'),
        'Y': LaunchConfiguration('yaw', default='2.78'),
    }
    robot_name = LaunchConfiguration('robot_name')
    robot_sdf = LaunchConfiguration('robot_sdf')

    # World file
    baylands_world = os.path.join(pkg_dir, 'worlds', 'baylands.sdf')

    # Declare launch arguments
    declare_headless_cmd = DeclareLaunchArgument(
        'headless',
        default_value='True',
        description='Whether to launch Gazebo headless (no GUI)',
    )

    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation clock',
    )

    declare_robot_name_cmd = DeclareLaunchArgument(
        'robot_name',
        default_value='nav2_turtlebot4',
        description='Robot name',
    )

    declare_robot_sdf_cmd = DeclareLaunchArgument(
        'robot_sdf',
        default_value=os.path.join(desc_dir, 'urdf', 'standard', 'turtlebot4.urdf.xacro'),
        description='Path to robot URDF/xacro file',
    )

    # Robot state publisher
    start_robot_state_publisher_cmd = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[
            {
                'use_sim_time': use_sim_time,
                'robot_description': Command(['xacro', ' ', robot_sdf]),
            }
        ],
    )

    # Process world file through xacro to inject headless parameter
    world_sdf = tempfile.mktemp(prefix='nav2_', suffix='.sdf')
    world_sdf_xacro = ExecuteProcess(
        cmd=['xacro', '-o', world_sdf, ['headless:=', headless], baylands_world],
    )

    # Gazebo server
    gazebo_server = ExecuteProcess(
        cmd=['gz', 'sim', '-r', '-s', '-v', '4', world_sdf],
        output='screen',
    )

    # Clean up temp file on shutdown
    remove_temp_sdf_file = RegisterEventHandler(
        event_handler=OnShutdown(
            on_shutdown=[OpaqueFunction(function=lambda _: os.remove(world_sdf))]
        )
    )

    # Set Gazebo resource path
    set_env_vars_resources = AppendEnvironmentVariable(
        'GZ_SIM_RESOURCE_PATH', os.path.join(sim_dir, 'worlds')
    )

    # Gazebo client (GUI)
    gazebo_client = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py'
            )
        ),
        condition=IfCondition(PythonExpression(['not ', headless])),
        launch_arguments={'gz_args': ['-v4 -g ']}.items(),
    )

    # Spawn robot
    gz_robot = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(sim_dir, 'launch', 'spawn_tb4.launch.py')
        ),
        launch_arguments={
            'namespace': '',
            'use_simulator': 'True',
            'use_sim_time': use_sim_time,
            'robot_name': robot_name,
            'robot_sdf': robot_sdf,
            'x_pose': pose['x'],
            'y_pose': pose['y'],
            'z_pose': pose['z'],
            'roll': pose['R'],
            'pitch': pose['P'],
            'yaw': pose['Y'],
        }.items(),
    )

    # Static transform to map namespaced frame to expected frame
    # Gazebo publishes with frame_id: turtlebot4/oakd_rgb_camera_frame/rgbd_camera
    # But TF tree has: oakd_rgb_camera_frame
    # This transform maps the namespaced frame to the expected frame
    static_tf_camera_frame = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_transform_publisher_camera_frame',
        arguments=[
            '--x', '0', '--y', '0', '--z', '0',
            '--roll', '0', '--pitch', '0', '--yaw', '0',
            '--frame-id', 'oakd_rgb_camera_optical_frame',
            '--child-frame-id', 'nav2_turtlebot4/oakd_rgb_camera_frame/rgbd_camera'
        ],
        output='screen',
    )

    # Depth image to pointcloud converter
    depth_to_pointcloud_node = Node(
        package='depth_image_proc',
        executable='point_cloud_xyz_node',
        name='depth_to_pointcloud',
        namespace='rgbd_camera',
        remappings=[
            ('image_rect', 'depth_image'),
            ('camera_info', 'camera_info'),
            ('points', 'depth/points'),
        ],
        parameters=[
            {
                'use_sim_time': use_sim_time,
                'qos_overrides./rgbd_camera/depth/points.reliability': 'best_effort',
                'qos_overrides./rgbd_camera/depth/points.durability': 'volatile',
            }
        ],
        output='screen',
    )

    # Create launch description
    ld = LaunchDescription()

    # Declare arguments
    ld.add_action(declare_headless_cmd)
    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(declare_robot_name_cmd)
    ld.add_action(declare_robot_sdf_cmd)

    # Add actions
    ld.add_action(set_env_vars_resources)
    ld.add_action(world_sdf_xacro)
    ld.add_action(remove_temp_sdf_file)
    ld.add_action(gazebo_server)
    ld.add_action(gazebo_client)
    ld.add_action(gz_robot)
    ld.add_action(start_robot_state_publisher_cmd)
    ld.add_action(static_tf_camera_frame)
    ld.add_action(depth_to_pointcloud_node)

    return ld
