import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():
    # Declare the 'use_gui' argument
    use_gui_arg = DeclareLaunchArgument(
        "use_gui",
        default_value="true",
        choices=["true", "false"],
        description="Whether to execute gzclient (GUI for Gazebo)"
    )

    # Get package paths
    pkg_gazebo_ros = get_package_share_directory("gazebo_ros")
    pkg_environments = get_package_share_directory("nav2_gps_waypoint_follower_demo")

    # Define the world file path
    world_file_path = os.path.join(pkg_environments, "worlds", "sonoma_raceway.world")

    # Define the models directory path
    models_directory = os.path.join(pkg_environments, "models")

    # Verify if the models directory exists
    if not os.path.exists(models_directory):
        raise FileNotFoundError(f"Models directory does not exist: {models_directory}")

    # Set GAZEBO_MODEL_PATH to include the 'models' directory in the ROS 2 package
    gazebo_model_path = SetEnvironmentVariable(
        name="GAZEBO_MODEL_PATH",
        value=models_directory
    )
    
    # Read URDF file
    urdf = os.path.join(pkg_environments, 'urdf', 'turtlebot3_waffle_gps.urdf')
    with open(urdf, 'r') as infp:
        robot_description = infp.read()

    # Create robot state publisher node
    start_robot_state_publisher_cmd = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='both',
        parameters=[{'robot_description': robot_description}])

    # Include the Gazebo server (gzserver) launch file
    gzserver_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo_ros, "launch", "gzserver.launch.py")
        ),
        launch_arguments={
            "world": world_file_path,
            "verbose": "true",
        }.items()
    )

    # Include the Gazebo client (gzclient) launch file if 'use_gui' is true
    gzclient_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo_ros, "launch", "gzclient.launch.py")
        )
    )

    # Return the LaunchDescription
    return LaunchDescription([use_gui_arg, gazebo_model_path, start_robot_state_publisher_cmd, gzserver_launch, gzclient_launch])
