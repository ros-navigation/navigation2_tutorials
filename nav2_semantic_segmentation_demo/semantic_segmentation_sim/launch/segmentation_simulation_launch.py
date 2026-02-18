"""Launch file for semantic segmentation simulation with Nav2."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression


def generate_launch_description():
    # Get package directory
    pkg_dir = get_package_share_directory('semantic_segmentation_sim')
    launch_dir = os.path.join(pkg_dir, 'launch')

    # Launch configuration variables
    use_sim_gui = LaunchConfiguration('use_sim_gui')
    use_rviz = LaunchConfiguration('use_rviz')

    # Declare launch arguments
    declare_use_sim_gui_cmd = DeclareLaunchArgument(
        'use_sim_gui',
        default_value='true',
        description='Launch Gazebo with GUI',
    )

    declare_use_rviz_cmd = DeclareLaunchArgument(
        'use_rviz',
        default_value='true',
        description='Launch RViz visualization',
    )

    # Convert use_sim_gui to headless (inverted logic)
    headless = PythonExpression(['"False" if "', use_sim_gui, '" == "true" else "True"'])

    # Include simulation launch
    simulation_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(launch_dir, 'simulation_launch.py')
        ),
        launch_arguments={
            'headless': headless,
        }.items(),
    )

    # Include Nav2 + segmentation launch
    nav2_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(launch_dir, 'nav2_segmentation_launch.py')
        ),
        launch_arguments={
            'rviz': use_rviz,
            'use_sim_time': 'true',
        }.items(),
    )

    # Create launch description
    ld = LaunchDescription()

    # Declare arguments
    ld.add_action(declare_use_sim_gui_cmd)
    ld.add_action(declare_use_rviz_cmd)

    # Add launch files
    ld.add_action(simulation_launch)
    ld.add_action(nav2_launch)

    return ld

