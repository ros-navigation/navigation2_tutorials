# Copyright (c) 2018 Intel Corporation
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import ExecuteProcess, IncludeLaunchDescription, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from nav2_common.launch import RewrittenYaml


def generate_launch_description():
    # Get the launch directory
    bringup_dir = get_package_share_directory('nav2_bringup')  
    gps_wpf_dir =  get_package_share_directory("nav2_gps_waypoint_follower_demo")
    launch_dir = os.path.join(gps_wpf_dir, 'launch')
    world = os.path.join(gps_wpf_dir, "worlds", "sonoma_raceway.world")
    params_dir = os.path.join(gps_wpf_dir, "config")
    nav2_params = os.path.join(params_dir, "nav2_no_map_params.yaml")
    configured_params = RewrittenYaml(
        source_file=nav2_params, root_key="", param_rewrites="", convert_types=True
    )

    models_dir = os.path.join(gps_wpf_dir, "models")
    models_dir += os.pathsep + f"/opt/ros/{os.getenv('ROS_DISTRO')}/share/turtlebot3_gazebo/models"
    set_gazebo_model_path_cmd = None

    if 'GAZEBO_MODEL_PATH' in os.environ:
        gazebo_model_path = os.environ['GAZEBO_MODEL_PATH'] +  os.pathsep + models_dir
        set_gazebo_model_path_cmd = SetEnvironmentVariable("GAZEBO_MODEL_PATH", gazebo_model_path)
    else:
        set_gazebo_model_path_cmd = SetEnvironmentVariable("GAZEBO_MODEL_PATH", models_dir)
    
    
    # Specify the actions
    start_gazebo_server_cmd = ExecuteProcess(
        cmd=['gzserver', '-s', 'libgazebo_ros_init.so',
             '-s', 'libgazebo_ros_factory.so', world],
        cwd=[launch_dir], output='both')

    start_gazebo_client_cmd = ExecuteProcess(
        cmd=['gzclient'],
        cwd=[launch_dir], output='both')

    robot_localization_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
        os.path.join(launch_dir, 'dual_ekf_navsat.launch.py'))
        )

    static_transforms_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
        os.path.join(launch_dir, 'static_transforms.launch.py'))
        )

    bringup_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(bringup_dir, "launch", "navigation_launch.py")
        ),
        launch_arguments={
            "use_sim_time": "True",
            "params_file": configured_params,
            "autostart": "True",
        }.items(),
    )

    # Create the launch description and populate
    ld = LaunchDescription()

    # Set gazebo up to find models properly
    ld.add_action(set_gazebo_model_path_cmd)

    # simulator launch
    ld.add_action(start_gazebo_server_cmd)
    ld.add_action(start_gazebo_client_cmd)

    # robot localization launch
    ld.add_action(robot_localization_cmd)

    # static transforms launch
    ld.add_action(static_transforms_cmd)


    # Add the actions to launch all of the navigation nodes
    ld.add_action(bringup_cmd)

    return ld
