# Copyright (c) 2018 Intel Corporation
# Copyright (C) 2024 Stevedan Ogochukwu Omodolor Omodia
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
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    ExecuteProcess,
)

from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import Node


def generate_launch_description():
    sim_dir = get_package_share_directory("nav2_minimal_tb3_sim")
    tutorial_dir = get_package_share_directory("nav2_gps_waypoint_follower_demo")

    # Create the launch configuration variables
    use_sim_time = LaunchConfiguration("use_sim_time")

    # Decalre the launch arguments
    declare_use_sim_time_cmd = DeclareLaunchArgument(
        "use_sim_time",
        default_value="True",
        description="Use simulation (Gazebo) clock if true",
    )

    world_sdf = os.path.join(tutorial_dir, "worlds", "tb3_sonoma_raceway.sdf.xacro")
    robot_sdf = os.path.join(sim_dir, "urdf", "gz_waffle_gps.sdf.xacro")

    urdf = os.path.join(sim_dir, "urdf", "turtlebot3_waffle_gps.urdf")
    with open(urdf, "r") as infp:
        robot_description = infp.read()

    gazebo_server = ExecuteProcess(
        cmd=["gz", "sim", "-r", "-s", world_sdf],
        output="screen",
    )

    gazebo_client = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("ros_gz_sim"), "launch", "gz_sim.launch.py"
            )
        ),
        launch_arguments={"gz_args": ["-v4 -g "]}.items(),
    )

    gz_robot = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(sim_dir, "launch", "spawn_tb3_gps.launch.py")
        ),
        launch_arguments={
            "use_sim_time": use_sim_time,
            "robot_sdf": robot_sdf,
            "x_pose": "2.0",
            "y_pose": "-2.5",
            "z_pose": "0.33",
            "roll": "0.0",
            "pitch": "0.0",
            "yaw": "0.0",
        }.items(),
    )

    start_robot_state_publisher_cmd = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="screen",
        parameters=[
            {"use_sim_time": use_sim_time, "robot_description": robot_description}
        ],
    )

    ld = LaunchDescription()

    # Declare the launch options
    ld.add_action(declare_use_sim_time_cmd)

    ld.add_action(gz_robot)
    ld.add_action(gazebo_server)
    ld.add_action(gazebo_client)

    ld.add_action(start_robot_state_publisher_cmd)
    return ld
