import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.descriptions import ParameterFile
from nav2_common.launch import RewrittenYaml


def generate_launch_description():
    bringup_dir = get_package_share_directory('nav2_operations_bringup')

    params_file = os.path.join(bringup_dir, 'config', 'nav2_params_predefined.yaml')

    use_sim_time = LaunchConfiguration('use_sim_time', default='false')

    param_substitutions = {
        'use_sim_time': use_sim_time,
    }
    configured_params = ParameterFile(
        RewrittenYaml(
            source_file=params_file,
            param_rewrites=param_substitutions,
            convert_types=True),
        allow_substs=True)

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation time'),

        # Nav2 Planner Server
        Node(
            package='nav2_planner',
            executable='planner_server',
            name='planner_server',
            output='screen',
            parameters=[configured_params],
        ),

        # Nav2 Controller Server
        Node(
            package='nav2_controller',
            executable='controller_server',
            name='controller_server',
            output='screen',
            parameters=[configured_params],
        ),

        # Blade Server (lifecycle node)
        Node(
            package='nav2_operations_servers',
            executable='blade_server_node',
            name='blade_server',
            output='screen',
            parameters=[configured_params],
        ),

        # Camera Server (lifecycle node)
        Node(
            package='nav2_operations_servers',
            executable='camera_server_node',
            name='camera_server',
            output='screen',
            parameters=[configured_params],
        ),

        Node(
            package='nav2_behaviors',
            executable='behavior_server',
            name='behavior_server',
            output='screen',
            respawn_delay=2.0,
            parameters=[configured_params],
        ),
        Node(
            package='nav2_velocity_smoother',
            executable='velocity_smoother',
            name='velocity_smoother',
            output='screen',
            respawn_delay=2.0,
            parameters=[configured_params],
        ),

        # Nav2 BT Navigator
        Node(
            package='nav2_bt_navigator',
            executable='bt_navigator',
            name='bt_navigator',
            output='screen',
            parameters=[configured_params],
        ),

        # Lifecycle Manager for Nav2 nodes
        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_navigation',
            output='screen',
            parameters=[{
                'use_sim_time': use_sim_time,
                'autostart': True,
                'node_names': [
                    'planner_server',
                    'controller_server',
                    'blade_server',
                    'camera_server',
                    'behavior_server',
                    'velocity_smoother',
                    'bt_navigator',
                ],
            }],
        ),

        # Fake Robot
        Node(
            package='nav2_operations_test_nodes',
            executable='fake_robot_node',
            name='fake_robot',
            output='screen',
            parameters=[{
                'use_sim_time': use_sim_time,
                'initial_x': 0.0,
                'initial_y': 0.0,
                'initial_yaw': 0.0,
                'publish_rate': 50.0,
            }],
        ),

        # Blade Simulator
        Node(
            package='nav2_operations_test_nodes',
            executable='blade_simulator_node',
            name='blade_simulator',
            output='screen',
        ),

        # Camera Simulator
        Node(
            package='nav2_operations_test_nodes',
            executable='camera_simulator_node',
            name='camera_simulator',
            output='screen',
            parameters=[{
                'rotation_speed': 45.0,
                'update_rate': 20.0,
            }],
        ),

        # RViz Goal Client — subscribes to /goal_pose and calls NavigateWithOperations
        Node(
            package='nav2_operations_test_nodes',
            executable='rviz_goal_client.py',
            name='rviz_goal_client',
            output='screen',
            parameters=[configured_params],
        ),
    ])
