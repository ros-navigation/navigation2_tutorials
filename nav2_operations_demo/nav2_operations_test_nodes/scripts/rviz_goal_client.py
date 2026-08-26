#!/usr/bin/env python3
# Copyright (c) 2024 Apache-2.0

"""
RViz client that:
  1. Subscribes to /goal_pose (RViz "2D Nav Goal").
  2. Uses Nav2 Simple Commander to compute a path.
  3. Sends a NavigateWithOperations action goal to the custom navigator.
"""

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node

import tf2_ros
from geometry_msgs.msg import PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator

from nav2_operations_msgs.action import NavigateWithOperations

DEFAULT_BT_XML_FILE = '/ros2_ws/src/nav2_operations_bringup/config/bt/navigate_with_operations.xml'
ACTION_SERVER_NAME = 'navigate_with_operations'


class RvizGoalClient(Node):
    def __init__(self):
        super().__init__('rviz_goal_client')

        self.declare_parameter('bt_xml_file', DEFAULT_BT_XML_FILE)
        self._bt_xml_file = self.get_parameter('bt_xml_file').get_parameter_value().string_value

        self._navigator = BasicNavigator(node_name='rviz_goal_client_navigator')

        self._tf_buffer = tf2_ros.Buffer()
        self._tf_listener = tf2_ros.TransformListener(self._tf_buffer, self)

        self._action_client = ActionClient(self, NavigateWithOperations, ACTION_SERVER_NAME)

        self._goal_sub = self.create_subscription(
            PoseStamped, '/goal_pose', self._goal_pose_callback, 10)

        self.get_logger().info(
            f'RvizGoalClient ready. BT file: {self._bt_xml_file}. '
            'Set a "2D Nav Goal" in RViz to trigger navigation.')

    def _goal_pose_callback(self, msg: PoseStamped):
        self.get_logger().info(
            f'Received goal pose: ({msg.pose.position.x:.2f}, {msg.pose.position.y:.2f})')

        start = self._get_current_pose()
        path = self._navigator.getPath(start=start, goal=msg, planner_id='GridBased', use_start=True)

        if path is None or len(path.poses) == 0:
            self.get_logger().error('Failed to compute a path to the goal.')
            return

        self.get_logger().info(f'Path computed with {len(path.poses)} poses.')
        self._send_goal(path)

    def _get_current_pose(self) -> PoseStamped:
        try:
            transform = self._tf_buffer.lookup_transform('map', 'base_link', rclpy.time.Time())
            pose = PoseStamped()
            pose.header.frame_id = 'map'
            pose.header.stamp = transform.header.stamp
            pose.pose.position.x = transform.transform.translation.x
            pose.pose.position.y = transform.transform.translation.y
            pose.pose.position.z = transform.transform.translation.z
            pose.pose.orientation = transform.transform.rotation
            return pose
        except (tf2_ros.LookupException, tf2_ros.ExtrapolationException):
            self.get_logger().warn('TF lookup failed, using origin as start.')

        pose = PoseStamped()
        pose.header.frame_id = 'map'
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.orientation.w = 1.0
        return pose

    def _send_goal(self, path):
        if not self._action_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error(f'Action server "{ACTION_SERVER_NAME}" not available.')
            return

        goal_msg = NavigateWithOperations.Goal()
        goal_msg.behavior_tree = self._bt_xml_file
        goal_msg.path = path

        self.get_logger().info('Sending NavigateWithOperations goal...')
        send_goal_future = self._action_client.send_goal_async(
            goal_msg, feedback_callback=self._feedback_callback)
        send_goal_future.add_done_callback(self._goal_response_callback)

    def _goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error('Goal was rejected by the action server.')
            return
        self.get_logger().info('Goal accepted. Waiting for result...')
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self._result_callback)

    def _feedback_callback(self, feedback_msg):
        fb = feedback_msg.feedback
        secs = fb.navigation_time.sec + fb.navigation_time.nanosec * 1e-9
        self.get_logger().info(
            f'Navigation time: {secs:.1f}s | Path poses: {fb.path_poses_count}')

    def _result_callback(self, future):
        result = future.result().result
        codes = {0: 'succeeded', 1: 'failed', 2: 'cancelled'}
        self.get_logger().info(
            f'NavigateWithOperations {codes.get(result.error_code, "unknown")}')


def main(args=None):
    rclpy.init(args=args)
    node = RvizGoalClient()

    executor = rclpy.executors.MultiThreadedExecutor()
    executor.add_node(node)
    executor.add_node(node._navigator)

    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown()
        node._navigator.destroy_node()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
