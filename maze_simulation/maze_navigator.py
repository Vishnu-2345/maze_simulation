#!/usr/bin/env python3
"""
Autonomous Maze Exit Navigator

Sends autonomous navigation goals to Nav2 (navigate_to_pose action)
to pilot the robot from the maze starting point to the exit coordinates.
"""

import math
import sys
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose

class MazeNavigator(Node):
    """
    Goal dispatcher and progress monitor for autonomous maze navigation.
    """

    def __init__(self):
        super().__init__('maze_navigator')

        # Parameters (Default maze exit marker pose is x=4.5, y=4.5)
        self.declare_parameter('goal_x', 4.5)
        self.declare_parameter('goal_y', 4.5)
        self.declare_parameter('goal_yaw', 0.0)

        self.goal_x = self.get_parameter('goal_x').value
        self.goal_y = self.get_parameter('goal_y').value
        self.goal_yaw = self.get_parameter('goal_yaw').value

        self._action_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')

        self.get_logger().info(
            f"Autonomous Maze Navigator initialized. Target Exit: ({self.goal_x}, {self.goal_y})"
        )

    def send_exit_goal(self):
        self.get_logger().info('Waiting for Nav2 navigate_to_pose action server...')
        if not self._action_client.wait_for_server(timeout_sec=30.0):
            self.get_logger().error('Nav2 action server not available after 30 seconds!')
            return False

        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = PoseStamped()
        goal_msg.pose.header.frame_id = 'map'
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()

        goal_msg.pose.pose.position.x = float(self.goal_x)
        goal_msg.pose.pose.position.y = float(self.goal_y)
        goal_msg.pose.pose.position.z = 0.0

        # Convert yaw to quaternion
        half_yaw = self.goal_yaw * 0.5
        goal_msg.pose.pose.orientation.x = 0.0
        goal_msg.pose.pose.orientation.y = 0.0
        goal_msg.pose.pose.orientation.z = math.sin(half_yaw)
        goal_msg.pose.pose.orientation.w = math.cos(half_yaw)

        self.get_logger().info(
            f"Dispatching Goal -> Map Coordinates: X={self.goal_x:.2f}, Y={self.goal_y:.2f}"
        )

        send_goal_future = self._action_client.send_goal_async(
            goal_msg,
            feedback_callback=self.feedback_callback
        )
        send_goal_future.add_done_callback(self.goal_response_callback)
        return True

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error('Nav2 rejected navigation goal!')
            return

        self.get_logger().info('Goal accepted by Nav2. Route execution underway...')
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.result_callback)

    def feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback
        dist_remaining = feedback.distance_remaining
        time_elapsed = feedback.navigation_time.sec
        recoveries = feedback.number_of_recoveries

        self.get_logger().info(
            f"Nav Status: Distance Remaining: {dist_remaining:.2f}m | "
            f"Time: {time_elapsed}s | Recoveries: {recoveries}",
            throttle_duration_sec=2.0
        )

    def result_callback(self, future):
        status = future.result().status
        if status == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info('🏆 SUCCESS! Robot has autonomously navigated to the maze exit!')
        elif status == GoalStatus.STATUS_CANCELED:
            self.get_logger().warn('Navigation goal was canceled.')
        else:
            self.get_logger().error(f'Navigation failed with status code: {status}')

        rclpy.shutdown()


def main(args=None):
    rclpy.init(args=args)
    navigator = MazeNavigator()

    if navigator.send_exit_goal():
        rclpy.spin(navigator)

    navigator.destroy_node()


if __name__ == '__main__':
    main()

