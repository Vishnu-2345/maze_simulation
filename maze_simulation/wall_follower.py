#!/usr/bin/env python3
"""
Maze Solver: Algorithmic Right-Hand Wall Follower Node

Listens to 2D LiDAR scans on /scan and controls robot kinematics on /cmd_vel
to autonomously navigate and solve the maze using robust wall-following.
"""

import math
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist

class WallFollower(Node):
    """
    Right-hand wall follower robot controller for 2D maze simulation.
    """

    def __init__(self):
        super().__init__('wall_follower')

        # Parameters
        self.declare_parameter('forward_speed', 0.25)
        self.declare_parameter('turn_speed', 0.7)
        self.declare_parameter('target_wall_dist', 0.45)
        self.declare_parameter('front_threshold', 0.55)
        self.declare_parameter('kp_gain', 1.8)

        self.forward_speed = self.get_parameter('forward_speed').value
        self.turn_speed = self.get_parameter('turn_speed').value
        self.target_dist = self.get_parameter('target_wall_dist').value
        self.front_threshold = self.get_parameter('front_threshold').value
        self.kp = self.get_parameter('kp_gain').value

        # Sensor QoS configuration (Best Effort + Reliable compatibility)
        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )

        # Subscribers & Publishers
        self.scan_sub = self.create_subscription(
            LaserScan,
            '/scan',
            self.scan_callback,
            sensor_qos
        )
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        # Internal state tracking
        self.state = 'FIND_WALL'
        self.last_log_time = self.get_clock().now()

        self.get_logger().info('Wall Follower Node Initialized. Tracking right-hand wall...')

    def get_sector_min(self, msg: LaserScan, angle_start_deg: float, angle_end_deg: float) -> float:
        """
        Calculates minimum valid distance within an angular sector in degrees.
        0 degrees = directly forward.
        +90 degrees = left.
        -90 degrees = right.
        """
        rad_start = math.radians(angle_start_deg)
        rad_end = math.radians(angle_end_deg)

        # Clamp angles within sensor bounds
        rad_start = max(rad_start, msg.angle_min)
        rad_end = min(rad_end, msg.angle_max)

        idx_start = int((rad_start - msg.angle_min) / msg.angle_increment)
        idx_end = int((rad_end - msg.angle_min) / msg.angle_increment)

        if idx_start > idx_end:
            idx_start, idx_end = idx_end, idx_start

        idx_start = max(0, min(len(msg.ranges) - 1, idx_start))
        idx_end = max(0, min(len(msg.ranges) - 1, idx_end))

        valid_ranges = [
            r for r in msg.ranges[idx_start:idx_end + 1]
            if msg.range_min <= r <= msg.range_max and not math.isinf(r) and not math.isnan(r)
        ]

        return min(valid_ranges) if valid_ranges else msg.range_max

    def scan_callback(self, msg: LaserScan):
        cmd = Twist()

        # Extract range distances for key sectors
        # Front sector: -25 deg to +25 deg
        dist_front = self.get_sector_min(msg, -25.0, 25.0)
        # Front-Right sector: -65 deg to -30 deg
        dist_front_right = self.get_sector_min(msg, -65.0, -30.0)
        # Direct Right sector: -105 deg to -75 deg
        dist_right = self.get_sector_min(msg, -105.0, -75.0)
        # Direct Left sector: +75 deg to +105 deg
        dist_left = self.get_sector_min(msg, 75.0, 105.0)

        # Reactive State Machine Logic
        # 1. Emergency Obstacle / Dead End Recovery
        if dist_front < 0.28:
            self.state = 'EMERGENCY_REVERSE'
            cmd.linear.x = -0.12
            cmd.angular.z = self.turn_speed if dist_right < dist_left else -self.turn_speed

        # 2. Wall directly in front -> Sharp Left Turn
        elif dist_front < self.front_threshold or dist_front_right < 0.35:
            self.state = 'TURN_LEFT'
            cmd.linear.x = 0.04
            cmd.angular.z = self.turn_speed

        # 3. Outer Corner Detection (Right wall disappeared) -> Arc Right to wrap around corner
        elif dist_right > 0.9 and dist_front_right > 0.85:
            self.state = 'ROUND_CORNER'
            cmd.linear.x = self.forward_speed * 0.75
            cmd.angular.z = -self.turn_speed * 0.65

        # 4. Normal Right-Wall Following with Proportional (P) Steering
        else:
            self.state = 'FOLLOW_WALL'
            # Error > 0 if robot is too far from right wall (need to steer right, negative z)
            # Error < 0 if robot is too close to right wall (need to steer left, positive z)
            error = dist_right - self.target_dist
            angular_correction = -self.kp * error

            # Limit angular rate for smooth motion
            angular_correction = max(-self.turn_speed, min(self.turn_speed, angular_correction))

            cmd.linear.x = self.forward_speed
            cmd.angular.z = angular_correction

        self.cmd_pub.publish(cmd)

        # Periodic Diagnostic Log (every 2 seconds)
        now = self.get_clock().now()
        if (now - self.last_log_time).nanoseconds > 2.0e9:
            self.get_logger().info(
                f"State: {self.state:<17} | Front: {dist_front:.2f}m | Right: {dist_right:.2f}m | "
                f"Cmd: [lin={cmd.linear.x:.2f}, ang={cmd.angular.z:.2f}]"
            )
            self.last_log_time = now


def main(args=None):
    rclpy.init(args=args)
    node = WallFollower()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # Publish zero velocity on shutdown
        stop_cmd = Twist()
        node.cmd_pub.publish(stop_cmd)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
