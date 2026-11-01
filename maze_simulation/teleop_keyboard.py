#!/usr/bin/env python3
"""
Interactive Keyboard Teleoperation Node for Maze Robot

Enables manual driving of the robot through the maze using WASD and Arrow Keys.
Publishes velocity commands to /cmd_vel.
"""

import sys
import select
import termios
import tty
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

BANNER = """
===========================================================
  🎮 MAZE ROBOT KEYBOARD TELEOPERATION CONTROLLER
===========================================================
  Controls:
  ---------------------------------------------------------
          [W] / [↑]  : Move Forward
  [A] / [←]  [S] / [↓]  [D] / [→] : Turn Left / Reverse / Turn Right

  Diagonal / Arc Moves:
    [Q] : Forward + Turn Left      [E] : Forward + Turn Right
    [Z] : Reverse + Turn Left      [C] : Reverse + Turn Right

  Braking & Speed:
    [SPACE] or [X] : EMERGENCY STOP
    [+] / [=]      : Increase Speed (+10%)
    [-] / [_]      : Decrease Speed (-10%)

  Press [Ctrl+C] to Exit safely
===========================================================
"""

class TeleopKeyboard(Node):

    def __init__(self):
        super().__init__('teleop_keyboard')
        self.pub = self.create_publisher(Twist, '/cmd_vel', 10)

        # Speed limits and current targets
        self.linear_speed = 0.35      # m/s
        self.angular_speed = 0.85     # rad/s

        self.target_linear = 0.0
        self.target_angular = 0.0

        self.get_logger().info('Keyboard Teleoperation ready. Use terminal to drive!')

    def get_key(self, settings):
        """Reads a keypress or escape sequence non-blockingly."""
        tty.setraw(sys.stdin.fileno())
        rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
        if rlist:
            key = sys.stdin.read(1)
            # Handle multi-byte ANSI escape codes for Arrow Keys
            if key == '\x1b':
                additional = sys.stdin.read(2)
                key += additional
        else:
            key = ''
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
        return key

    def run_loop(self):
        settings = termios.tcgetattr(sys.stdin)
        print(BANNER)

        try:
            while rclpy.ok():
                key = self.get_key(settings)

                if key in ['w', 'W', '\x1b[A']:     # Forward / Up Arrow
                    self.target_linear = self.linear_speed
                    self.target_angular = 0.0
                elif key in ['s', 'S', '\x1b[B']:   # Backward / Down Arrow
                    self.target_linear = -self.linear_speed
                    self.target_angular = 0.0
                elif key in ['a', 'A', '\x1b[D']:   # Turn Left / Left Arrow
                    self.target_linear = 0.0
                    self.target_angular = self.angular_speed
                elif key in ['d', 'D', '\x1b[C']:   # Turn Right / Right Arrow
                    self.target_linear = 0.0
                    self.target_angular = -self.angular_speed
                elif key in ['q', 'Q']:             # Forward + Left
                    self.target_linear = self.linear_speed * 0.8
                    self.target_angular = self.angular_speed * 0.8
                elif key in ['e', 'E']:             # Forward + Right
                    self.target_linear = self.linear_speed * 0.8
                    self.target_angular = -self.angular_speed * 0.8
                elif key in ['z', 'Z']:             # Reverse + Left
                    self.target_linear = -self.linear_speed * 0.8
                    self.target_angular = self.angular_speed * 0.8
                elif key in ['c', 'C']:             # Reverse + Right
                    self.target_linear = -self.linear_speed * 0.8
                    self.target_angular = -self.angular_speed * 0.8
                elif key in [' ', 'x', 'X']:        # Stop
                    self.target_linear = 0.0
                    self.target_angular = 0.0
                elif key in ['+', '=']:             # Speed Up
                    self.linear_speed = min(1.0, round(self.linear_speed + 0.05, 2))
                    self.angular_speed = min(2.5, round(self.angular_speed + 0.1, 2))
                    print(f"\r⚡ Speed Up: Lin={self.linear_speed:.2f} m/s | Ang={self.angular_speed:.2f} rad/s    ", end="")
                elif key in ['-', '_']:             # Speed Down
                    self.linear_speed = max(0.1, round(self.linear_speed - 0.05, 2))
                    self.angular_speed = max(0.2, round(self.angular_speed - 0.1, 2))
                    print(f"\r🐢 Speed Down: Lin={self.linear_speed:.2f} m/s | Ang={self.angular_speed:.2f} rad/s    ", end="")
                elif key == '\x03':                 # Ctrl+C
                    break

                # Publish Twist command
                twist = Twist()
                twist.linear.x = float(self.target_linear)
                twist.angular.z = float(self.target_angular)
                self.pub.publish(twist)

                if key != '':
                    status = "STOPPED" if (twist.linear.x == 0 and twist.angular.z == 0) else "DRIVING"
                    print(
                        f"\r🏎️ [{status:<7}] Linear: {twist.linear.x:+.2f} m/s | Angular: {twist.angular.z:+.2f} rad/s    ",
                        end="",
                        flush=True
                    )

        except Exception as e:
            print("\nTeleop encountered error:", e)
        finally:
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
            # Send zero velocity on shutdown
            stop_twist = Twist()
            self.pub.publish(stop_twist)
            print("\n\nTeleop stopped. Robot velocity set to zero.")


def main(args=None):
    rclpy.init(args=args)
    node = TeleopKeyboard()
    try:
        node.run_loop()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
