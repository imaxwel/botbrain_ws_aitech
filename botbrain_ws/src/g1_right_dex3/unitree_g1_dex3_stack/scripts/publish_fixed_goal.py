#!/usr/bin/python3
"""One-shot publisher: sends a fixed Cartesian goal to /goal_pose after a delay."""

import math
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped


def euler_to_quaternion(roll, pitch, yaw):
    """ZYX Euler angles (rpy) → quaternion (x, y, z, w)."""
    cr, sr = math.cos(roll / 2), math.sin(roll / 2)
    cp, sp = math.cos(pitch / 2), math.sin(pitch / 2)
    cy, sy = math.cos(yaw / 2), math.sin(yaw / 2)
    w = cr * cp * cy + sr * sp * sy
    x = sr * cp * cy - cr * sp * sy
    y = cr * sp * cy + sr * cp * sy
    z = cr * cp * sy - sr * sp * cy
    return x, y, z, w


class FixedGoalPublisher(Node):
    def __init__(self):
        super().__init__('fixed_goal_publisher')
        self.declare_parameter('goal_x', 0.2637)
        self.declare_parameter('goal_y', -0.1423)
        self.declare_parameter('goal_z', 0.0058)
        self.declare_parameter('goal_roll', -0.0006)
        self.declare_parameter('goal_pitch', -0.0987)
        self.declare_parameter('goal_yaw', 0.0578)
        self.declare_parameter('goal_frame', 'torso_link')
        self.declare_parameter('publish_delay_s', 5.0)

        self.pub = self.create_publisher(PoseStamped, '/goal_pose', 10)
        delay = self.get_parameter('publish_delay_s').value
        self.timer = self.create_timer(delay, self.publish_once)
        self.published = False

    def publish_once(self):
        if self.published:
            return
        x = self.get_parameter('goal_x').value
        y = self.get_parameter('goal_y').value
        z = self.get_parameter('goal_z').value
        roll = self.get_parameter('goal_roll').value
        pitch = self.get_parameter('goal_pitch').value
        yaw = self.get_parameter('goal_yaw').value
        frame = self.get_parameter('goal_frame').value

        qx, qy, qz, qw = euler_to_quaternion(roll, pitch, yaw)

        msg = PoseStamped()
        msg.header.frame_id = frame
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.pose.position.x = float(x)
        msg.pose.position.y = float(y)
        msg.pose.position.z = float(z)
        msg.pose.orientation.x = float(qx)
        msg.pose.orientation.y = float(qy)
        msg.pose.orientation.z = float(qz)
        msg.pose.orientation.w = float(qw)

        self.pub.publish(msg)
        self.get_logger().info(
            f'Published fixed goal: xyz=[{x:.4f}, {y:.4f}, {z:.4f}] '
            f'rpy=[{roll:.4f}, {pitch:.4f}, {yaw:.4f}] frame={frame}')
        self.published = True
        self.timer.cancel()


def main(args=None):
    rclpy.init(args=args)
    node = FixedGoalPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
