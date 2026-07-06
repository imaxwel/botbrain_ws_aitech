#!/usr/bin/env python3
"""
标定数据采集ROS2节点
订阅AprilTag检测结果和机器人末端位置，记录对应点对
"""
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
import json
from pathlib import Path

class CalibrationCollectorNode(Node):
    def __init__(self):
        super().__init__('calibration_collector')

        self.declare_parameter('save_path', 'calibration_data.json')
        self.save_path = Path(self.get_parameter('save_path').value)

        self.data = {'camera_points': [], 'robot_points': []}
        if self.save_path.exists():
            self._load()

        self.latest_camera_pose = None
        self.latest_robot_pose = None

        self.create_subscription(PoseStamped, '/apriltag/target_pose', self._camera_cb, 10)
        self.create_subscription(PoseStamped, '/robot/end_effector_pose', self._robot_cb, 10)

        self.get_logger().info(f'标定采集节点启动，数据保存到: {self.save_path}')
        self.get_logger().info('按 SPACE 键记录当前点对，按 q 退出')

        self.create_timer(0.1, self._check_keyboard)

    def _camera_cb(self, msg: PoseStamped):
        self.latest_camera_pose = msg

    def _robot_cb(self, msg: PoseStamped):
        self.latest_robot_pose = msg

    def _check_keyboard(self):
        import sys, select, termios, tty
        if select.select([sys.stdin], [], [], 0)[0]:
            old_settings = termios.tcgetattr(sys.stdin)
            try:
                tty.setcbreak(sys.stdin.fileno())
                key = sys.stdin.read(1)
                if key == ' ':
                    self._record_point()
                elif key == 'q':
                    raise SystemExit(0)
            finally:
                termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_settings)

    def _record_point(self):
        if self.latest_camera_pose is None or self.latest_robot_pose is None:
            self.get_logger().warn('缺少相机或机器人位置数据')
            return

        cp = self.latest_camera_pose.pose.position
        rp = self.latest_robot_pose.pose.position

        self.data['camera_points'].append([cp.x, cp.y, cp.z])
        self.data['robot_points'].append([rp.x, rp.y, rp.z])

        n = len(self.data['camera_points'])
        self.get_logger().info(
            f'已记录第 {n} 组数据 - 相机:[{cp.x:.3f},{cp.y:.3f},{cp.z:.3f}] '
            f'机器人:[{rp.x:.3f},{rp.y:.3f},{rp.z:.3f}]'
        )
        self._save()

    def _save(self):
        with open(self.save_path, 'w') as f:
            json.dump(self.data, f, indent=2)

    def _load(self):
        with open(self.save_path, 'r') as f:
            self.data = json.load(f)
        self.get_logger().info(f'已加载 {len(self.data["camera_points"])} 组数据')

def main():
    rclpy.init()
    node = CalibrationCollectorNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, SystemExit):
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
