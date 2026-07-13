#!/usr/bin/env python3
"""在 dev_dex3 容器内运行：从 Unix socket 读取帧数据，发布为本地 CycloneDDS 话题。"""
import os, socket, struct, threading
os.environ['RMW_IMPLEMENTATION'] = 'rmw_cyclonedds_cpp'
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from sensor_msgs.msg import Image

SOCK_PATH = '/run/ros2_cam.sock'
OUT_TOPIC = '/front_camera/image_raw_local'
HDR_FMT = '>III32sIII'
HDR_SIZE = struct.calcsize(HDR_FMT)  # 56 bytes

class BridgeReceiver(Node):
    def __init__(self):
        super().__init__('cam_bridge_receiver')
        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST, depth=5,
            durability=DurabilityPolicy.VOLATILE)
        self._pub = self.create_publisher(Image, OUT_TOPIC, qos)
        threading.Thread(target=self._loop, daemon=True).start()
        self.get_logger().info(f'[cam_bridge_receiver] connecting to {SOCK_PATH} → {OUT_TOPIC}')

    def _loop(self):
        import time
        while rclpy.ok():
            try:
                s = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
                s.connect(SOCK_PATH)
                self.get_logger().info('[cam_bridge_receiver] connected!')
                self._read(s)
                s.close()
            except Exception as e:
                self.get_logger().warn(f'[cam_bridge_receiver] {e}, retry in 1s')
                time.sleep(1.0)

    def _read(self, s):
        while rclpy.ok():
            sz_b = self._recv(s, 4)
            if not sz_b: break
            pkt = self._recv(s, struct.unpack('>I', sz_b)[0])
            if not pkt or len(pkt) < HDR_SIZE: break
            height, width, step, enc_b, sec, nsec, dlen = struct.unpack(HDR_FMT, pkt[:HDR_SIZE])
            frame = pkt[HDR_SIZE:HDR_SIZE+dlen]
            if len(frame) != dlen: break
            msg = Image()
            msg.header.stamp.sec = sec
            msg.header.stamp.nanosec = nsec
            msg.header.frame_id = 'camera_color_optical_frame'
            msg.height = height
            msg.width = width
            msg.encoding = enc_b.rstrip(b'\x00').decode('utf-8', errors='replace')
            msg.step = step
            msg.data = frame
            self._pub.publish(msg)

    def _recv(self, s, n):
        buf = b''
        while len(buf) < n:
            chunk = s.recv(n - len(buf))
            if not chunk: return None
            buf += chunk
        return buf

def main():
    rclpy.init()
    node = BridgeReceiver()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, SystemExit):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()

if __name__ == '__main__':
    main()
