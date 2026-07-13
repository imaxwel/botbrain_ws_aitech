#!/usr/bin/env python3
import os, sys, struct, tempfile, time
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from sensor_msgs.msg import Image

OUT_PATH = '/run/latest_cam.bin'
HDR_FMT = '>III32sIII'
WATCHDOG_TIMEOUT_S = 15.0

class CamFrameWriter(Node):
    def __init__(self):
        super().__init__('cam_frame_writer')
        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST, depth=10,
            durability=DurabilityPolicy.VOLATILE)
        self.create_subscription(Image, '/g1_robot/front_camera/color/image_raw', self._cb, qos)
        self._count = 0
        self._last_frame_time = time.monotonic()
        self.create_timer(5.0, self._watchdog)
        self.get_logger().info(f'[cam_frame_writer] writing to {OUT_PATH}')

    def _cb(self, msg):
        raw = bytes(msg.data)
        enc = msg.encoding.encode()[:31].ljust(32, b'\x00')
        hdr = struct.pack(HDR_FMT,
            msg.height, msg.width, msg.step, enc,
            msg.header.stamp.sec, msg.header.stamp.nanosec, len(raw))
        try:
            d = os.path.dirname(OUT_PATH) or '.'
            with tempfile.NamedTemporaryFile(dir=d, delete=False) as f:
                f.write(hdr + raw)
                tmp = f.name
            os.replace(tmp, OUT_PATH)
            self._last_frame_time = time.monotonic()
            self._count += 1
            if self._count == 1 or self._count % 60 == 0:
                self.get_logger().info(
                    f'[cam_frame_writer] frame #{self._count} {msg.width}x{msg.height} enc={msg.encoding}')
        except Exception as e:
            self.get_logger().warn(f'[cam_frame_writer] write failed: {e}')

    def _watchdog(self):
        if self._count == 0 and time.monotonic() - self._last_frame_time < 30.0:
            return
        elapsed = time.monotonic() - self._last_frame_time
        if elapsed > WATCHDOG_TIMEOUT_S:
            self.get_logger().warn(
                f'[cam_frame_writer] no frame for {elapsed:.0f}s — restarting process')
            sys.stdout.flush()
            sys.stderr.flush()
            os.execv(sys.executable, [sys.executable] + sys.argv)

def main():
    rclpy.init()
    node = CamFrameWriter()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, SystemExit):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()

if __name__ == '__main__':
    main()
