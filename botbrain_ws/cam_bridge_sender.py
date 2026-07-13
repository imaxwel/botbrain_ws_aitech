#!/usr/bin/env python3
"""在 bringup 容器内运行：订阅相机话题，通过 Unix socket 发送帧数据。"""
import os, socket, struct, threading
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from sensor_msgs.msg import Image

SOCK_PATH = '/run/ros2_cam.sock'

class BridgeSender(Node):
    def __init__(self):
        super().__init__('cam_bridge_sender')
        self._sockets = []
        self._sock_lock = threading.Lock()

        if os.path.exists(SOCK_PATH):
            os.unlink(SOCK_PATH)
        self._srv = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
        self._srv.bind(SOCK_PATH)
        self._srv.listen(5)
        os.chmod(SOCK_PATH, 0o666)
        threading.Thread(target=self._accept, daemon=True).start()

        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST, depth=1,
            durability=DurabilityPolicy.VOLATILE)
        self.create_subscription(Image,
            '/g1_robot/front_camera/color/image_raw', self._cb, qos)
        self.get_logger().info(f'[cam_bridge_sender] listening on {SOCK_PATH}')

    def _accept(self):
        while True:
            conn, _ = self._srv.accept()
            with self._sock_lock:
                self._sockets.append(conn)
            self.get_logger().info('[cam_bridge_sender] receiver connected')

    def _cb(self, msg):
        raw = bytes(msg.data)
        enc = msg.encoding.encode()[:31].ljust(32, b'\x00')
        hdr = struct.pack('>III32sIII',
            msg.height, msg.width, msg.step, enc,
            msg.header.stamp.sec, msg.header.stamp.nanosec, len(raw))
        pkt = hdr + raw
        prefix = struct.pack('>I', len(pkt))
        dead = []
        with self._sock_lock:
            for c in self._sockets:
                try:
                    c.sendall(prefix + pkt)
                except Exception:
                    dead.append(c)
            for c in dead:
                self._sockets.remove(c)
                try: c.close()
                except: pass

def main():
    import os as _os, tempfile as _tempfile
    _os.environ.pop('ZENOH_CONFIG_OVERRIDE', None)
    _tmp = _tempfile.NamedTemporaryFile(mode='w', suffix='.json5', prefix='zenoh_rmw_', delete=False)
    _tmp.write('{\n  mode: "client",\n  connect: { endpoints: ["tcp/127.0.0.1:7448"] }\n}\n')
    _tmp.flush()
    _tmp.close()
    _os.environ['ZENOH_CONFIG'] = _tmp.name
    rclpy.init()
    node = BridgeSender()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, SystemExit):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()

if __name__ == '__main__':
    main()
