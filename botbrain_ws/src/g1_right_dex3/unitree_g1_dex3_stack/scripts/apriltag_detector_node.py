#!/usr/bin/python3
"""ROS 2 node: detect AprilTag 36h11 in the RealSense color stream,
filter low-quality / wrong-id detections, apply a tag-local XYZ offset,
transform both the raw tag pose and the offset target pose into
``torso_link`` via tf2, publish them as ``geometry_msgs/PoseStamped``,
and (optionally) overlay corners + 3-axis + HUD in an OpenCV window.

Pipeline (per CONTEXT D-01..D-20):
    image_cb → cv_bridge → pupil_apriltags.detect(estimate_tag_pose=True)
        → filter (target_tag_id, hamming==0, decision_margin>=min)
        → T_cam_target = T_cam_tag · Translate(offset_xyz)  [tag-local]
        → tf_buffer.transform(pose, output_frame, timeout)
        → publish PoseStamped on /apriltag/{tag_pose,target_pose}
        → cv2.polylines + cv2.projectPoints + HUD (if imshow enabled)

All knobs come from parameters declared via ``declare_parameter``;
nothing about tag size / target id / margin / offset / topics is
hardcoded.
"""

import collections
import queue
import threading
import time
from http.server import HTTPServer, BaseHTTPRequestHandler

import numpy as np
import cv2

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.duration import Duration

from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import PoseStamped
from cv_bridge import CvBridge, CvBridgeError

import tf2_ros
import tf2_geometry_msgs  # noqa: F401  side-effect: registers PoseStamped/Pose plugin

from pupil_apriltags import Detector
from scipy.spatial.transform import Rotation as R


class AprilTagDetectorNode(Node):
    """Single-purpose AprilTag detector — see module docstring."""

    def __init__(self):
        # Node name MUST match config/apriltag.yaml top-level key.
        super().__init__('apriltag_detector')

        # ---------- declare parameters (11 from YAML + 1 launch-arg) ----------
        self.declare_parameter('tag_family', 'tag36h11')
        self.declare_parameter('tag_size', 0.08)
        self.declare_parameter('target_tag_id', 0)
        self.declare_parameter('offset_xyz', [0.0, 0.0, 0.05])
        self.declare_parameter('decision_margin_min', 25.0)
        self.declare_parameter('output_frame', 'torso_link')
        self.declare_parameter('rgb_topic', '/camera/color/image_raw')
        self.declare_parameter('camera_info_topic', '/camera/color/camera_info')
        self.declare_parameter('tag_pose_topic', '/apriltag/tag_pose')
        self.declare_parameter('target_pose_topic', '/apriltag/target_pose')
        self.declare_parameter('tf_lookup_timeout_s', 0.02)
        # imshow: launch arg (D-15), default false for headless/container use
        self.declare_parameter('imshow', False)
        self.declare_parameter('stream_port', 0)

        # ---------- read parameters into instance state ----------
        self.tag_family = self.get_parameter('tag_family').value
        self.tag_size = float(self.get_parameter('tag_size').value)
        self.target_tag_id = int(self.get_parameter('target_tag_id').value)
        offset_list = list(self.get_parameter('offset_xyz').value)
        self.offset_xyz = np.asarray(offset_list, dtype=np.float64).reshape(3)
        self.decision_margin_min = float(
            self.get_parameter('decision_margin_min').value)
        self.output_frame = self.get_parameter('output_frame').value
        self.rgb_topic = self.get_parameter('rgb_topic').value
        self.camera_info_topic = self.get_parameter('camera_info_topic').value
        self.tag_pose_topic = self.get_parameter('tag_pose_topic').value
        self.target_pose_topic = self.get_parameter('target_pose_topic').value
        self.tf_lookup_timeout_s = float(
            self.get_parameter('tf_lookup_timeout_s').value)
        self.imshow = bool(self.get_parameter('imshow').value)
        self.imshow_enabled = self.imshow  # mutable so 'q' key can disable
        self.stream_port = int(self.get_parameter('stream_port').value)
        self.stream_enabled = self.stream_port > 0

        # ---------- detector + helpers ----------
        self.detector = Detector(
            families=self.tag_family,
            nthreads=1,
            quad_decimate=2.0,
            refine_edges=1,
        )
        self.bridge = CvBridge()

        # ---------- TF ----------
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # ---------- camera intrinsics (filled on first CameraInfo) ----------
        self.camera_params = None        # (fx, fy, cx, cy)
        self.dist_coeffs = None          # np.ndarray shape (N,)
        self.camera_matrix = None        # 3x3 K matrix, cached for projectPoints

        # ---------- FPS sliding window (D-20) ----------
        self.frame_times = collections.deque(maxlen=30)
        self.last_processed_time = 0.0

        # ---------- log throttling ----------
        self.last_warn_time = 0.0
        self.last_info_warn_time = 0.0
        self._info_logged_once = False

        # ---------- MJPEG stream ----------
        self._stream_frame = None  # latest JPEG bytes
        self._stream_lock = threading.Lock()
        if self.stream_enabled:
            self._start_mjpeg_server()

        # ---------- OpenCV display thread (D-18) ----------
        self.window_name = f"AprilTag detector (id={self.target_tag_id})"
        self._display_queue = queue.Queue(maxsize=1)  # drop old frames
        if self.imshow_enabled:
            cv2.namedWindow(self.window_name, cv2.WINDOW_AUTOSIZE)
            self._display_timer = self.create_timer(0.03, self._display_timer_cb)
        else:
            self._display_timer = None

        # ---------- subs (sensor_data QoS — RealSense is BEST_EFFORT) ----------
        self.create_subscription(
            Image, self.rgb_topic, self.image_cb, qos_profile_sensor_data)
        self.create_subscription(
            CameraInfo, self.camera_info_topic, self.info_cb,
            qos_profile_sensor_data)

        # ---------- pubs (default reliable QoS, depth 10) ----------
        self.tag_pose_pub = self.create_publisher(
            PoseStamped, self.tag_pose_topic, 10)
        self.target_pose_pub = self.create_publisher(
            PoseStamped, self.target_pose_topic, 10)

        self.get_logger().info(
            f"AprilTag detector ready: family={self.tag_family} "
            f"target_id={self.target_tag_id} tag_size={self.tag_size}m "
            f"output_frame={self.output_frame} imshow={self.imshow_enabled} "
            f"stream={'http://0.0.0.0:' + str(self.stream_port) + '/stream' if self.stream_enabled else 'off'}")

    # ------------------------------------------------------------------
    # CameraInfo callback — cache intrinsics on first arrival
    # ------------------------------------------------------------------
    def info_cb(self, msg: CameraInfo):
        if self.camera_params is not None:
            return  # cached on first call; intrinsics are constant for the stream
        fx = float(msg.k[0])
        fy = float(msg.k[4])
        cx = float(msg.k[2])
        cy = float(msg.k[5])
        self.camera_params = (fx, fy, cx, cy)
        self.camera_matrix = np.array(
            [[fx, 0.0, cx],
             [0.0, fy, cy],
             [0.0, 0.0, 1.0]], dtype=np.float64)

        d = np.asarray(msg.d, dtype=np.float64) if msg.d else np.zeros(5)
        if d.size == 0 or not np.any(d):
            self.dist_coeffs = np.zeros(5, dtype=np.float64)
        else:
            self.dist_coeffs = d

        self.get_logger().info(
            f"CameraInfo cached: fx={fx:.2f} fy={fy:.2f} "
            f"cx={cx:.2f} cy={cy:.2f}")

    # ------------------------------------------------------------------
    # Image callback — full detect → filter → offset → TF → publish + viz
    # ------------------------------------------------------------------
    def image_cb(self, msg: Image):
        # FPS bookkeeping (D-20: sliding window over last 30 frames)
        now = time.monotonic()
        self.frame_times.append(now)

        # Skip frames to limit detection rate to ~3 Hz
        if now - self.last_processed_time < 0.33:
            return
        self.last_processed_time = now

        if self.camera_params is None:
            self._warn_camera_info_missing()
            return

        # cv_bridge → BGR8 ndarray
        try:
            bgr = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except CvBridgeError as ex:
            self._warn_throttled(f"cv_bridge error: {ex}")
            return

        gray = cv2.cvtColor(bgr, cv2.COLOR_BGR2GRAY)

        # Always downscale to half resolution for detection (critical on ARM/Tegra)
        scale = 0.5
        h, w = gray.shape[:2]
        gray_small = cv2.resize(gray, (int(w * scale), int(h * scale)),
                                interpolation=cv2.INTER_AREA)
        fx, fy, cx, cy = self.camera_params
        cam_params_small = (fx * scale, fy * scale, cx * scale, cy * scale)

        detections = self.detector.detect(
            gray_small,
            estimate_tag_pose=True,
            camera_params=cam_params_small,
            tag_size=self.tag_size,
        )

        # Scale corners back to original resolution for viz overlay
        for d in detections:
            d.corners = d.corners / scale
            d.center = d.center / scale

        # Prepare display canvas if imshow or streaming is active
        display = bgr.copy() if (self.imshow_enabled or self.stream_enabled) else None

        # Track best margin across all detections (for HUD)
        best_margin = 0.0
        accepted_poses = []

        for d in detections:
            if d.decision_margin > best_margin:
                best_margin = float(d.decision_margin)

            # ---------- filter chain (D-07 + D-11 + D-10) ----------
            accepted = (
                (d.tag_id == self.target_tag_id)
                and (d.hamming == 0)
                and (d.decision_margin >= self.decision_margin_min)
            )

            # ---------- visualisation: corners + id text + axes ----------
            if display is not None:
                color = (0, 255, 0) if accepted else (0, 0, 255)  # BGR green/red
                corners_int = d.corners.astype(int)
                cv2.polylines(
                    display, [corners_int], isClosed=True,
                    color=color, thickness=2)
                label_org = tuple(corners_int[0] + np.array([0, -8]))
                cv2.putText(
                    display,
                    f"id={d.tag_id}",
                    label_org,
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.5,
                    color,
                    1,
                    cv2.LINE_AA,
                )

                if accepted:
                    # 3-axis projection (3 cm axes, x=red, y=green, z=blue)
                    obj_pts = np.array(
                        [[0.0, 0.0, 0.0],
                         [0.03, 0.0, 0.0],
                         [0.0, 0.03, 0.0],
                         [0.0, 0.0, 0.03]], dtype=np.float64)
                    rvec = cv2.Rodrigues(d.pose_R)[0]
                    tvec = np.asarray(d.pose_t, dtype=np.float64).reshape(3, 1)
                    img_pts, _ = cv2.projectPoints(
                        obj_pts, rvec, tvec,
                        self.camera_matrix, self.dist_coeffs)
                    img_pts = img_pts.reshape(-1, 2).astype(int)
                    o, x, y, z = img_pts
                    cv2.line(display, tuple(o), tuple(x), (0, 0, 255), 2)   # x red
                    cv2.line(display, tuple(o), tuple(y), (0, 255, 0), 2)   # y green
                    cv2.line(display, tuple(o), tuple(z), (255, 0, 0), 2)   # z blue

            # No publish for rejected detections (event-driven, D-03)
            if not accepted:
                continue

            # ---------- raw tag PoseStamped in camera_color_optical_frame ----------
            tvec = np.asarray(d.pose_t, dtype=np.float64).reshape(3)
            quat = R.from_matrix(d.pose_R).as_quat()  # [qx, qy, qz, qw]

            pose_cam = PoseStamped()
            pose_cam.header.stamp = msg.header.stamp
            pose_cam.header.frame_id = "camera_color_optical_frame"
            pose_cam.pose.position.x = float(tvec[0])
            pose_cam.pose.position.y = float(tvec[1])
            pose_cam.pose.position.z = float(tvec[2])
            pose_cam.pose.orientation.x = float(quat[0])
            pose_cam.pose.orientation.y = float(quat[1])
            pose_cam.pose.orientation.z = float(quat[2])
            pose_cam.pose.orientation.w = float(quat[3])

            # ---------- target pose in camera frame (tag-local offset, D-01) ----------
            T_cam_tag = np.eye(4, dtype=np.float64)
            T_cam_tag[:3, :3] = d.pose_R
            T_cam_tag[:3, 3] = tvec

            T_tag_target = np.eye(4, dtype=np.float64)
            T_tag_target[:3, 3] = self.offset_xyz

            T_cam_target = T_cam_tag @ T_tag_target

            target_quat = R.from_matrix(T_cam_target[:3, :3]).as_quat()

            target_pose_cam = PoseStamped()
            target_pose_cam.header.stamp = msg.header.stamp
            target_pose_cam.header.frame_id = "camera_color_optical_frame"
            target_pose_cam.pose.position.x = float(T_cam_target[0, 3])
            target_pose_cam.pose.position.y = float(T_cam_target[1, 3])
            target_pose_cam.pose.position.z = float(T_cam_target[2, 3])
            target_pose_cam.pose.orientation.x = float(target_quat[0])
            target_pose_cam.pose.orientation.y = float(target_quat[1])
            target_pose_cam.pose.orientation.z = float(target_quat[2])
            target_pose_cam.pose.orientation.w = float(target_quat[3])

            accepted_poses.append((pose_cam, target_pose_cam))

        # ---------- HUD + window blit (only if imshow active) ----------
        if display is not None:
            fps = 0.0
            if len(self.frame_times) >= 2:
                span = self.frame_times[-1] - self.frame_times[0]
                if span > 0.0:
                    fps = (len(self.frame_times) - 1) / span

            hud = (
                f"margin={best_margin:.1f}  "
                f"fps={fps:.1f}  "
                f"id={self.target_tag_id}"
            )
            (text_w, text_h), _ = cv2.getTextSize(
                hud, cv2.FONT_HERSHEY_SIMPLEX, 0.5, 1)
            h, w = display.shape[:2]
            margin_px = 8
            x0 = w - text_w - margin_px - 4
            y0 = h - margin_px - text_h - 4
            cv2.rectangle(
                display,
                (x0 - 4, y0 - 4),
                (x0 + text_w + 4, y0 + text_h + 6),
                (0, 0, 0),
                cv2.FILLED,
            )
            cv2.putText(
                display,
                hud,
                (x0, y0 + text_h),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.5,
                (255, 255, 255),
                1,
                cv2.LINE_AA,
            )

            display_show = cv2.resize(display, (640, 480))

            # MJPEG stream: encode and store
            if self.stream_enabled:
                _, jpeg = cv2.imencode('.jpg', display_show,
                                       [cv2.IMWRITE_JPEG_QUALITY, 70])
                with self._stream_lock:
                    self._stream_frame = jpeg.tobytes()

            # Local window
            if self.imshow_enabled:
                try:
                    self._display_queue.put_nowait(display_show)
                except queue.Full:
                    pass  # drop frame, display thread is busy

        # ---------- TF transform + publish (after display to keep window responsive) ----------
        if accepted_poses:
            timeout = Duration(seconds=self.tf_lookup_timeout_s)
            for pose_cam, target_pose_cam in accepted_poses:
                try:
                    pose_torso = self.tf_buffer.transform(
                        pose_cam, self.output_frame, timeout=timeout)
                    target_torso = self.tf_buffer.transform(
                        target_pose_cam, self.output_frame, timeout=timeout)
                except tf2_ros.TransformException as ex:
                    self._warn_throttled(f"TF transform failed: {ex}")
                    continue
                self.tag_pose_pub.publish(pose_torso)
                self.target_pose_pub.publish(target_torso)

    # ------------------------------------------------------------------
    # OpenCV display worker (runs in dedicated thread, never blocks ROS)
    # ------------------------------------------------------------------
    def _display_timer_cb(self):
        if not self.imshow_enabled:
            if self._display_timer is not None:
                self.destroy_timer(self._display_timer)
                self._display_timer = None
            return

        frame = None
        try:
            while True:
                frame = self._display_queue.get_nowait()
        except queue.Empty:
            pass

        if frame is not None:
            cv2.imshow(self.window_name, frame)

        key = cv2.waitKey(1) & 0xFF
        if key == ord('q'):
            cv2.destroyWindow(self.window_name)
            self.imshow_enabled = False
            self.get_logger().info(
                "imshow disabled by 'q' key (node continues publishing)")

    # ------------------------------------------------------------------
    # MJPEG HTTP server
    # ------------------------------------------------------------------
    def _start_mjpeg_server(self):
        node_ref = self

        class MjpegHandler(BaseHTTPRequestHandler):
            def do_GET(self):
                if self.path == '/stream':
                    self.send_response(200)
                    self.send_header('Content-Type',
                                     'multipart/x-mixed-replace; boundary=frame')
                    self.end_headers()
                    try:
                        while True:
                            with node_ref._stream_lock:
                                frame = node_ref._stream_frame
                            if frame is None:
                                time.sleep(0.1)
                                continue
                            self.wfile.write(b'--frame\r\n')
                            self.wfile.write(b'Content-Type: image/jpeg\r\n')
                            self.wfile.write(f'Content-Length: {len(frame)}\r\n'.encode())
                            self.wfile.write(b'\r\n')
                            self.wfile.write(frame)
                            self.wfile.write(b'\r\n')
                            time.sleep(0.15)  # ~6-7 fps to clients
                    except (BrokenPipeError, ConnectionResetError):
                        pass
                else:
                    self.send_response(200)
                    self.send_header('Content-Type', 'text/html')
                    self.end_headers()
                    self.wfile.write(
                        b'<html><body style="margin:0;background:#000">'
                        b'<img src="/stream" style="width:100%;height:auto">'
                        b'</body></html>')

            def log_message(self, format, *args):
                pass  # suppress per-request logs

        server = HTTPServer(('0.0.0.0', self.stream_port), MjpegHandler)
        t = threading.Thread(target=server.serve_forever, daemon=True)
        t.start()
        self.get_logger().info(
            f"MJPEG stream server started on http://0.0.0.0:{self.stream_port}/stream")

    # ------------------------------------------------------------------
    # helpers
    # ------------------------------------------------------------------
    def _warn_throttled(self, msg: str, period_s: float = 2.0):
        """Manual throttle (portable across rclpy distros)."""
        now = time.monotonic()
        if now - self.last_warn_time >= period_s:
            self.get_logger().warn(msg)
            self.last_warn_time = now

    def _warn_camera_info_missing(self, period_s: float = 5.0):
        now = time.monotonic()
        if now - self.last_info_warn_time >= period_s:
            self.get_logger().warn(
                f"Waiting for CameraInfo on {self.camera_info_topic}...")
            self.last_info_warn_time = now


def main(args=None):
    rclpy.init(args=args)
    node = AprilTagDetectorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node.imshow_enabled:
            cv2.destroyAllWindows()
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
