#!/usr/bin/python3
"""
button_detector_node.py — 电梯按钮视觉检测 ROS2 节点

订阅:
  /camera/color/image_raw          sensor_msgs/Image
  /camera/color/camera_info        sensor_msgs/CameraInfo
  /camera/depth/image_rect_raw     sensor_msgs/Image  (16UC1, mm)

发布:
  /elevator/target_pose   geometry_msgs/PoseStamped  (torso_link 系)
  /elevator/floor_label   std_msgs/String            (识别到的楼层，如 "3")

参数:
  target_floor     int    目标楼层（0=取置信度最高的楼层按钮）
  det_threshold    float  检测置信度阈值，默认 0.5
  output_frame     str    输出坐标系，默认 torso_link
  frozen_model_dir str    frozen_model/ 目录路径
"""
import glob
import os
import sys
import threading
import time
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String
from cv_bridge import CvBridge
from pathlib import Path
import tf2_ros
import tf2_geometry_msgs  # noqa: F401

pass  # ButtonDetector imported after frozen_model_dir param is read (see __init__)


class ButtonDetectorNode(Node):
    def __init__(self):
        super().__init__('button_detector_node')

        self.declare_parameter('target_floor',     0)
        self.declare_parameter('det_threshold',    0.5)
        self.declare_parameter('camera_frame',     'camera_color_optical_frame')
        self.declare_parameter('output_frame',     'torso_link')
        self.declare_parameter('tf_lookup_timeout_s', 0.2)
        self.declare_parameter('frozen_model_dir', '/workspaces/yolonas_ocr/frozen_model')
        self.declare_parameter('input_backend',    'ros')
        self.declare_parameter('video_device',     'auto')
        self.declare_parameter('image_width',      640)
        self.declare_parameter('image_height',     480)
        self.declare_parameter('fps',              3.0)
        self.declare_parameter('fourcc',           'YUYV')
        self.declare_parameter('save_debug_images', True)
        self.declare_parameter('debug_image_dir',  '/workspaces/unitree_dex3/detect_img/button_detector')
        self.declare_parameter('jpeg_quality',     90)
        self.declare_parameter('transform_matrix_path', '')

        self.target_floor = self.get_parameter('target_floor').value
        self.camera_frame = self.get_parameter('camera_frame').value
        self.output_frame = self.get_parameter('output_frame').value
        self.tf_timeout_s = float(self.get_parameter('tf_lookup_timeout_s').value)
        thresh            = self.get_parameter('det_threshold').value
        model_dir         = self.get_parameter('frozen_model_dir').value or None
        self.input_backend = str(self.get_parameter('input_backend').value).lower()
        self.video_device = str(self.get_parameter('video_device').value)
        self.image_width = int(self.get_parameter('image_width').value)
        self.image_height = int(self.get_parameter('image_height').value)
        self.fps = float(self.get_parameter('fps').value)
        self.fourcc = str(self.get_parameter('fourcc').value)
        self.save_debug_images = bool(self.get_parameter('save_debug_images').value)
        self.debug_image_dir = str(self.get_parameter('debug_image_dir').value)
        self.jpeg_quality = int(self.get_parameter('jpeg_quality').value)

        # 从 frozen_model_dir 的父目录导入 button_detector（兼容 install 后的路径）
        yolonas_dir = os.path.dirname(model_dir) if model_dir else ''
        if yolonas_dir and yolonas_dir not in sys.path:
            sys.path.insert(0, yolonas_dir)
        from button_detector import ButtonDetector  # noqa: E402

        self.bridge       = CvBridge()
        self.detector     = ButtonDetector(det_threshold=thresh,
                                           model_dir=model_dir)
        self.camera_info  = None   # (fx, fy, cx, cy)
        self.latest_depth = None   # np.ndarray uint16, mm
        self._capture = None
        self._last_no_floor_log = 0.0
        self._last_image_time = 0.0   # 用于检测图像断流
        # 后台推理线程（避免YOLO阻塞rgb_cb导致RealSense停流）
        self._latest_frame = None     # (img_bgr, depth_snapshot)
        self._frame_lock = threading.Lock()
        self._frame_event = threading.Event()
        threading.Thread(target=self._inference_loop, daemon=True).start()

        # TF2 动态变换
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # 移除静态变换矩阵加载（改用TF2）
        # transform_path = self.get_parameter('transform_matrix_path').value
        # if not transform_path:
        #     transform_path = str(Path(__file__).parent.parent / 'transforms' / 'camera_to_robot.npy')
        # self.T_cam_to_robot = np.load(transform_path)
        # self.get_logger().info(f'已加载变换矩阵: {transform_path}')

        self.pose_pub  = self.create_publisher(PoseStamped, '/apriltag/target_pose', 10)
        self.label_pub = self.create_publisher(String,      '/elevator/floor_label',  10)

        qos = rclpy.qos.QoSPresetProfiles.SENSOR_DATA.value
        self.declare_parameter('image_topic', '/camera/camera/color/image_raw')
        self.declare_parameter('info_topic',  '/camera/camera/color/camera_info')
        self.declare_parameter('depth_topic', '/camera/camera/depth/image_rect_raw')
        self.declare_parameter('capture_trigger_topic', '/apriltag/capture_trigger')

        # 订阅触发话题（与旧方案兼容）
        capture_topic = str(self.get_parameter('capture_trigger_topic').value)
        if capture_topic:
            from std_msgs.msg import Empty
            self.create_subscription(Empty, capture_topic, self._trigger_cb, 10)
            self.get_logger().info(f'subscribed to capture trigger: {capture_topic}')

        if self.input_backend == 'v4l2':
            self._init_v4l2()
            period = 1.0 / max(self.fps, 0.1)
            self.create_timer(period, self._v4l2_tick)
        else:
            self.create_subscription(CameraInfo, self.get_parameter('info_topic').value,
                                     self._info_cb,  qos)
            self.create_subscription(Image, self.get_parameter('depth_topic').value,
                                     self._depth_cb, qos)
            self.create_subscription(Image, self.get_parameter('image_topic').value,
                                     self._rgb_cb,   qos)

        self.get_logger().info(
            f'button_detector_node ready  backend={self.input_backend} '
            f'threshold={thresh} target_floor={self.target_floor} '
            f'output_frame={self.output_frame}')

    def _trigger_cb(self, msg):
        """响应按压节点的capture trigger（与旧方案兼容）"""
        age = time.monotonic() - self._last_image_time
        if self._last_image_time == 0.0 or age > 3.0:
            self.get_logger().warn(
                f'capture trigger received but no image in {age:.1f}s — '
                'check RealSense is publishing!')
        else:
            self.get_logger().info('received capture trigger, processing next frame...')

    def _info_cb(self, msg: CameraInfo):
        if self.camera_info is None:
            k = msg.k
            self.camera_info = (k[0], k[4], k[2], k[5])  # fx, fy, cx, cy
            self.get_logger().info(
                f'camera_info  fx={k[0]:.1f} fy={k[4]:.1f} '
                f'cx={k[2]:.1f} cy={k[5]:.1f}')

    def _depth_cb(self, msg: Image):
        self.latest_depth = self.bridge.imgmsg_to_cv2(
            msg, desired_encoding='passthrough')

    def _rgb_cb(self, msg: Image):
        self._last_image_time = time.monotonic()
        if self.camera_info is None or self.latest_depth is None:
            return
        import cv2
        img_bgr = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        # 只存最新帧，立刻返回，不在回调中跑YOLO
        with self._frame_lock:
            self._latest_frame = (img_bgr, self.latest_depth.copy())
        self._frame_event.set()

    def _inference_loop(self):
        """后台线程：消费最新帧跑YOLO推理，不阻塞ROS2 executor"""
        self.get_logger().info('inference thread started')
        while rclpy.ok():
            if not self._frame_event.wait(timeout=1.0):
                continue
            self._frame_event.clear()
            with self._frame_lock:
                frame_data = self._latest_frame
            if frame_data is None:
                continue
            img_bgr, depth = frame_data
            try:
                self._process_bgr(img_bgr, depth=depth)
            except Exception as e:
                self.get_logger().error(f'inference error: {e}', throttle_duration_sec=5.0)

    def _process_bgr(self, img_bgr, stamp=None, depth=None):
        results = self.detector.detect(img_bgr)
        self._save_debug_image(img_bgr, results)

        floors = [r for r in results if r['is_floor']]
        if not floors:
            now = time.monotonic()
            if now - self._last_no_floor_log > 2.0:
                self._last_no_floor_log = now
                self.get_logger().info(f'no floor detected; candidates={len(results)}')
            return

        if self.target_floor:
            candidate = next(
                (r for r in floors if r['text'] == str(self.target_floor)), None)
            if candidate is None:
                self.get_logger().warn(
                    f'target floor {self.target_floor} not detected')
                return
        else:
            candidate = floors[0]  # 已按 score 降序

        lbl = String()
        lbl.data = candidate['text']
        self.label_pub.publish(lbl)

        if self.camera_info is None or depth is None:
            self.get_logger().info(
                f"floor='{candidate['text']}' conf={candidate['score']:.2f} "
                f"pixel({candidate['cx']},{candidate['cy']})")
            return

        # ── 深度投影 ───────────────────────────────────────────────────────────
        cx_px, cy_px = candidate['cx'], candidate['cy']
        dh, dw = depth.shape[:2]
        ih, iw = img_bgr.shape[:2]
        sx, sy = dw / iw, dh / ih

        # 用整个bbox区域采样深度（内收10%避免背景），应对发光按键中心depth无效问题
        x1b, y1b, x2b, y2b = candidate['bbox']
        shrink_x = max(1, int((x2b - x1b) * 0.1 * sx))
        shrink_y = max(1, int((y2b - y1b) * 0.1 * sy))
        rx1 = max(0, int(x1b * sx) + shrink_x)
        rx2 = min(dw, int(x2b * sx) - shrink_x)
        ry1 = max(0, int(y1b * sy) + shrink_y)
        ry2 = min(dh, int(y2b * sy) - shrink_y)
        roi = depth[ry1:ry2, rx1:rx2]
        valid_depths = roi[roi > 0]

        # 降级：bbox采样不足时回退到中心点9×9
        if len(valid_depths) < 5:
            dx, dy = int(cx_px * sx), int(cy_px * sy)
            roi = depth[max(0, dy-4):dy+5, max(0, dx-4):dx+5]
            valid_depths = roi[roi > 0]

        if len(valid_depths) == 0:
            self.get_logger().warn(f'no valid depth in button bbox ({x1b},{y1b},{x2b},{y2b})')
            return

        # 取25th百分位：发光中心depth无效→有效点偏向按键物理面近侧
        depth_mm = float(np.percentile(valid_depths, 25))

        if depth_mm <= 0 or depth_mm > 3000:
            self.get_logger().warn(f'invalid depth {depth_mm:.0f}mm at ({dx},{dy})')
            return

        fx, fy, cx, cy = self.camera_info
        d = depth_mm / 1000.0

        # 相机坐标系中的3D点（用Time(0)请求最新TF，避免时间戳过期）
        pose_cam = PoseStamped()
        pose_cam.header.stamp = rclpy.time.Time().to_msg()
        pose_cam.header.frame_id = self.camera_frame
        pose_cam.pose.position.x = (cx_px - cx) * d / fx
        pose_cam.pose.position.y = (cy_px - cy) * d / fy
        pose_cam.pose.position.z = d
        pose_cam.pose.orientation.w = 1.0

        # 使用TF2动态变换到机器人坐标系
        try:
            timeout = Duration(seconds=self.tf_timeout_s)
            pose_robot = self.tf_buffer.transform(pose_cam, self.output_frame, timeout=timeout)

            self.pose_pub.publish(pose_robot)

            self.get_logger().info(
                f"floor='{candidate['text']}' conf={candidate['score']:.2f} "
                f"depth={d:.3f}m  torso({pose_robot.pose.position.x:.3f},"
                f"{pose_robot.pose.position.y:.3f},{pose_robot.pose.position.z:.3f})")
        except tf2_ros.TransformException as ex:
            self.get_logger().warn(f'TF transform failed: {ex}')

    def _init_v4l2(self):
        import cv2
        os.makedirs(self.debug_image_dir, exist_ok=True)
        for device in self._candidate_video_devices():
            cap = cv2.VideoCapture(device, cv2.CAP_V4L2)
            if not cap.isOpened():
                cap.release()
                continue

            cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
            cap.set(cv2.CAP_PROP_FRAME_WIDTH, float(self.image_width))
            cap.set(cv2.CAP_PROP_FRAME_HEIGHT, float(self.image_height))
            cap.set(cv2.CAP_PROP_FPS, self.fps)
            if len(self.fourcc) == 4:
                cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*self.fourcc))

            ok, frame = cap.read()
            if ok and frame is not None:
                self._capture = cap
                self.video_device = device
                self.get_logger().info(
                    f'v4l2 capture opened: {device} '
                    f'{frame.shape[1]}x{frame.shape[0]}@{self.fps:.1f}')
                return
            cap.release()

        raise RuntimeError('no readable V4L2 video device found')

    def _candidate_video_devices(self):
        if self.video_device and self.video_device.lower() != 'auto':
            return [self.video_device]
        candidates = []
        candidates.extend(sorted(glob.glob('/dev/v4l/by-path/*1.3-video-index0')))
        candidates.extend(sorted(glob.glob('/dev/v4l/by-path/*video-index0')))
        candidates.extend(sorted(glob.glob('/dev/video*')))
        out = []
        seen = set()
        for path in candidates:
            if path not in seen:
                out.append(path)
                seen.add(path)
        return out

    def _v4l2_tick(self):
        if self._capture is None:
            return
        ok, frame = self._capture.read()
        if not ok or frame is None:
            self.get_logger().warn('failed to read V4L2 frame')
            return
        self._process_bgr(frame)

    def _save_debug_image(self, img_bgr, results):
        if not self.save_debug_images:
            return
        import cv2
        out = img_bgr.copy()
        for result in results:
            x1, y1, x2, y2 = result['bbox']
            color = (0, 255, 0) if result['is_floor'] else (0, 165, 255)
            cv2.rectangle(out, (x1, y1), (x2, y2), color, 2)
            label = f"{result['text']} {result['score']:.2f}"
            cv2.putText(out, label, (x1, max(0, y1 - 6)),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, color, 2)
        cv2.imwrite(
            os.path.join(self.debug_image_dir, 'latest_button_detector.jpg'),
            out,
            [cv2.IMWRITE_JPEG_QUALITY, self.jpeg_quality],
        )

    def destroy_node(self):
        if self._capture is not None:
            self._capture.release()
            self._capture = None
        self.detector.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = ButtonDetectorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
