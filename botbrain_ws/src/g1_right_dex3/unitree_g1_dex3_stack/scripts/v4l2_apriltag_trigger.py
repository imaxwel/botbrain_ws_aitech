#!/usr/bin/python3

import glob
import math
import os
import select
import subprocess
import threading
import termios
import time
import tty

import cv2
import numpy as np

import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time

from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import CameraInfo
from std_msgs.msg import Empty
from trajectory_msgs.msg import JointTrajectory

import tf2_geometry_msgs  # noqa: F401
import tf2_ros

from pupil_apriltags import Detector
from scipy.spatial.transform import Rotation as R


def _as_bool(value):
    if isinstance(value, bool):
        return value
    if isinstance(value, str):
        return value.strip().lower() in ('1', 'true', 'yes', 'on')
    return bool(value)


class V4L2AprilTagTrigger(Node):
    def __init__(self):
        super().__init__('v4l2_apriltag_trigger')

        self.declare_parameter('camera_id', 'd435i_front')
        self.declare_parameter('expected_serial', '253243060636')
        self.declare_parameter(
            'video_device',
            'auto')
        self.declare_parameter('expected_usb_interface_num', '03')
        self.declare_parameter('image_width', 640)
        self.declare_parameter('image_height', 480)
        self.declare_parameter('fps', 6.0)
        self.declare_parameter('fourcc', 'YUYV')
        self.declare_parameter('sample_count', 4)
        self.declare_parameter('warmup_frames', 12)
        self.declare_parameter('warmup_min_s', 2.0)
        self.declare_parameter('sample_interval_s', 0.05)
        self.declare_parameter('continuous_capture', False)

        self.declare_parameter('tag_family', 'tag36h11')
        self.declare_parameter('tag_size', 0.08)
        self.declare_parameter('target_tag_id', 0)
        self.declare_parameter('offset_xyz', [0.0, 0.0, 0.0])
        self.declare_parameter('decision_margin_min', 25.0)
        self.declare_parameter('quad_decimate', 2.0)
        self.declare_parameter('detect_scale', 0.5)

        self.declare_parameter('camera_matrix', [
            602.0224609375, 0.0, 330.956695556641,
            0.0, 601.472839355469, 256.269927978516,
            0.0, 0.0, 1.0,
        ])
        self.declare_parameter('dist_coeffs', [0.0, 0.0, 0.0, 0.0, 0.0])
        self.declare_parameter(
            'camera_info_topic', '/camera/realsense2_camera/color/camera_info')
        self.declare_parameter('use_live_camera_info', True)
        self.declare_parameter('camera_frame', 'camera_color_optical_frame')
        self.declare_parameter('output_frame', 'torso_link')
        self.declare_parameter('tf_lookup_timeout_s', 0.2)
        self.declare_parameter('tf_stable_required', True)
        self.declare_parameter('tf_stable_wait_s', 3.0)
        self.declare_parameter('tf_stable_sample_count', 5)
        self.declare_parameter('tf_stable_sample_interval_s', 0.15)
        self.declare_parameter('tf_stable_translation_tol_m', 0.003)
        self.declare_parameter('tf_stable_rotation_tol_deg', 0.5)
        self.declare_parameter('tf_stable_source_frame', 'camera_color_optical_frame')

        self.declare_parameter('goal_pose_topic', '/goal_pose')
        self.declare_parameter('tag_pose_topic', '/apriltag/tag_pose')
        self.declare_parameter('target_pose_topic', '/apriltag/target_pose')
        self.declare_parameter('joint_trajectory_topic', '/joint_trajectory_targets')
        self.declare_parameter('reach_max_distance', 0.55)
        self.declare_parameter('trigger_key', 'g')
        self.declare_parameter('trigger_topic', '')
        self.declare_parameter('publish_intermediate_poses', True)
        self.declare_parameter('detect_only', False)
        self.declare_parameter('fixed_orientation_enabled', False)
        self.declare_parameter('fixed_rpy', [-0.0873, -0.0340, 0.0199])

        self.declare_parameter('save_debug_images', True)
        self.declare_parameter('debug_image_dir', '/botbrain_ws/detect_img')
        self.declare_parameter('save_raw_images', False)
        self.declare_parameter('jpeg_quality', 90)
        # Baseline compensation parameters
        self.declare_parameter('baseline_compensation_enabled', False)
        self.declare_parameter('baseline_x', 0.0)
        self.declare_parameter('baseline_y', 0.0)
        self.declare_parameter('baseline_z', 0.0)

        self.camera_id = str(self.get_parameter('camera_id').value)
        self.expected_serial = str(self.get_parameter('expected_serial').value)
        self.video_device = str(self.get_parameter('video_device').value)
        self.configured_video_device = self.video_device
        self.expected_usb_interface_num = str(
            self.get_parameter('expected_usb_interface_num').value)
        self.image_width = int(self.get_parameter('image_width').value)
        self.image_height = int(self.get_parameter('image_height').value)
        self.fps = float(self.get_parameter('fps').value)
        self.fourcc = str(self.get_parameter('fourcc').value)
        self.sample_count = int(self.get_parameter('sample_count').value)
        self.warmup_frames = int(self.get_parameter('warmup_frames').value)
        self.warmup_min_s = float(self.get_parameter('warmup_min_s').value)
        self.sample_interval_s = float(
            self.get_parameter('sample_interval_s').value)
        self.continuous_capture = _as_bool(
            self.get_parameter('continuous_capture').value)

        self.tag_family = str(self.get_parameter('tag_family').value)
        self.tag_size = float(self.get_parameter('tag_size').value)
        self.target_tag_id = int(self.get_parameter('target_tag_id').value)
        self.offset_xyz = np.asarray(
            list(self.get_parameter('offset_xyz').value), dtype=np.float64).reshape(3)
        self.decision_margin_min = float(
            self.get_parameter('decision_margin_min').value)
        self.detect_scale = float(self.get_parameter('detect_scale').value)

        camera_matrix = list(self.get_parameter('camera_matrix').value)
        self.camera_matrix_fallback = np.asarray(
            camera_matrix, dtype=np.float64).reshape(3, 3)
        self.camera_params_fallback = (
            float(self.camera_matrix_fallback[0, 0]),
            float(self.camera_matrix_fallback[1, 1]),
            float(self.camera_matrix_fallback[0, 2]),
            float(self.camera_matrix_fallback[1, 2]),
        )
        self.dist_coeffs_fallback = np.asarray(
            list(self.get_parameter('dist_coeffs').value), dtype=np.float64)
        self.camera_info_topic = str(
            self.get_parameter('camera_info_topic').value)
        self.use_live_camera_info = _as_bool(
            self.get_parameter('use_live_camera_info').value)
        self.camera_frame = str(self.get_parameter('camera_frame').value)
        self.output_frame = str(self.get_parameter('output_frame').value)
        self.tf_lookup_timeout_s = float(
            self.get_parameter('tf_lookup_timeout_s').value)
        self.tf_stable_required = _as_bool(
            self.get_parameter('tf_stable_required').value)
        self.tf_stable_wait_s = float(
            self.get_parameter('tf_stable_wait_s').value)
        self.tf_stable_sample_count = max(
            2, int(self.get_parameter('tf_stable_sample_count').value))
        self.tf_stable_sample_interval_s = max(
            0.01, float(self.get_parameter('tf_stable_sample_interval_s').value))
        self.tf_stable_translation_tol_m = max(
            0.0, float(self.get_parameter('tf_stable_translation_tol_m').value))
        self.tf_stable_rotation_tol_deg = max(
            0.0, float(self.get_parameter('tf_stable_rotation_tol_deg').value))
        self.tf_stable_source_frame = str(
            self.get_parameter('tf_stable_source_frame').value)

        self.goal_pose_topic = str(self.get_parameter('goal_pose_topic').value)
        self.tag_pose_topic = str(self.get_parameter('tag_pose_topic').value)
        self.target_pose_topic = str(
            self.get_parameter('target_pose_topic').value)
        self.joint_trajectory_topic = str(
            self.get_parameter('joint_trajectory_topic').value)
        self.reach_max = float(self.get_parameter('reach_max_distance').value)
        self.trigger_char = str(self.get_parameter('trigger_key').value).lower()
        self.trigger_topic = str(self.get_parameter('trigger_topic').value)
        self.publish_intermediate_poses = _as_bool(
            self.get_parameter('publish_intermediate_poses').value)
        self.detect_only = _as_bool(self.get_parameter('detect_only').value)
        self.fixed_orientation_enabled = _as_bool(
            self.get_parameter('fixed_orientation_enabled').value)
        fixed_rpy = list(self.get_parameter('fixed_rpy').value)
        self.fixed_rpy = [float(v) for v in fixed_rpy[:3]]

        self.save_debug_images = _as_bool(
            self.get_parameter('save_debug_images').value)
        self.debug_image_dir = str(self.get_parameter('debug_image_dir').value)
        self.save_raw_images = _as_bool(self.get_parameter('save_raw_images').value)
        self.jpeg_quality = int(self.get_parameter('jpeg_quality').value)

        # Read baseline compensation parameters
        self.baseline_enabled = _as_bool(self.get_parameter('baseline_compensation_enabled').value)
        self.baseline = np.array([
            float(self.get_parameter('baseline_x').value),
            float(self.get_parameter('baseline_y').value),
            float(self.get_parameter('baseline_z').value),
        ])
        if self.baseline_enabled:
            self.get_logger().info(
                f'[baseline_compensation] enabled, baseline: ({self.baseline[0]:.4f}, '
                f'{self.baseline[1]:.4f}, {self.baseline[2]:.4f})')

        self._waiting_for_completion = False
        self._completion_timer = None
        self._shoulder_origin = None
        self._shoulder_retry_count = 0
        self._last_warn_time = 0.0
        self._trigger_lock = threading.Lock()
        self._trigger_thread = None
        self._trigger_busy = False
        self._capture = None
        self._capture_thread = None
        self._capture_stop = threading.Event()
        self._capture_lock = threading.Lock()
        self._latest_frame = None
        self._latest_frame_time = 0.0
        self._latest_frame_stamp = None
        self._latest_frame_seq = 0
        self._stream_start_time = 0.0
        self._stream_read_count = 0
        self._stream_ready = False
        self.camera_matrix_live = None
        self.camera_params_live = None
        self.dist_coeffs_live = None
        self._camera_info_logged = False
        self._camera_info_event = threading.Event()
        self._camera_info_fallback_warned = False

        self.detector = Detector(
            families=self.tag_family,
            nthreads=1,
            quad_decimate=float(self.get_parameter('quad_decimate').value),
            refine_edges=1,
        )

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self.goal_pub = self.create_publisher(PoseStamped, self.goal_pose_topic, 10)
        self.tag_pose_pub = self.create_publisher(PoseStamped, self.tag_pose_topic, 10)
        self.target_pose_pub = self.create_publisher(PoseStamped, self.target_pose_topic, 10)
        if self.camera_info_topic:
            self.create_subscription(
                CameraInfo,
                self.camera_info_topic,
                self._camera_info_cb,
                qos_profile_sensor_data)
        self.create_subscription(
            JointTrajectory, self.joint_trajectory_topic, self._traj_cb, 10)
        if self.trigger_topic:
            self.create_subscription(Empty, self.trigger_topic, self._trigger_topic_cb, 10)

        self.fd = None
        self.old_settings = None
        if self.trigger_char:
            self.fd = os.open('/dev/tty', os.O_RDONLY | os.O_NONBLOCK)
            self.old_settings = termios.tcgetattr(self.fd)
            tty.setcbreak(self.fd)

        self._validate_video_device()
        if self.continuous_capture:
            self._capture_thread = threading.Thread(
                target=self._capture_loop, daemon=True)
            self._capture_thread.start()
        if self.fd is not None:
            self.create_timer(0.1, self._tick)
        self.create_timer(0.5, self._retry_shoulder_lookup)

        _, _, camera_params, camera_model_source = self._current_camera_model()
        fx, fy, cx, cy = camera_params
        self.get_logger().info(
            f'[v4l2_apriltag_trigger] Ready — trigger_key={self.trigger_char or "disabled"} '
            f'trigger_topic={self.trigger_topic or "disabled"} '
            f'to capture {self.sample_count} frames from {self.video_device} '
            f'camera_id={self.camera_id} serial={self.expected_serial} '
            f'{self.image_width}x{self.image_height}@{self.fps:g} {self.fourcc} '
            f'warmup={self.warmup_frames} frames/{self.warmup_min_s:.1f}s '
            f'continuous_capture={self.continuous_capture} '
            f'detect_only={self.detect_only} '
            f'publish_intermediate_poses={self.publish_intermediate_poses} '
            f'offset_xyz=[{self.offset_xyz[0]:.3f}, {self.offset_xyz[1]:.3f}, {self.offset_xyz[2]:.3f}] '
            f'fx={fx:.3f} fy={fy:.3f} cx={cx:.3f} cy={cy:.3f} '
            f'camera_model={camera_model_source} '
            f'tf_stable_required={self.tf_stable_required}')

    def _camera_info_cb(self, msg):
        camera_matrix = np.asarray(msg.k, dtype=np.float64).reshape(3, 3)
        self.camera_matrix_live = camera_matrix
        self.camera_params_live = (
            float(camera_matrix[0, 0]),
            float(camera_matrix[1, 1]),
            float(camera_matrix[0, 2]),
            float(camera_matrix[1, 2]),
        )
        dist_coeffs = np.asarray(msg.d, dtype=np.float64) if msg.d else np.zeros(5)
        if dist_coeffs.size == 0:
            dist_coeffs = np.zeros(5, dtype=np.float64)
        self.dist_coeffs_live = dist_coeffs
        self._camera_info_event.set()
        if not self._camera_info_logged:
            fx, fy, cx, cy = self.camera_params_live
            self.get_logger().info(
                f'[v4l2_apriltag_trigger] CameraInfo ready from {self.camera_info_topic}: '
                f'fx={fx:.3f} fy={fy:.3f} cx={cx:.3f} cy={cy:.3f}')
            self._camera_info_logged = True

    def _current_camera_model(self):
        if (self.use_live_camera_info
                and self.camera_matrix_live is not None
                and self.camera_params_live is not None
                and self.dist_coeffs_live is not None):
            return (
                self.camera_matrix_live,
                self.dist_coeffs_live,
                self.camera_params_live,
                'live',
            )
        return (
            self.camera_matrix_fallback,
            self.dist_coeffs_fallback,
            self.camera_params_fallback,
            'fallback',
        )

    def _validate_video_device(self):
        selected = self._select_video_device()
        if selected is None:
            self.get_logger().error(
                f'[v4l2_apriltag_trigger] no usable video_device found '
                f'(configured={self.configured_video_device}, '
                f'serial={self.expected_serial or "any"}, '
                f'usb_interface={self.expected_usb_interface_num or "any"}); '
                f'will retry on capture')
            return False

        self.video_device = selected
        self.video_realpath = os.path.realpath(self.video_device)
        properties = self._read_udev_properties(self.video_realpath)
        serial = properties.get('ID_SERIAL_SHORT', '')
        interface_num = properties.get('ID_USB_INTERFACE_NUM', '')

        if self.expected_serial and serial and serial != self.expected_serial:
            raise RuntimeError(
                f'video_device serial mismatch: expected {self.expected_serial}, got {serial}')
        if (self.expected_usb_interface_num and interface_num
                and interface_num != self.expected_usb_interface_num):
            raise RuntimeError(
                f'video_device USB interface mismatch: expected '
                f'{self.expected_usb_interface_num}, got {interface_num}')

        self.get_logger().info(
            f'[v4l2_apriltag_trigger] video_device={self.video_device} '
            f'realpath={self.video_realpath} serial={serial or "unknown"} '
            f'usb_interface={interface_num or "unknown"}')
        return True

    def _select_video_device(self):
        candidate_paths = []
        configured = (self.configured_video_device or '').strip()
        if configured and configured.lower() != 'auto':
            candidate_paths.append(configured)
        candidate_paths.extend(sorted(glob.glob('/dev/v4l/by-path/*video-index0')))
        candidate_paths.extend(sorted(glob.glob('/dev/v4l/by-id/*video-index0')))
        candidate_paths.extend(sorted(glob.glob('/dev/video*')))

        seen_realpaths = set()
        for path in candidate_paths:
            if not path or not os.path.exists(path):
                continue
            realpath = os.path.realpath(path)
            if realpath in seen_realpaths:
                continue
            seen_realpaths.add(realpath)
            ok, reason = self._video_device_matches(realpath)
            if not ok:
                self.get_logger().debug(
                    f'[v4l2_apriltag_trigger] skipping {path}: {reason}')
                continue
            if path != self.video_device:
                self.get_logger().warn(
                    f'[v4l2_apriltag_trigger] auto-selected video_device {path} '
                    f'(realpath={realpath})')
            return path
        return None

    def _video_device_matches(self, device_path):
        properties = self._read_udev_properties(device_path)
        serial = properties.get('ID_SERIAL_SHORT', '')
        interface_num = properties.get('ID_USB_INTERFACE_NUM', '')

        if self.expected_serial and serial != self.expected_serial:
            return False, f'serial expected {self.expected_serial}, got {serial or "unknown"}'
        if self.expected_usb_interface_num and interface_num != self.expected_usb_interface_num:
            return False, (
                f'USB interface expected {self.expected_usb_interface_num}, '
                f'got {interface_num or "unknown"}')
        if not self._video_device_supports_format(device_path):
            return False, (
                f'missing requested format {self.image_width}x{self.image_height} '
                f'{self.fourcc}')
        return True, 'ok'

    def _video_device_supports_format(self, device_path):
        try:
            result = subprocess.run(
                ['v4l2-ctl', '--list-formats-ext', '-d', device_path],
                check=False,
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
                text=True,
                timeout=2.0,
            )
        except Exception as ex:
            self.get_logger().warn(f'v4l2-ctl query failed for {device_path}: {ex}')
            return True

        output = result.stdout
        if result.returncode != 0:
            return False
        requested_size = f'Size: Discrete {self.image_width}x{self.image_height}'
        if self.fourcc and f"'{self.fourcc}'" not in output:
            return False
        if requested_size not in output:
            return False
        return True

    def _read_udev_properties(self, device_path):
        try:
            result = subprocess.run(
                ['udevadm', 'info', '--query=property', '--name', device_path],
                check=False,
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
                text=True,
                timeout=2.0,
            )
        except FileNotFoundError:
            return self._read_sysfs_video_properties(device_path)
        except Exception as ex:
            self.get_logger().warn(f'udevadm query failed: {ex}')
            return self._read_sysfs_video_properties(device_path)

        properties = {}
        for line in result.stdout.splitlines():
            if '=' not in line:
                continue
            key, value = line.split('=', 1)
            properties[key] = value
        sysfs_properties = self._read_sysfs_video_properties(device_path)
        for key in ('ID_SERIAL_SHORT', 'ID_USB_INTERFACE_NUM'):
            if not properties.get(key) and sysfs_properties.get(key):
                properties[key] = sysfs_properties[key]
        return properties

    def _read_sysfs_video_properties(self, device_path):
        video_name = os.path.basename(os.path.realpath(device_path))
        sys_device = os.path.realpath(
            os.path.join('/sys/class/video4linux', video_name, 'device'))
        properties = {}

        interface_num = self._read_sysfs_value(
            os.path.join(sys_device, 'bInterfaceNumber'))
        if interface_num:
            properties['ID_USB_INTERFACE_NUM'] = interface_num

        path = sys_device
        while path and path != '/':
            serial = self._read_sysfs_value(os.path.join(path, 'serial'))
            if serial:
                properties['ID_SERIAL_SHORT'] = serial
                break
            parent = os.path.dirname(path)
            if parent == path:
                break
            path = parent
        return properties

    def _read_sysfs_value(self, path):
        try:
            with open(path, 'r', encoding='utf-8') as f:
                return f.read().strip()
        except OSError:
            return ''

    def _tick(self):
        if self.fd is None:
            return
        if not select.select([self.fd], [], [], 0.0)[0]:
            return
        try:
            ch = os.read(self.fd, 1).decode('utf-8', errors='ignore')
        except Exception:
            return
        if ch and ch.lower() == self.trigger_char:
            self._on_trigger()

    def _trigger_topic_cb(self, _msg):
        self._on_trigger()

    def _traj_cb(self, msg):
        if not msg.points:
            return
        t = msg.points[-1].time_from_start
        duration_s = float(t.sec) + float(t.nanosec) * 1e-9 + 1.0
        if self._completion_timer is not None:
            self.destroy_timer(self._completion_timer)
            self._completion_timer = None

        def _on_completion():
            self._waiting_for_completion = False
            if self._completion_timer is not None:
                self.destroy_timer(self._completion_timer)
                self._completion_timer = None

        self._completion_timer = self.create_timer(duration_s, _on_completion)

    def _retry_shoulder_lookup(self):
        if self._shoulder_origin is not None:
            return
        self._shoulder_retry_count += 1
        try:
            transform = self.tf_buffer.lookup_transform(
                self.output_frame,
                'right_shoulder_pitch_link',
                rclpy.time.Time(),
                timeout=Duration(seconds=0.5))
            self._shoulder_origin = (
                transform.transform.translation.x,
                transform.transform.translation.y,
                transform.transform.translation.z,
            )
            self.get_logger().info(
                f'[v4l2_apriltag_trigger] Cached shoulder origin: '
                f'({self._shoulder_origin[0]:.3f}, '
                f'{self._shoulder_origin[1]:.3f}, '
                f'{self._shoulder_origin[2]:.3f})')
        except Exception as ex:
            if self._shoulder_retry_count == 1 or self._shoulder_retry_count % 10 == 0:
                self.get_logger().warn(
                    f'[v4l2_apriltag_trigger] Shoulder TF lookup failed '
                    f'({self._shoulder_retry_count}): {ex}')

    def _on_trigger(self):
        with self._trigger_lock:
            if self._trigger_busy:
                self.get_logger().warn(
                    '[v4l2_apriltag_trigger] trigger already running, ignoring')
                return
            self._trigger_busy = True
        self._trigger_thread = threading.Thread(
            target=self._run_trigger, daemon=True)
        self._trigger_thread.start()

    def _run_trigger(self):
        try:
            self._run_trigger_once()
        finally:
            with self._trigger_lock:
                self._trigger_busy = False

    def _run_trigger_once(self):
        if not self.detect_only and self._waiting_for_completion:
            self.get_logger().warn(
                '[v4l2_apriltag_trigger] previous goal still in flight, ignoring trigger')
            return
        if not self.detect_only and self._shoulder_origin is None:
            self._retry_shoulder_lookup()
            if self._shoulder_origin is None:
                self.get_logger().warn(
                    '[v4l2_apriltag_trigger] shoulder origin not yet available')
                return
        if self.tf_stable_required and not self._wait_for_stable_tf():
            self.get_logger().warn(
                '[v4l2_apriltag_trigger] TF did not stabilize before capture')
            return

        self.get_logger().info(
            f'[v4l2_apriltag_trigger] {self.trigger_char.upper()} pressed — capturing')
        frames = self._capture_frames()
        if not frames:
            self.get_logger().warn('[v4l2_apriltag_trigger] no frames captured')
            return

        self._prepare_debug_dir()
        accepted = []
        for index, frame_entry in enumerate(frames):
            frame, capture_stamp = frame_entry
            result = self._process_frame(frame, index, capture_stamp)
            if result is not None:
                accepted.append(result)

        if not accepted:
            self.get_logger().warn(
                '[v4l2_apriltag_trigger] no accepted AprilTag detections')
            return

        xs = [item['target_torso'].pose.position.x for item in accepted]
        ys = [item['target_torso'].pose.position.y for item in accepted]
        zs = [item['target_torso'].pose.position.z for item in accepted]
        avg_x = sum(xs) / len(xs)
        avg_y = sum(ys) / len(ys)
        avg_z = sum(zs) / len(zs)
        tag_xs = [item['tag_torso'].pose.position.x for item in accepted]
        tag_ys = [item['tag_torso'].pose.position.y for item in accepted]
        tag_zs = [item['tag_torso'].pose.position.z for item in accepted]
        tag_avg_x = sum(tag_xs) / len(tag_xs)
        tag_avg_y = sum(tag_ys) / len(tag_ys)
        tag_avg_z = sum(tag_zs) / len(tag_zs)
        best = max(accepted, key=lambda item: item['decision_margin'])
        final_tag_pose = best['tag_torso']
        final_target_pose = best['target_torso']
        final_stamp = self.get_clock().now().to_msg()
        final_tag_pose.header.stamp = final_stamp
        final_tag_pose.pose.position.x = tag_avg_x
        final_tag_pose.pose.position.y = tag_avg_y
        final_tag_pose.pose.position.z = tag_avg_z
        final_target_pose.header.stamp = final_stamp
        final_target_pose.pose.position.x = avg_x
        final_target_pose.pose.position.y = avg_y
        final_target_pose.pose.position.z = avg_z

        # Apply baseline compensation
        if self.baseline_enabled:
            current_tag = np.array([tag_avg_x, tag_avg_y, tag_avg_z])
            offset_compensation = current_tag - self.baseline
            final_target_pose.pose.position.x -= offset_compensation[0]
            final_target_pose.pose.position.y -= offset_compensation[1]
            final_target_pose.pose.position.z -= offset_compensation[2]
            self.get_logger().info(
                f'[baseline_compensation] offset: ({offset_compensation[0]:.4f}, '
                f'{offset_compensation[1]:.4f}, {offset_compensation[2]:.4f}) '
                f'corrected: ({final_target_pose.pose.position.x:.3f}, '
                f'{final_target_pose.pose.position.y:.3f}, {final_target_pose.pose.position.z:.3f})')

        if not self.publish_intermediate_poses:
            self.tag_pose_pub.publish(final_tag_pose)
            self.target_pose_pub.publish(final_target_pose)
        if self.detect_only:
            _o = best['tag_torso'].pose.orientation
            self.get_logger().info(
                f'[v4l2_apriltag_trigger] detect_only accepted={len(accepted)}/{len(frames)} '
                f'tag=({tag_avg_x:.3f}, {tag_avg_y:.3f}, {tag_avg_z:.3f}) '
                f'tag_quat=({_o.x:.6f},{_o.y:.6f},{_o.z:.6f},{_o.w:.6f}) '
                f'target=({avg_x:.3f}, {avg_y:.3f}, {avg_z:.3f}) '
                f'delta=({avg_x - tag_avg_x:.3f}, {avg_y - tag_avg_y:.3f}, {avg_z - tag_avg_z:.3f}) '
                f'@ {self.output_frame}, best_margin={best["decision_margin"]:.1f}, not publishing {self.goal_pose_topic}')
            return

        sx, sy, sz = self._shoulder_origin
        dist = math.sqrt((avg_x - sx) ** 2 + (avg_y - sy) ** 2 + (avg_z - sz) ** 2)
        goal_x = avg_x
        goal_y = avg_y
        goal_z = avg_z
        if dist >= self.reach_max:
            if dist <= 1e-9 or self.reach_max <= 0.0:
                self.get_logger().warn(
                    f'[v4l2_apriltag_trigger] invalid reach projection '
                    f'(dist={dist:.6f}, reach_max={self.reach_max}), not publishing')
                return
            scale = self.reach_max / dist
            goal_x = sx + (avg_x - sx) * scale
            goal_y = sy + (avg_y - sy) * scale
            goal_z = sz + (avg_z - sz) * scale
            self.get_logger().warn(
                f'[v4l2_apriltag_trigger] reach exceeds {dist:.3f} m '
                f'> {self.reach_max} m; publishing nearest reach-limit target '
                f'({goal_x:.3f}, {goal_y:.3f}, {goal_z:.3f}) for planner fallback')

        goal = PoseStamped()
        goal.header.frame_id = self.output_frame
        goal.header.stamp = self.get_clock().now().to_msg()
        goal.pose.position.x = goal_x
        goal.pose.position.y = goal_y
        goal.pose.position.z = goal_z
        if self.fixed_orientation_enabled:
            qx, qy, qz, qw = self._fixed_orientation_quaternion()
            goal.pose.orientation.x = qx
            goal.pose.orientation.y = qy
            goal.pose.orientation.z = qz
            goal.pose.orientation.w = qw
        else:
            goal.pose.orientation = best['target_torso'].pose.orientation

        self.goal_pub.publish(goal)
        self._waiting_for_completion = True
        self.get_logger().info(
            f'[v4l2_apriltag_trigger] accepted={len(accepted)}/{len(frames)} '
            f'tag=({tag_avg_x:.3f}, {tag_avg_y:.3f}, {tag_avg_z:.3f}) '
            f'target=({goal_x:.3f}, {goal_y:.3f}, {goal_z:.3f}) '
            f'delta=({avg_x - tag_avg_x:.3f}, {avg_y - tag_avg_y:.3f}, {avg_z - tag_avg_z:.3f}) '
            f'@ {self.output_frame}, |target-shoulder|={dist:.3f} m, publishing {self.goal_pose_topic}')

    def _lookup_stable_transform_sample(self):
        transform = self.tf_buffer.lookup_transform(
            self.output_frame,
            self.tf_stable_source_frame,
            Time(),
            timeout=Duration(seconds=self.tf_lookup_timeout_s))
        translation = np.array([
            transform.transform.translation.x,
            transform.transform.translation.y,
            transform.transform.translation.z,
        ], dtype=np.float64)
        quaternion = np.array([
            transform.transform.rotation.x,
            transform.transform.rotation.y,
            transform.transform.rotation.z,
            transform.transform.rotation.w,
        ], dtype=np.float64)
        norm = np.linalg.norm(quaternion)
        if norm > 1e-9:
            quaternion = quaternion / norm
        return translation, quaternion

    def _wait_for_stable_tf(self):
        deadline = time.monotonic() + max(0.0, self.tf_stable_wait_s)
        samples = []
        while time.monotonic() < deadline:
            try:
                sample = self._lookup_stable_transform_sample()
            except Exception as ex:
                self._warn_throttled(
                    f'[v4l2_apriltag_trigger] TF stability lookup failed: {ex}')
                time.sleep(self.tf_stable_sample_interval_s)
                continue
            samples.append(sample)
            if len(samples) > self.tf_stable_sample_count:
                samples.pop(0)
            if len(samples) >= self.tf_stable_sample_count:
                if self._samples_are_stable(samples):
                    return True
            time.sleep(self.tf_stable_sample_interval_s)
        return False

    def _samples_are_stable(self, samples):
        translations = np.asarray([sample[0] for sample in samples], dtype=np.float64)
        translation_span = np.max(
            np.linalg.norm(translations - translations[0], axis=1))
        max_angle_deg = 0.0
        base_quaternion = samples[0][1]
        for _, quaternion in samples[1:]:
            dot = float(np.clip(np.abs(np.dot(base_quaternion, quaternion)), 0.0, 1.0))
            angle_rad = 2.0 * math.acos(dot)
            max_angle_deg = max(max_angle_deg, math.degrees(angle_rad))
        return (translation_span <= self.tf_stable_translation_tol_m
                and max_angle_deg <= self.tf_stable_rotation_tol_deg)

    def _capture_frames(self):
        if not self.continuous_capture:
            return self._capture_frames_on_demand()
        return self._capture_frames_from_stream()

    def _capture_frames_from_stream(self):
        frames = []
        last_seq = 0
        deadline = time.monotonic() + max(
            1.0,
            max(1, self.sample_count) / max(self.fps, 1.0)
            + max(0.0, self.sample_interval_s) * max(0, self.sample_count - 1)
            + 1.0)

        while len(frames) < max(1, self.sample_count) and time.monotonic() < deadline:
            with self._capture_lock:
                frame = self._latest_frame
                frame_time = self._latest_frame_time
                frame_stamp = self._latest_frame_stamp
                seq = self._latest_frame_seq
                ready = self._stream_ready

            if not ready:
                self.get_logger().warn(
                    '[v4l2_apriltag_trigger] camera stream not warmed up yet')
                return []
            if frame is None or seq == last_seq:
                time.sleep(0.01)
                continue
            if time.monotonic() - frame_time > 1.0:
                self.get_logger().warn(
                    '[v4l2_apriltag_trigger] latest camera frame is stale')
                return []

            capture_stamp = frame_stamp if frame_stamp is not None else self.get_clock().now().to_msg()
            frames.append((frame.copy(), capture_stamp))
            last_seq = seq
            if self.sample_interval_s > 0.0:
                time.sleep(self.sample_interval_s)

        if len(frames) < max(1, self.sample_count):
            self.get_logger().warn(
                f'[v4l2_apriltag_trigger] captured only '
                f'{len(frames)}/{max(1, self.sample_count)} warmed frames')
        return frames

    def _capture_frames_on_demand(self):
        for attempt in range(2):
            cap = self._open_capture()
            if cap is None:
                return []

            try:
                frames = []
                warmup_target = max(0, self.warmup_frames)
                warmup_start = time.monotonic()
                warmup_min_deadline = warmup_start + max(0.0, self.warmup_min_s)
                warmup_max_deadline = warmup_start + max(
                    1.0,
                    max(warmup_target, 1) / max(self.fps, 1.0)
                    + max(0.0, self.warmup_min_s)
                    + 1.0)
                warmup_reads = 0
                while ((warmup_reads < warmup_target
                        or time.monotonic() < warmup_min_deadline)
                       and time.monotonic() < warmup_max_deadline):
                    ret, _ = cap.read()
                    if ret:
                        warmup_reads += 1
                    else:
                        self.get_logger().warn(
                            '[v4l2_apriltag_trigger] failed to read one warmup frame')
                        time.sleep(0.05)
                if warmup_reads < warmup_target:
                    self.get_logger().warn(
                        f'[v4l2_apriltag_trigger] warmup only read '
                        f'{warmup_reads}/{warmup_target} frames')
                for _ in range(max(1, self.sample_count)):
                    ret, frame = cap.read()
                    if ret and frame is not None:
                        frames.append((frame.copy(), self.get_clock().now().to_msg()))
                    else:
                        self.get_logger().warn(
                            '[v4l2_apriltag_trigger] failed to read one frame')
                    if self.sample_interval_s > 0.0:
                        time.sleep(self.sample_interval_s)
                if frames or attempt > 0:
                    return frames
                self.get_logger().warn(
                    '[v4l2_apriltag_trigger] no frames from current video device; rescanning and retrying once')
                self.video_device = self.configured_video_device
                time.sleep(0.2)
            finally:
                cap.release()
        return []

    def _open_capture(self):
        if not self.video_device or self.video_device.lower() == 'auto' or not os.path.exists(self.video_device):
            if not self._validate_video_device():
                return None

        cap = cv2.VideoCapture(self.video_device, cv2.CAP_V4L2)
        if not cap.isOpened():
            failed_device = self.video_device
            cap.release()
            self.get_logger().warn(
                f'[v4l2_apriltag_trigger] failed to open {failed_device}; rescanning video devices')
            self.video_device = self.configured_video_device
            if self._validate_video_device():
                time.sleep(0.2)
                cap = cv2.VideoCapture(self.video_device, cv2.CAP_V4L2)
            if not cap.isOpened():
                self.get_logger().error(
                    f'[v4l2_apriltag_trigger] failed to open {self.video_device}')
                return None

        cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, float(self.image_width))
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, float(self.image_height))
        cap.set(cv2.CAP_PROP_FPS, float(self.fps))
        if len(self.fourcc) == 4:
            cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*self.fourcc))

        actual_w = cap.get(cv2.CAP_PROP_FRAME_WIDTH)
        actual_h = cap.get(cv2.CAP_PROP_FRAME_HEIGHT)
        actual_fps = cap.get(cv2.CAP_PROP_FPS)
        actual_fourcc_int = int(cap.get(cv2.CAP_PROP_FOURCC))
        actual_fourcc = actual_fourcc_int.to_bytes(4, 'little', signed=False).decode(
            'latin-1', errors='replace')
        self.get_logger().info(
            f'[v4l2_apriltag_trigger] capture opened: '
            f'{actual_w:.0f}x{actual_h:.0f}@{actual_fps:.1f} {actual_fourcc}')
        return cap

    def _capture_loop(self):
        while not self._capture_stop.is_set():
            cap = self._open_capture()
            if cap is None:
                time.sleep(1.0)
                continue

            self._capture = cap
            self._stream_start_time = time.monotonic()
            self._stream_read_count = 0
            with self._capture_lock:
                self._latest_frame = None
                self._latest_frame_time = 0.0
                self._latest_frame_seq = 0
                self._stream_ready = False

            try:
                while not self._capture_stop.is_set():
                    ret, frame = cap.read()
                    if not ret or frame is None:
                        self._warn_throttled(
                            '[v4l2_apriltag_trigger] failed to read camera frame')
                        time.sleep(0.05)
                        continue

                    now = time.monotonic()
                    stamp = self.get_clock().now().to_msg()
                    self._stream_read_count += 1
                    ready = (
                        self._stream_read_count >= max(0, self.warmup_frames)
                        and now - self._stream_start_time >= max(0.0, self.warmup_min_s)
                    )
                    with self._capture_lock:
                        self._latest_frame = frame.copy()
                        self._latest_frame_time = now
                        self._latest_frame_stamp = stamp
                        self._latest_frame_seq += 1
                        if ready and not self._stream_ready:
                            self.get_logger().info(
                                f'[v4l2_apriltag_trigger] camera stream warmed up '
                                f'({self._stream_read_count} frames, '
                                f'{now - self._stream_start_time:.1f}s)')
                        self._stream_ready = ready
            finally:
                cap.release()
                self._capture = None

    def _process_frame(self, frame, index, capture_stamp):
        if frame.ndim == 2:
            gray = frame
            bgr = cv2.cvtColor(frame, cv2.COLOR_GRAY2BGR)
        else:
            bgr = frame
            gray = cv2.cvtColor(bgr, cv2.COLOR_BGR2GRAY)

        raw = bgr.copy()
        h, w = gray.shape[:2]
        scale = self.detect_scale if self.detect_scale > 0.0 else 1.0
        if scale != 1.0:
            gray_detect = cv2.resize(
                gray, (int(w * scale), int(h * scale)), interpolation=cv2.INTER_AREA)
        else:
            gray_detect = gray

        camera_matrix, dist_coeffs, camera_params, camera_model_source = self._current_camera_model()
        if (self.use_live_camera_info and camera_model_source == 'fallback'
                and not self._camera_info_fallback_warned):
            self.get_logger().warn(
                f'[v4l2_apriltag_trigger] no CameraInfo on {self.camera_info_topic}; '
                f'using fallback camera_matrix from parameters')
            self._camera_info_fallback_warned = True
        fx, fy, cx, cy = camera_params
        cam_params = (fx * scale, fy * scale, cx * scale, cy * scale)
        detections = self.detector.detect(
            gray_detect,
            estimate_tag_pose=True,
            camera_params=cam_params,
            tag_size=self.tag_size,
        )

        if scale != 1.0:
            for detection in detections:
                detection.corners = detection.corners / scale
                detection.center = detection.center / scale

        display = bgr.copy()
        best_margin = 0.0
        best_result = None
        for detection in detections:
            best_margin = max(best_margin, float(detection.decision_margin))
            accepted = (
                detection.tag_id == self.target_tag_id
                and detection.hamming == 0
                and detection.decision_margin >= self.decision_margin_min
            )
            self._draw_detection(display, detection, accepted, camera_matrix, dist_coeffs)
            if not accepted:
                continue

            result = self._make_poses(detection, capture_stamp)
            if result is None:
                continue
            if best_result is None or detection.decision_margin > best_result['decision_margin']:
                best_result = result

        cv2.putText(
            display,
            f'frame={index} detections={len(detections)} best_margin={best_margin:.1f}',
            (10, 24),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.55,
            (255, 255, 255),
            2,
            cv2.LINE_AA,
        )
        self._save_debug_image(index, display, raw)
        return best_result

    def _make_poses(self, detection, capture_stamp):
        tvec = np.asarray(detection.pose_t, dtype=np.float64).reshape(3)
        quat = R.from_matrix(detection.pose_R).as_quat()

        pose_cam = PoseStamped()
        pose_cam.header.stamp = capture_stamp
        pose_cam.header.frame_id = self.camera_frame
        pose_cam.pose.position.x = float(tvec[0])
        pose_cam.pose.position.y = float(tvec[1])
        pose_cam.pose.position.z = float(tvec[2])
        pose_cam.pose.orientation.x = float(quat[0])
        pose_cam.pose.orientation.y = float(quat[1])
        pose_cam.pose.orientation.z = float(quat[2])
        pose_cam.pose.orientation.w = float(quat[3])

        t_cam_tag = np.eye(4, dtype=np.float64)
        t_cam_tag[:3, :3] = detection.pose_R
        t_cam_tag[:3, 3] = tvec
        t_tag_target = np.eye(4, dtype=np.float64)
        t_tag_target[:3, 3] = self.offset_xyz
        t_cam_target = t_cam_tag @ t_tag_target
        target_quat = R.from_matrix(t_cam_target[:3, :3]).as_quat()

        target_pose_cam = PoseStamped()
        target_pose_cam.header.stamp = capture_stamp
        target_pose_cam.header.frame_id = self.camera_frame
        target_pose_cam.pose.position.x = float(t_cam_target[0, 3])
        target_pose_cam.pose.position.y = float(t_cam_target[1, 3])
        target_pose_cam.pose.position.z = float(t_cam_target[2, 3])
        target_pose_cam.pose.orientation.x = float(target_quat[0])
        target_pose_cam.pose.orientation.y = float(target_quat[1])
        target_pose_cam.pose.orientation.z = float(target_quat[2])
        target_pose_cam.pose.orientation.w = float(target_quat[3])

        timeout = Duration(seconds=self.tf_lookup_timeout_s)
        try:
            pose_torso = self.tf_buffer.transform(
                pose_cam, self.output_frame, timeout=timeout)
            target_torso = self.tf_buffer.transform(
                target_pose_cam, self.output_frame, timeout=timeout)
        except tf2_ros.TransformException as ex:
            self._warn_throttled(f'[v4l2_apriltag_trigger] TF transform failed: {ex}')
            return None

        pose_torso.header.stamp = self.get_clock().now().to_msg()
        target_torso.header.stamp = pose_torso.header.stamp
        if self.publish_intermediate_poses:
            self.tag_pose_pub.publish(pose_torso)
            self.target_pose_pub.publish(target_torso)
        return {
            'tag_torso': pose_torso,
            'target_torso': target_torso,
            'decision_margin': float(detection.decision_margin),
        }

    def _draw_detection(self, image, detection, accepted, camera_matrix, dist_coeffs):
        color = (0, 255, 0) if accepted else (0, 0, 255)
        corners = detection.corners.astype(int)
        cv2.polylines(image, [corners], isClosed=True, color=color, thickness=2)
        label = f'id={detection.tag_id} margin={float(detection.decision_margin):.1f}'
        cv2.putText(
            image,
            label,
            tuple(corners[0] + np.array([0, -8])),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.5,
            color,
            1,
            cv2.LINE_AA,
        )
        if not accepted:
            return
        obj_pts = np.array(
            [[0.0, 0.0, 0.0],
             [0.03, 0.0, 0.0],
             [0.0, 0.03, 0.0],
             [0.0, 0.0, 0.03]], dtype=np.float64)
        rvec = cv2.Rodrigues(detection.pose_R)[0]
        tvec = np.asarray(detection.pose_t, dtype=np.float64).reshape(3, 1)
        img_pts, _ = cv2.projectPoints(
            obj_pts, rvec, tvec, camera_matrix, dist_coeffs)
        img_pts = img_pts.reshape(-1, 2).astype(int)
        o, x, y, z = img_pts
        cv2.line(image, tuple(o), tuple(x), (0, 0, 255), 2)
        cv2.line(image, tuple(o), tuple(y), (0, 255, 0), 2)
        cv2.line(image, tuple(o), tuple(z), (255, 0, 0), 2)

    def _prepare_debug_dir(self):
        if not self.save_debug_images:
            return
        try:
            os.makedirs(self.debug_image_dir, exist_ok=True)
            for pattern in ('detected_*.jpg', 'raw_*.jpg'):
                for path in glob.glob(os.path.join(self.debug_image_dir, pattern)):
                    os.remove(path)
        except Exception as ex:
            self.get_logger().warn(
                f'[v4l2_apriltag_trigger] failed to prepare debug dir: {ex}')

    def _save_debug_image(self, index, display, raw):
        if not self.save_debug_images:
            return
        params = [cv2.IMWRITE_JPEG_QUALITY, self.jpeg_quality]
        try:
            detected_path = os.path.join(self.debug_image_dir, f'detected_{index:02d}.jpg')
            cv2.imwrite(detected_path, display, params)
            if self.save_raw_images:
                raw_path = os.path.join(self.debug_image_dir, f'raw_{index:02d}.jpg')
                cv2.imwrite(raw_path, raw, params)
        except Exception as ex:
            self.get_logger().warn(
                f'[v4l2_apriltag_trigger] failed to save debug image: {ex}')

    def _fixed_orientation_quaternion(self):
        roll, pitch, yaw = self.fixed_rpy
        cy = math.cos(yaw * 0.5)
        sy = math.sin(yaw * 0.5)
        cp = math.cos(pitch * 0.5)
        sp = math.sin(pitch * 0.5)
        cr = math.cos(roll * 0.5)
        sr = math.sin(roll * 0.5)
        return (
            sr * cp * cy - cr * sp * sy,
            cr * sp * cy + sr * cp * sy,
            cr * cp * sy - sr * sp * cy,
            cr * cp * cy + sr * sp * sy,
        )

    def _warn_throttled(self, msg, period_s=2.0):
        now = time.monotonic()
        if now - self._last_warn_time >= period_s:
            self.get_logger().warn(msg)
            self._last_warn_time = now

    def destroy_node(self):
        if self.fd is not None and self.old_settings is not None:
            try:
                termios.tcsetattr(self.fd, termios.TCSADRAIN, self.old_settings)
            except Exception:
                pass
        if self.fd is not None:
            try:
                os.close(self.fd)
            except Exception:
                pass
        self._capture_stop.set()
        if self._capture_thread is not None:
            self._capture_thread.join(timeout=1.0)
            self._capture_thread = None
        if self._completion_timer is not None:
            try:
                self.destroy_timer(self._completion_timer)
            except Exception:
                pass
            self._completion_timer = None
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = V4L2AprilTagTrigger()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, SystemExit):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
