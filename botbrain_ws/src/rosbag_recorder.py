#!/usr/bin/python3
"""ROS2 bag recorder: records all published topics to MCAP format using rosbag2_py.

Features:
- Auto-discovers all active topics
- Supports MCAP format (modern, compact, cross-platform)
- Configurable recording duration and output path
- Filter topics by name pattern (include/exclude)
- Progress reporting and statistics
- TF transform recording
"""

import argparse
import os
import signal
import sys
import time
from datetime import datetime
from threading import Thread

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSHistoryPolicy, QoSReliabilityPolicy

try:
    from rosbag2_py import SequentialWriter
    from rosbag2_py import StorageOptions
    from rosbag2_py import ConverterOptions
    HAS_ROSBAG2_PY = True
except ImportError:
    HAS_ROSBAG2_PY = False


class RosbagRecorderNode(Node):
    def __init__(self, output_dir, duration=None, topics=None, exclude_topics=None, queue_size=100):
        super().__init__('rosbag_recorder')
        self.output_dir = output_dir
        self.duration = duration
        self.topics = topics
        self.exclude_topics = exclude_topics or []
        self.queue_size = queue_size
        self.recording = False
        self.start_time = None
        self.topic_subscribers = {}
        self.message_counts = {}
        self.total_messages = 0
        self.writer = None
        self._sigint_received = False

        os.makedirs(output_dir, exist_ok=True)
        self.bag_name = f"recording_{datetime.now().strftime('%Y%m%d_%H%M%S')}"
        self.bag_path = os.path.join(output_dir, self.bag_name)

        self.discovery_timer = self.create_timer(2.0, self._discover_topics)
        self.stats_timer = self.create_timer(5.0, self._print_stats)

        if HAS_ROSBAG2_PY:
            self._init_writer()

    def _init_writer(self):
        storage_options = StorageOptions(
            uri=self.bag_path,
            storage_id='mcap'
        )
        converter_options = ConverterOptions(
            input_serialization_format='cdr',
            output_serialization_format='cdr'
        )
        self.writer = SequentialWriter()
        self.writer.open(storage_options, converter_options)

    def _should_record_topic(self, topic_name):
        if topic_name in self.exclude_topics:
            return False
        if self.topics is not None and len(self.topics) > 0:
            return topic_name in self.topics
        return True

    def _discover_topics(self):
        if not self.recording:
            return

        try:
            topic_names_and_types = self.get_topic_names_and_types()
        except Exception as e:
            self.get_logger().warn(f'Failed to get topic list: {e}')
            return

        for topic_name, types in topic_names_and_types:
            if topic_name in self.topic_subscribers:
                continue
            if not self._should_record_topic(topic_name):
                continue

            msg_type = types[0] if types else ''
            self.get_logger().info(f'Discovered topic: {topic_name} ({msg_type})')

            try:
                qos = QoSProfile(
                    history=QoSHistoryPolicy.KEEP_LAST,
                    depth=self.queue_size,
                    reliability=QoSReliabilityPolicy.BEST_EFFORT
                )

                msg_class = self._get_message_class(msg_type)

                if HAS_ROSBAG2_PY and self.writer:
                    self.writer.create_topic({
                        'name': topic_name,
                        'type': msg_type,
                        'serialization_format': 'cdr'
                    })

                sub = self.create_subscription(
                    msg_class,
                    topic_name,
                    self._create_callback(topic_name),
                    qos
                )
                self.topic_subscribers[topic_name] = {
                    'subscriber': sub,
                    'type': msg_type,
                    'count': 0
                }
                self.message_counts[topic_name] = 0
            except Exception as e:
                self.get_logger().warn(f'Failed to subscribe to {topic_name}: {e}')

    def _get_message_class(self, msg_type_str):
        try:
            parts = msg_type_str.split('/')
            if len(parts) >= 3:
                pkg = parts[0]
                msg_module = parts[1]
                msg_class_name = parts[2]

                module_path = f'{pkg}.{msg_module}.msg'
                module = __import__(module_path, fromlist=[msg_class_name])
                return getattr(module, msg_class_name)
        except Exception as e:
            self.get_logger().warn(f'Failed to load message type {msg_type_str}: {e}')

        from std_msgs.msg import String
        return String

    def _create_callback(self, topic_name):
        def callback(msg):
            if not self.recording:
                return
            self.message_counts[topic_name] += 1
            self.total_messages += 1

            if HAS_ROSBAG2_PY and self.writer:
                try:
                    import rclpy.serialization
                    serialized_msg = rclpy.serialization.serialize_message(msg)
                    self.writer.write(
                        topic_name,
                        serialized_msg,
                        self.get_clock().now().nanoseconds
                    )
                except Exception as e:
                    self.get_logger().warn(f'Failed to write message for {topic_name}: {e}')

        return callback

    def start_recording(self):
        self.get_logger().info('=' * 60)
        self.get_logger().info('Starting ROS bag recording...')
        self.get_logger().info(f'Output directory: {self.output_dir}')
        self.get_logger().info(f'Bag name: {self.bag_name}')
        self.get_logger().info(f'Output path: {self.bag_path}')
        self.get_logger().info(f'MCAP format: {"Yes" if HAS_ROSBAG2_PY else "No (using ros2 bag CLI)"}')

        if self.topics:
            self.get_logger().info(f'Recording topics: {self.topics}')
        else:
            self.get_logger().info('Recording all topics')

        if self.exclude_topics:
            self.get_logger().info(f'Excluding topics: {self.exclude_topics}')

        if self.duration:
            self.get_logger().info(f'Recording will stop after {self.duration} seconds')
        else:
            self.get_logger().info('Recording until interrupted (Ctrl+C)')
        self.get_logger().info('=' * 60)

        self.start_time = time.time()
        self.recording = True

        time.sleep(1.0)
        self._discover_topics()
        self._print_stats()

    def stop_recording(self):
        if not self.recording:
            return

        self.recording = False
        elapsed = time.time() - self.start_time

        self.get_logger().info('=' * 60)
        self.get_logger().info('Recording stopped')
        self.get_logger().info(f'Total duration: {elapsed:.2f} seconds')
        self.get_logger().info(f'Total messages recorded: {self.total_messages}')
        self.get_logger().info(f'Total topics recorded: {len(self.topic_subscribers)}')
        self.get_logger().info('=' * 60)

        self.get_logger().info('Per-topic message counts:')
        for topic, count in sorted(self.message_counts.items(), key=lambda x: -x[1]):
            self.get_logger().info(f'  {topic}: {count} messages')

        if HAS_ROSBAG2_PY and self.writer:
            try:
                self.writer.close()
            except Exception as e:
                self.get_logger().warn(f'Error closing writer: {e}')

        self.get_logger().info(f'Recording saved to: {self.bag_path}')

    def _check_duration(self):
        if self.duration and self.recording:
            elapsed = time.time() - self.start_time
            if elapsed >= self.duration:
                self.get_logger().info(f'Duration {self.duration}s reached, stopping...')
                self.stop_recording()
                rclpy.shutdown()
                sys.exit(0)

    def _print_stats(self):
        if not self.recording:
            return

        elapsed = time.time() - self.start_time
        self.get_logger().info(
            f'Recording: {elapsed:.1f}s | '
            f'Topics: {len(self.topic_subscribers)} | '
            f'Messages: {self.total_messages}'
        )


def cli_recorder(output_dir, duration, topics, exclude):
    os.makedirs(output_dir, exist_ok=True)
    bag_name = f"recording_{datetime.now().strftime('%Y%m%d_%H%M%S')}"
    bag_path = os.path.join(output_dir, bag_name)

    print(f'[ROS2 Bag Recorder] Output directory: {output_dir}')
    print(f'[ROS2 Bag Recorder] Bag name: {bag_name}')
    print(f'[ROS2 Bag Recorder] Output path: {bag_path}')

    import subprocess
    cmd_parts = ['ros2', 'bag', 'record', '-o', bag_path, '--storage', 'mcap']

    if topics:
        cmd_parts.extend(topics)
    else:
        cmd_parts.append('--all')

    if exclude:
        for topic in exclude:
            cmd_parts.extend(['--exclude', topic])

    print(f'[ROS2 Bag Recorder] Command: {" ".join(cmd_parts)}')
    print('[ROS2 Bag Recorder] Starting recording...')
    print('[ROS2 Bag Recorder] Press Ctrl+C to stop')

    proc = subprocess.Popen(cmd_parts)

    def signal_handler(sig, frame):
        print('\n[ROS2 Bag Recorder] Stopping recording...')
        proc.terminate()
        try:
            proc.wait(timeout=5)
        except subprocess.TimeoutExpired:
            proc.kill()
        print('[ROS2 Bag Recorder] Recording stopped')
        sys.exit(0)

    signal.signal(signal.SIGINT, signal_handler)

    if duration:
        print(f'[ROS2 Bag Recorder] Recording will stop after {duration} seconds')
        time.sleep(duration)
        print(f'[ROS2 Bag Recorder] Duration {duration}s reached')
        proc.terminate()
        try:
            proc.wait(timeout=5)
        except subprocess.TimeoutExpired:
            proc.kill()
        print('[ROS2 Bag Recorder] Recording stopped')
    else:
        proc.wait()

    print(f'[ROS2 Bag Recorder] Recording saved to: {bag_path}')


def main():
    parser = argparse.ArgumentParser(description='ROS2 Bag Recorder to MCAP')
    parser.add_argument('-o', '--output', default='./bags',
                        help='Output directory for recorded bags')
    parser.add_argument('-d', '--duration', type=int, default=None,
                        help='Recording duration in seconds (default: unlimited)')
    parser.add_argument('-t', '--topics', nargs='+', default=None,
                        help='Specific topics to record (default: all)')
    parser.add_argument('-x', '--exclude', nargs='+', default=None,
                        help='Topics to exclude from recording')
    parser.add_argument('-q', '--queue-size', type=int, default=100,
                        help='Subscriber queue size')
    parser.add_argument('--list-topics', action='store_true',
                        help='List all available topics and exit')
    parser.add_argument('--use-cli', action='store_true',
                        help='Use ros2 bag CLI instead of rosbag2_py API')
    args = parser.parse_args()

    rclpy.init(args=None)

    if args.list_topics:
        node = Node('topic_lister')
        time.sleep(1.0)
        topics = node.get_topic_names_and_types()
        print('Available topics:')
        for name, types in topics:
            print(f'  {name} ({", ".join(types)})')
        node.destroy_node()
        rclpy.shutdown()
        return

    if args.use_cli or not HAS_ROSBAG2_PY:
        if not HAS_ROSBAG2_PY:
            print('[ROS2 Bag Recorder] rosbag2_py not available, falling back to CLI')
        rclpy.shutdown()
        cli_recorder(args.output, args.duration, args.topics, args.exclude)
        return

    recorder = RosbagRecorderNode(
        output_dir=args.output,
        duration=args.duration,
        topics=args.topics,
        exclude_topics=args.exclude,
        queue_size=args.queue_size
    )

    def signal_handler(sig, frame):
        recorder.get_logger().info('Received SIGINT, stopping recording...')
        recorder.stop_recording()
        rclpy.shutdown()
        sys.exit(0)

    signal.signal(signal.SIGINT, signal_handler)

    recorder.start_recording()

    if args.duration:
        duration_thread = Thread(target=recorder._check_duration, daemon=True)
        duration_thread.start()

    try:
        rclpy.spin(recorder)
    except KeyboardInterrupt:
        recorder.stop_recording()
    finally:
        recorder.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()