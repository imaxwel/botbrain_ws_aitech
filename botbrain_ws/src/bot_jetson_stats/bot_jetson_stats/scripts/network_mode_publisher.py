#!/usr/bin/env python3

import rclpy
from rclpy.lifecycle import LifecycleNode
from rclpy.lifecycle import State
from rclpy.lifecycle import TransitionCallbackReturn
from std_msgs.msg import String
import os

STATUS_FILE = "/opt/network_status/network_mode_status.txt"


class NetworkModePublisher(LifecycleNode):
    def __init__(self):
        super().__init__('network_mode_publisher')

        self.publisher_ = self.create_lifecycle_publisher(String, 'network_mode_status', 10)

        self.last_status = None
        self.timer = None

    def _ensure_publisher(self):
        if self.publisher_ is None:
            self.publisher_ = self.create_lifecycle_publisher(String, 'network_mode_status', 10)

    # -------- Lifecycle Callbacks -------- #
    def on_configure(self, state: State) -> TransitionCallbackReturn:
        self.get_logger().info("Configuring node...")

        self.last_status = None
        self._ensure_publisher()

        return TransitionCallbackReturn.SUCCESS

    def on_activate(self, state: State) -> TransitionCallbackReturn:
        self.get_logger().info("Activating node...")
        try:

            self._ensure_publisher()
            ret = super().on_activate(state)
            if ret != TransitionCallbackReturn.SUCCESS:
                return ret

            self.timer = self.create_timer(10.0, self.publish_status)
            return TransitionCallbackReturn.SUCCESS
        except Exception as e:
            self.get_logger().error(f"Error activating node: {e}")
            return TransitionCallbackReturn.FAILURE

    def on_deactivate(self, state: State) -> TransitionCallbackReturn:
        self.get_logger().info("Deactivating node...")

        super().on_deactivate(state)

        if self.timer:
            self.timer.cancel()
            self.timer = None

        return TransitionCallbackReturn.SUCCESS

    def on_cleanup(self, state: State) -> TransitionCallbackReturn:
        self.get_logger().info("Cleaning up node...")
        if self.timer:
            self.timer.cancel()
            self.timer = None
        if self.publisher_ is not None:
            try:
                self.destroy_publisher(self.publisher_)
            except Exception:
                pass
            self.publisher_ = None
        return TransitionCallbackReturn.SUCCESS

    def on_shutdown(self, state: State) -> TransitionCallbackReturn:
        self.get_logger().info("Shutting down node...")
        if self.timer:
            self.timer.cancel()
            self.timer = None
        return TransitionCallbackReturn.SUCCESS

    def publish_status(self):
        try:
            if self.publisher_ is None:
                return
            if os.path.exists(STATUS_FILE):
                with open(STATUS_FILE, 'r') as f:
                    status = f.read().strip()

                if status != self.last_status:
                    msg = String()
                    msg.data = status
                    self.publisher_.publish(msg)
                    self.get_logger().info(f'Published network status: {status}')
                    self.last_status = status
        except Exception as e:
            self.get_logger().error(f"Error reading status file: {e}")


def main(args=None):
    rclpy.init(args=args)
    node = NetworkModePublisher()
    executor = rclpy.executors.MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()