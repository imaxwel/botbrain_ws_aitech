#!/usr/bin/env python3

"""Bridge brief Nav2 optimizer gaps while keeping bounded stop behavior."""

import copy
import math
import time
from collections import deque

import rclpy
from geometry_msgs.msg import Twist
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node


PUBLISH_PERIOD_SEC = 1.0 / 30.0
COMMAND_HOLD_SEC = 0.18
ZERO_COMMAND_GRACE_SEC = 0.12
EMA_ALPHA = 0.45
OSCILLATION_ALPHA = 0.22
OSCILLATION_WINDOW_SEC = 0.8
OSCILLATION_MIN_REVERSALS = 3
OSCILLATION_LINEAR_DEADBAND = 0.05
OSCILLATION_ANGULAR_DEADBAND = 0.10
ZERO_EPSILON = 1.0e-4


def _values(command: Twist):
    return (
        command.linear.x,
        command.linear.y,
        command.linear.z,
        command.angular.x,
        command.angular.y,
        command.angular.z,
    )


def _is_zero(command: Twist) -> bool:
    return all(abs(value) <= ZERO_EPSILON for value in _values(command))


def _blend(previous: Twist, current: Twist, alpha: float) -> Twist:
    output = Twist()
    for field in ("x", "y", "z"):
        old_value = getattr(previous.linear, field)
        new_value = getattr(current.linear, field)
        setattr(
            output.linear,
            field,
            alpha * new_value + (1.0 - alpha) * old_value,
        )
        old_value = getattr(previous.angular, field)
        new_value = getattr(current.angular, field)
        setattr(
            output.angular,
            field,
            alpha * new_value + (1.0 - alpha) * old_value,
        )
    return output


class CmdVelContinuity(Node):
    def __init__(self) -> None:
        super().__init__("cmd_vel_continuity")
        self._publisher = self.create_publisher(
            Twist, "cmd_vel_nav_filtered", 10)
        self.create_subscription(
            Twist, "cmd_vel_nav_raw", self._command_callback, 10)
        self.create_timer(PUBLISH_PERIOD_SEC, self._publish)

        self._filtered = Twist()
        self._last_input_time = None
        self._zero_command_since = None
        self._stale = True
        self._direction_history = deque()
        self._oscillation_active = False
        self.get_logger().info(
            "Navigation command continuity active: alpha=%.2f, hold=%.2fs, "
            "zero_grace=%.2fs"
            % (EMA_ALPHA, COMMAND_HOLD_SEC, ZERO_COMMAND_GRACE_SEC))

    def _command_callback(self, command: Twist) -> None:
        now = time.monotonic()
        if not all(math.isfinite(value) for value in _values(command)):
            self.get_logger().error(
                "Rejected non-finite navigation velocity command")
            self._filtered = Twist()
            self._direction_history.clear()
            self._oscillation_active = False
            self._zero_command_since = None
        elif _is_zero(command):
            if self._zero_command_since is None:
                self._zero_command_since = now
            zero_duration = now - self._zero_command_since
            if (
                self._stale or _is_zero(self._filtered) or
                zero_duration >= ZERO_COMMAND_GRACE_SEC
            ):
                # A persistent zero is a real controller stop. The separate
                # high-priority navigation safety topic bypasses this grace
                # period and remains immediate.
                self._filtered = copy.deepcopy(command)
                self._direction_history.clear()
                self._oscillation_active = False
        elif self._stale:
            # The Nav2 velocity smoother already owns acceleration limiting;
            # avoid adding an extra startup ramp after a real stop.
            self._filtered = copy.deepcopy(command)
            self._zero_command_since = None
        else:
            self._zero_command_since = None
            self._record_direction(now, command)
            alpha = (
                OSCILLATION_ALPHA
                if self._oscillation_active else EMA_ALPHA
            )
            self._filtered = _blend(self._filtered, command, alpha)

        self._last_input_time = now
        self._stale = False

    @staticmethod
    def _sign(value: float, deadband: float) -> int:
        if value > deadband:
            return 1
        if value < -deadband:
            return -1
        return 0

    def _record_direction(self, now: float, command: Twist) -> None:
        direction = (
            self._sign(command.linear.x, OSCILLATION_LINEAR_DEADBAND),
            self._sign(command.angular.z, OSCILLATION_ANGULAR_DEADBAND),
        )
        self._direction_history.append((now, direction))
        while (
            self._direction_history and
            now - self._direction_history[0][0] > OSCILLATION_WINDOW_SEC
        ):
            self._direction_history.popleft()

        reversals = [0, 0]
        previous = [0, 0]
        for _, sample in self._direction_history:
            for axis in range(2):
                if sample[axis] == 0:
                    continue
                if previous[axis] != 0 and sample[axis] != previous[axis]:
                    reversals[axis] += 1
                previous[axis] = sample[axis]

        was_active = self._oscillation_active
        self._oscillation_active = max(reversals) >= OSCILLATION_MIN_REVERSALS
        if self._oscillation_active and not was_active:
            self.get_logger().warn(
                "Navigation command oscillation detected; increasing damping")

    def _publish(self) -> None:
        now = time.monotonic()
        input_timed_out = (
            self._last_input_time is None or
            now - self._last_input_time > COMMAND_HOLD_SEC
        )
        zero_grace_expired = (
            self._zero_command_since is not None and
            now - self._zero_command_since >= ZERO_COMMAND_GRACE_SEC
        )
        if input_timed_out or zero_grace_expired:
            self._filtered = Twist()
            self._direction_history.clear()
            self._oscillation_active = False
            if input_timed_out:
                self._stale = True
                self._zero_command_since = None
        self._publisher.publish(self._filtered)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = CmdVelContinuity()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
