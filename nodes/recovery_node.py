"""
nodes/recovery_node.py -- ROS 2 node that handles stuck / blocked
situations with a simple behaviour sequence:

    1. Stop  -> zero velocity for a pause.
    2. Back up -> small negative linear.x pulse.
    3. Rotate -> slow angular.z sweep to refresh sensors.
    4. Report -> publish "recovered" or "failed" on /recovery/status.

Subscriptions
    /recovery/trigger        std_msgs/Bool   (True = begin recovery)

Publications
    /cmd_vel_recovery        geometry_msgs/Twist  (via cmd_vel_mux)
    /recovery/status         std_msgs/String
"""

import sys
import os
import enum

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool, String

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))
from config import (
    MAX_RECOVERY_ATTEMPTS,
    RECOVERY_ROTATION_SPEED,
    RECOVERY_ROTATION_DURATION,
    RECOVERY_BACKUP_SPEED,
    RECOVERY_BACKUP_DURATION,
    RECOVERY_PAUSE_DURATION,
)


class _Phase(enum.Enum):
    IDLE = 0
    STOPPING = 1
    BACKING_UP = 2
    ROTATING = 3
    DONE = 4


class RecoveryNode(Node):
    """Simple recovery behaviour manager."""

    def __init__(self) -> None:
        super().__init__("recovery_node")
        self.get_logger().info("RecoveryNode starting …")

        # ── State ──
        self._phase = _Phase.IDLE
        self._attempt = 0
        self._phase_start_time = self.get_clock().now()

        # ── Publishers ──
        self._pub_cmd = self.create_publisher(Twist, "/cmd_vel_recovery", 10)
        self._pub_status = self.create_publisher(String, "/recovery/status", 10)

        # ── Subscribers ──
        self._sub_trigger = self.create_subscription(
            Bool, "/recovery/trigger", self._trigger_callback, 10
        )

        # ── Timer — 10 Hz state machine ──
        self._timer = self.create_timer(0.1, self._tick)

        self.get_logger().info("RecoveryNode ready.")

    # ──────────────────────────────────────────
    #  Trigger
    # ──────────────────────────────────────────

    def _trigger_callback(self, msg: Bool) -> None:
        if msg.data and self._phase == _Phase.IDLE:
            self._attempt += 1
            if self._attempt > MAX_RECOVERY_ATTEMPTS:
                
                self.get_logger().error(
                    f"Max recovery attempts ({MAX_RECOVERY_ATTEMPTS}) exhausted."
                )
                self._publish_status("failed")
                self._attempt = 0
                return
            self.get_logger().warn(
                f"Recovery triggered (attempt {self._attempt}/{MAX_RECOVERY_ATTEMPTS})"
            )
            self._enter_phase(_Phase.STOPPING)

    # ──────────────────────────────────────────
    #  State Machine
    # ──────────────────────────────────────────

    def _tick(self) -> None:
        if self._phase == _Phase.IDLE:
            return

        elapsed = (
            self.get_clock().now() - self._phase_start_time
        ).nanoseconds / 1e9

        if self._phase == _Phase.STOPPING:
            self._pub_cmd.publish(Twist())  # zero velocity
            if elapsed >= RECOVERY_PAUSE_DURATION:
                self._enter_phase(_Phase.BACKING_UP)

        elif self._phase == _Phase.BACKING_UP:
            twist = Twist()
            twist.linear.x = RECOVERY_BACKUP_SPEED
            self._pub_cmd.publish(twist)
            if elapsed >= RECOVERY_BACKUP_DURATION:
                self._enter_phase(_Phase.ROTATING)

        elif self._phase == _Phase.ROTATING:
            twist = Twist()
            twist.angular.z = RECOVERY_ROTATION_SPEED
            self._pub_cmd.publish(twist)
            if elapsed >= RECOVERY_ROTATION_DURATION:
                self._enter_phase(_Phase.DONE)

        elif self._phase == _Phase.DONE:
            self._pub_cmd.publish(Twist())  # stop
            self.get_logger().info("Recovery sequence complete.")
            self._publish_status("recovered")
            self._phase = _Phase.IDLE

    # ──────────────────────────────────────────
    #  Helpers
    # ──────────────────────────────────────────

    def _enter_phase(self, phase: _Phase) -> None:
        self.get_logger().info(f"Recovery phase → {phase.name}")
        self._phase = phase
        self._phase_start_time = self.get_clock().now()

    def _publish_status(self, status: str) -> None:
        msg = String()
        msg.data = status
        self._pub_status.publish(msg)

    def reset(self) -> None:
        """Reset attempt counter (called after a successful navigation)."""
        self._attempt = 0
        self._phase = _Phase.IDLE


# ──────────────────────────────────────────────
#  Entry Point
# ──────────────────────────────────────────────

def main(args=None) -> None:
    rclpy.init(args=args)
    node = RecoveryNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
