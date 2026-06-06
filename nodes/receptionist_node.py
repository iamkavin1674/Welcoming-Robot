# -*- coding: utf-8 -*-
"""
nodes/receptionist_node.py — Receptionist greeting-workflow orchestrator.

Implements a five-state machine that detects a visitor, greets them,
reads a QR-code destination, navigates to the named location, and
announces arrival before returning to idle.

State Machine
    IDLE → GREETING → WAITING_QR → NAVIGATING → ARRIVED → IDLE

Subscriptions
    /visitor/detected          std_msgs/Bool    — visitor presence flag
    /visitor/qr_data           std_msgs/String  — scanned QR payload (destination name)
    /navigation/goal_reached   std_msgs/Bool    — navigation-complete signal

Publications
    /goal_pose                 geometry_msgs/PoseStamped  — navigation goal
    /receptionist/status       std_msgs/String            — status for UI / logging
    /receptionist/greeting     std_msgs/String            — TTS greeting text
"""

import sys
import os
import enum
import time

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Bool, String

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))
from config import (
    NAMED_DESTINATIONS,
    GREETING_MESSAGE,
    VISITOR_PRESENCE_COOLDOWN,
    MAP_FRAME,
)


# ──────────────────────────────────────────────
#  State Enum
# ──────────────────────────────────────────────

class _State(enum.Enum):
    """Receptionist workflow states."""
    IDLE = 0
    GREETING = 1
    WAITING_QR = 2
    NAVIGATING = 3
    ARRIVED = 4


# ──────────────────────────────────────────────
#  Constants
# ──────────────────────────────────────────────

_TICK_HZ: float = 10.0            # state-machine tick rate
_QR_TIMEOUT: float = 30.0         # seconds to wait for QR scan
_ARRIVAL_DISPLAY: float = 2.0     # seconds to hold arrival announcement


class ReceptionistNode(Node):
    """Orchestrates the receptionist greeting → QR scan → navigate → arrival workflow."""

    def __init__(self) -> None:
        super().__init__("receptionist_node")
        self.get_logger().info("ReceptionistNode starting …")

        # ── State ──
        self._state: _State = _State.IDLE
        self._last_visitor_time: float = 0.0          # cooldown tracker
        self._qr_wait_start: float = 0.0              # when WAITING_QR began
        self._arrival_start: float = 0.0              # when ARRIVED began
        self._destination_name: str = ""               # current target name

        # Volatile flags set by subscriber callbacks, consumed by the tick
        self._visitor_detected: bool = False
        self._qr_payload: str = ""
        self._goal_reached: bool = False

        # ── Publishers ──
        self._pub_goal = self.create_publisher(
            PoseStamped, "/goal_pose", 10
        )
        self._pub_status = self.create_publisher(
            String, "/receptionist/status", 10
        )
        self._pub_greeting = self.create_publisher(
            String, "/receptionist/greeting", 10
        )

        # ── Subscribers ──
        self.create_subscription(
            Bool, "/visitor/detected", self._visitor_cb, 10
        )
        self.create_subscription(
            String, "/visitor/qr_data", self._qr_cb, 10
        )
        self.create_subscription(
            Bool, "/navigation/goal_reached", self._goal_reached_cb, 10
        )

        # ── Tick Timer (10 Hz) ──
        period: float = 1.0 / _TICK_HZ
        self._tick_timer = self.create_timer(period, self._tick)

        self.get_logger().info("ReceptionistNode ready.")

    # ══════════════════════════════════════════
    #  Subscription Callbacks
    # ══════════════════════════════════════════

    def _visitor_cb(self, msg: Bool) -> None:
        """Latch visitor-detected flag for the tick to consume."""
        self._visitor_detected = msg.data

    def _qr_cb(self, msg: String) -> None:
        """Store last QR payload (non-empty strings only)."""
        if msg.data:
            self._qr_payload = msg.data.strip()

    def _goal_reached_cb(self, msg: Bool) -> None:
        """Latch goal-reached flag for the tick to consume."""
        if msg.data:
            self._goal_reached = True

    # ══════════════════════════════════════════
    #  Helper Publishers
    # ══════════════════════════════════════════

    def _publish_status(self, text: str) -> None:
        """Publish a status string and log it."""
        msg = String()
        msg.data = text
        self._pub_status.publish(msg)
        self.get_logger().info(f"[status] {text}")

    def _publish_greeting(self, text: str) -> None:
        """Publish a TTS greeting string."""
        msg = String()
        msg.data = text
        self._pub_greeting.publish(msg)

    # ══════════════════════════════════════════
    #  Navigation Goal Builder
    # ══════════════════════════════════════════

    def _build_goal_pose(self, x: float, y: float) -> PoseStamped:
        """
        Build a PoseStamped in MAP_FRAME with z=0 and identity quaternion.

        Parameters
        ----------
        x : float
            Target x position in metres (map frame).
        y : float
            Target y position in metres (map frame).

        Returns
        -------
        PoseStamped
            Ready-to-publish navigation goal.
        """
        goal = PoseStamped()
        goal.header.stamp = self.get_clock().now().to_msg()
        goal.header.frame_id = MAP_FRAME
        goal.pose.position.x = x
        goal.pose.position.y = y
        goal.pose.position.z = 0.0
        # Identity quaternion (no rotation)
        goal.pose.orientation.x = 0.0
        goal.pose.orientation.y = 0.0
        goal.pose.orientation.z = 0.0
        goal.pose.orientation.w = 1.0
        return goal

    # ══════════════════════════════════════════
    #  State-Machine Tick  (10 Hz)
    # ══════════════════════════════════════════

    def _tick(self) -> None:
        """Main state-machine tick — called at _TICK_HZ."""

        # ─── IDLE ────────────────────────────
        if self._state == _State.IDLE:
            self._tick_idle()
            return

        # ─── GREETING ────────────────────────
        if self._state == _State.GREETING:
            self._tick_greeting()
            return

        # ─── WAITING_QR ──────────────────────
        if self._state == _State.WAITING_QR:
            self._tick_waiting_qr()
            return

        # ─── NAVIGATING ──────────────────────
        if self._state == _State.NAVIGATING:
            self._tick_navigating()
            return

        # ─── ARRIVED ─────────────────────────
        if self._state == _State.ARRIVED:
            self._tick_arrived()
            return

    # ──────────────────────────────────────────
    #  Per-State Handlers
    # ──────────────────────────────────────────

    def _tick_idle(self) -> None:
        """IDLE: wait for a visitor, respecting the cooldown window."""
        if not self._visitor_detected:
            return

        now: float = time.monotonic()
        if now - self._last_visitor_time < VISITOR_PRESENCE_COOLDOWN:
            return  # still in cooldown

        self.get_logger().info("Visitor detected — transitioning to GREETING.")
        self._last_visitor_time = now
        self._state = _State.GREETING

    def _tick_greeting(self) -> None:
        """GREETING: publish the greeting then advance to WAITING_QR."""
        self._publish_greeting(GREETING_MESSAGE)
        self._publish_status("Greeting visitor — waiting for QR code.")

        # Reset QR data so we only accept fresh scans
        self._qr_payload = ""
        self._qr_wait_start = time.monotonic()

        self.get_logger().info("Greeting sent — transitioning to WAITING_QR.")
        self._state = _State.WAITING_QR

    def _tick_waiting_qr(self) -> None:
        """WAITING_QR: wait for a valid QR destination or time out."""
        # Check timeout first
        elapsed: float = time.monotonic() - self._qr_wait_start
        if elapsed > _QR_TIMEOUT:
            self.get_logger().warn(
                f"QR scan timed out after {_QR_TIMEOUT:.0f}s — returning to IDLE."
            )
            self._publish_status("QR scan timed out. Returning to idle.")
            self._state = _State.IDLE
            return

        # No data yet
        if not self._qr_payload:
            return

        destination: str = self._qr_payload
        self._qr_payload = ""  # consume

        if destination not in NAMED_DESTINATIONS:
            self.get_logger().error(
                f"Unknown destination '{destination}'. "
                f"Valid: {list(NAMED_DESTINATIONS.keys())}"
            )
            self._publish_status(
                f"Unknown destination '{destination}'. Returning to idle."
            )
            self._state = _State.IDLE
            return

        # Valid destination
        self._destination_name = destination
        coords = NAMED_DESTINATIONS[destination]
        goal_msg: PoseStamped = self._build_goal_pose(coords["x"], coords["y"])
        self._pub_goal.publish(goal_msg)

        self.get_logger().info(
            f\"Navigating to '{destination}' at \"
            f\"({coords['x']:.2f}, {coords['y']:.2f}).\"
        )
        self._publish_status(f"Navigating to {destination}.")

        # Reset goal-reached flag before waiting
        self._goal_reached = False
        self._state = _State.NAVIGATING

    def _tick_navigating(self) -> None:
        """NAVIGATING: wait for the navigation stack to report goal reached."""
        if not self._goal_reached:
            return

        self._goal_reached = False
        self.get_logger().info(
            f"Arrived at '{self._destination_name}' — transitioning to ARRIVED."
        )
        self._arrival_start = time.monotonic()
        self._state = _State.ARRIVED

    def _tick_arrived(self) -> None:
        """ARRIVED: announce arrival, hold for a short period, then go IDLE."""
        elapsed: float = time.monotonic() - self._arrival_start

        if elapsed < 0.15:
            # Publish announcements once (first tick only)
            arrival_text: str = (
                f"We have arrived at {self._destination_name}. "
                "Have a great day!"
            )
            self._publish_greeting(arrival_text)
            self._publish_status(
                f"Arrived at {self._destination_name}."
            )

        if elapsed >= _ARRIVAL_DISPLAY:
            self.get_logger().info(
                "Arrival display complete — transitioning to IDLE."
            )
            self._publish_status("Returning to idle.")
            self._state = _State.IDLE


# ──────────────────────────────────────────────
#  Entry Point
# ──────────────────────────────────────────────

def main(args=None) -> None:
    """Spin the ReceptionistNode until shutdown."""
    rclpy.init(args=args)
    node = ReceptionistNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
