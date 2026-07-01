# -*- coding: utf-8 -*-
"""
nodes/receptionist_node.py -- Receptionist greeting-workflow orchestrator.

Implements a six-state machine that detects a visitor, requests an
AI-generated greeting from the LLM node, speaks it via the TTS node,
reads a QR-code destination, navigates to the named location, and
announces arrival before returning to idle.

State Machine
    IDLE -> GREETING -> AWAITING_LLM -> WAITING_QR -> NAVIGATING -> ARRIVED -> IDLE

Subscriptions
    /visitor/detected          std_msgs/Bool    -- visitor presence flag
    /visitor/qr_data           std_msgs/String  -- scanned QR payload (destination name)
    /navigation/goal_reached   std_msgs/Bool    -- navigation-complete signal
    /llm_response              std_msgs/String  -- AI-generated greeting text

Publications
    /goal_pose                 geometry_msgs/PoseStamped  -- navigation goal
    /receptionist/status       std_msgs/String            -- status for UI / logging
    /receptionist/greeting     std_msgs/String            -- greeting text
    /llm_request               std_msgs/String            -- request to LLM node
    /tts_request               std_msgs/String            -- text for TTS node
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
    LLM_RESPONSE_TIMEOUT,
    LLM_FALLBACK_GREETING,
)


# ──────────────────────────────────────────────
#  State Enum
# ──────────────────────────────────────────────

class _State(enum.Enum):
    """Receptionist workflow states."""
    IDLE = 0
    GREETING = 1
    AWAITING_LLM = 2
    WAITING_QR = 3
    NAVIGATING = 4
    ARRIVED = 5


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
        self._llm_response: str = ""                  # set by /llm_response callback
        self._llm_request_time: float = 0.0            # when LLM request was sent

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
        self._pub_llm_request = self.create_publisher(
            String, "/llm_request", 10
        )
        self._pub_tts_request = self.create_publisher(
            String, "/tts_request", 10
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
        self.create_subscription(
            String, "/llm_response", self._llm_response_cb, 10
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

    def _llm_response_cb(self, msg: String) -> None:
        """Store LLM response for the AWAITING_LLM state to consume."""
        if msg.data:
            self._llm_response = msg.data.strip()

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

        # ─── AWAITING_LLM ────────────────────
        if self._state == _State.AWAITING_LLM:
            self._tick_awaiting_llm()
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
        """GREETING: request an AI greeting from the LLM node."""
        # Publish request to LLM node
        request_msg = String()
        request_msg.data = "greeting"
        self._pub_llm_request.publish(request_msg)

        # Reset LLM response and start timeout clock
        self._llm_response = ""
        self._llm_request_time = time.monotonic()

        self._publish_status("Requesting AI greeting ...")
        self.get_logger().info(
            "LLM request sent — transitioning to AWAITING_LLM."
        )
        self._state = _State.AWAITING_LLM

    def _tick_awaiting_llm(self) -> None:
        """AWAITING_LLM: wait for the LLM response, then greet and
        advance to WAITING_QR."""
        # Check for timeout
        elapsed: float = time.monotonic() - self._llm_request_time
        if elapsed > LLM_RESPONSE_TIMEOUT:
            self.get_logger().warn(
                f"LLM response timed out after {LLM_RESPONSE_TIMEOUT:.0f}s "
                "— using fallback greeting."
            )
            greeting_text = LLM_FALLBACK_GREETING
        elif self._llm_response:
            greeting_text = self._llm_response
            self._llm_response = ""  # consume
        else:
            return  # still waiting

        # Publish greeting to TTS, greeting topic, and status
        tts_msg = String()
        tts_msg.data = greeting_text
        self._pub_tts_request.publish(tts_msg)

        self._publish_greeting(greeting_text)
        self._publish_status("Greeting visitor — waiting for QR code.")

        # Reset QR data so we only accept fresh scans
        self._qr_payload = ""
        self._qr_wait_start = time.monotonic()

        self.get_logger().info(
            f"Greeting spoken: '{greeting_text}' — "
            "transitioning to WAITING_QR."
        )
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

            # Also speak the arrival announcement
            tts_msg = String()
            tts_msg.data = arrival_text
            self._pub_tts_request.publish(tts_msg)

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
