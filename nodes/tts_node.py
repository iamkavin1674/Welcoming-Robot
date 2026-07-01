# -*- coding: utf-8 -*-
"""
nodes/tts_node.py -- ROS 2 node wrapping the TTS engine.

This node acts as a reusable speech service for the entire robot.
Any node can publish to /tts_request to have text spoken aloud —
without importing pyttsx3 or knowing which TTS backend is used.

Architecture
    - This node owns ALL TTS-related ROS communication.
    - The actual speech logic lives in core/tts_engine.py.
    - Swapping TTS backends (e.g. pyttsx3 → Piper) requires
      changing only tts_engine.py.

Subscriptions
    /tts_request            std_msgs/String   (text to speak)

Publications
    /tts_status             std_msgs/String   ("speaking" / "idle")
"""

import sys
import os

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))
from core.tts_engine import TTSEngine


class TTSNode(Node):
    """ROS 2 wrapper around TTSEngine.

    Subscribes to ``/tts_request`` and plays received text through
    the system speakers.  Publishes status updates on ``/tts_status``
    so other nodes know when the robot is speaking.
    """

    def __init__(self) -> None:
        super().__init__("tts_node")
        self.get_logger().info("TTSNode starting ...")

        # -- TTS engine (pure Python, no ROS) ---------------
        self._engine = TTSEngine()

        if not self._engine.is_available():
            self.get_logger().warn(
                "TTS engine unavailable — speech will be logged only."
            )

        # -- Publisher --------------------------------------
        self._pub_status = self.create_publisher(
            String, "/tts_status", 10
        )

        # -- Subscriber ------------------------------------
        self.create_subscription(
            String, "/tts_request", self._request_cb, 10
        )

        # Publish initial idle status
        self._publish_status("idle")

        self.get_logger().info("TTSNode ready — listening on /tts_request.")

    # --------------------------------------------------
    #  Request Callback
    # --------------------------------------------------

    def _request_cb(self, msg: String) -> None:
        """Handle an incoming TTS request.

        Speaks the text and publishes status transitions:
        ``idle`` → ``speaking`` → ``idle``.

        Parameters
        ----------
        msg : String
            The text to speak.
        """
        text: str = msg.data.strip()
        if not text:
            return

        self.get_logger().info(f"TTS request: '{text}'")

        # Signal that we are speaking
        self._publish_status("speaking")

        # Block until speech completes
        self._engine.speak(text)

        # Signal that we are done
        self._publish_status("idle")

    # --------------------------------------------------
    #  Status Publisher
    # --------------------------------------------------

    def _publish_status(self, status: str) -> None:
        """Publish the current TTS status.

        Parameters
        ----------
        status : str
            Either ``"speaking"`` or ``"idle"``.
        """
        msg = String()
        msg.data = status
        self._pub_status.publish(msg)


# --------------------------------------------------
#  Entry Point
# --------------------------------------------------

def main(args=None) -> None:
    """Spin the TTSNode until shutdown."""
    rclpy.init(args=args)
    node = TTSNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
