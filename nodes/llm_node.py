# -*- coding: utf-8 -*-
"""
nodes/llm_node.py -- ROS 2 node wrapping the Gemini interface.

This node acts as a reusable LLM service for the entire robot.
Any node can publish to /llm_request to get AI-generated text
back on /llm_response — without importing Gemini or knowing
which LLM provider is used.

Architecture
    - This node owns ALL LLM-related ROS communication.
    - The actual Gemini logic lives in core/gemini_interface.py.
    - Swapping LLM providers requires changing only gemini_interface.py.

Subscriptions
    /llm_request            std_msgs/String   (request type, e.g. "greeting")

Publications
    /llm_response           std_msgs/String   (generated text)
"""

import sys
import os
from datetime import datetime

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))
from config import LLM_FALLBACK_GREETING
from core.gemini_interface import GeminiInterface


class LLMNode(Node):
    """ROS 2 wrapper around GeminiInterface.

    Subscribes to ``/llm_request`` and publishes AI-generated text
    on ``/llm_response``.  Currently supports one request type:

    - ``"greeting"`` — generates a time-aware receptionist greeting.

    Future request types (e.g. ``"escort_announce"``, ``"farewell"``)
    can be added by extending the ``_handle_request`` method.
    """

    def __init__(self) -> None:
        super().__init__("llm_node")
        self.get_logger().info("LLMNode starting ...")

        # -- Gemini interface (pure Python, no ROS) --------
        self._gemini = GeminiInterface()

        # -- Publisher -------------------------------------
        self._pub_response = self.create_publisher(
            String, "/llm_response", 10
        )

        # -- Subscriber ------------------------------------
        self.create_subscription(
            String, "/llm_request", self._request_cb, 10
        )

        self.get_logger().info("LLMNode ready — listening on /llm_request.")

    # --------------------------------------------------
    #  Request Callback
    # --------------------------------------------------

    def _request_cb(self, msg: String) -> None:
        """Handle an incoming LLM request.

        Parameters
        ----------
        msg : String
            The request payload.  Currently supports:
            - ``"greeting"`` — generate a time-aware greeting.
        """
        request_type: str = msg.data.strip().lower()
        self.get_logger().info(f"LLM request received: '{request_type}'")

        response_text: str = self._handle_request(request_type)

        # Publish response
        response_msg = String()
        response_msg.data = response_text
        self._pub_response.publish(response_msg)
        self.get_logger().info(f"LLM response published: '{response_text}'")

    # --------------------------------------------------
    #  Request Dispatcher
    # --------------------------------------------------

    def _handle_request(self, request_type: str) -> str:
        """Dispatch the request to the appropriate handler.

        Parameters
        ----------
        request_type : str
            The type of content to generate.

        Returns
        -------
        str
            The generated text.
        """
        if request_type == "greeting":
            return self._generate_greeting()

        self.get_logger().warn(
            f"Unknown request type '{request_type}' — using fallback."
        )
        return LLM_FALLBACK_GREETING

    # --------------------------------------------------
    #  Greeting Handler
    # --------------------------------------------------

    def _generate_greeting(self) -> str:
        """Generate a time-aware greeting via Gemini.

        Returns
        -------
        str
            The AI-generated greeting text.
        """
        time_str: str = datetime.now().strftime("%H:%M")
        self.get_logger().info(f"Requesting greeting for time {time_str}.")
        return self._gemini.generate_greeting(time_str)


# --------------------------------------------------
#  Entry Point
# --------------------------------------------------

def main(args=None) -> None:
    """Spin the LLMNode until shutdown."""
    rclpy.init(args=args)
    node = LLMNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
