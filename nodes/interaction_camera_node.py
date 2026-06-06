# -*- coding: utf-8 -*-
"""
nodes/interaction_camera_node.py -- ROS 2 node driving the OV2710 USB
camera for human interaction: face detection, visitor presence, and
QR code scanning.

Publishes
    /interaction_camera/image_raw    sensor_msgs/Image   (lazy: only when subscribers exist)
    /visitor/detected                std_msgs/Bool
    /visitor/face_count              std_msgs/Int32
    /visitor/qr_data                 std_msgs/String
"""

import sys
import os
import time as _time

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Bool, Int32, String
from cv_bridge import CvBridge

import cv2
import numpy as np

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))
from config import (
    INTERACTION_CAM_DEVICE,
    INTERACTION_CAM_WIDTH,
    INTERACTION_CAM_HEIGHT,
    INTERACTION_CAM_FPS,
    QR_SCAN_INTERVAL,
)
from core.vision_interaction import InteractionVision


class InteractionCameraNode(Node):
    """Drives the OV2710 USB camera for face detection and QR scanning."""

    def __init__(self) -> None:
        super().__init__("interaction_camera_node")
        self.get_logger().info("InteractionCameraNode starting ...")

        self._vision = InteractionVision()
        self._bridge = CvBridge()

        # -- Open camera -----------------------------------
        self._cap = cv2.VideoCapture(INTERACTION_CAM_DEVICE, cv2.CAP_V4L2)
        if not self._cap.isOpened():
            # Fallback: try without V4L2 backend (for non-Linux dev)
            self._cap = cv2.VideoCapture(INTERACTION_CAM_DEVICE)

        if self._cap.isOpened():
            self._cap.set(cv2.CAP_PROP_FRAME_WIDTH, INTERACTION_CAM_WIDTH)
            self._cap.set(cv2.CAP_PROP_FRAME_HEIGHT, INTERACTION_CAM_HEIGHT)
            self._cap.set(cv2.CAP_PROP_FPS, INTERACTION_CAM_FPS)
            self.get_logger().info(
                f"  OV2710 opened: device={INTERACTION_CAM_DEVICE}, "
                f"{INTERACTION_CAM_WIDTH}x{INTERACTION_CAM_HEIGHT}@{INTERACTION_CAM_FPS}fps"
            )
        else:
            self.get_logger().error(
                f"  FAILED to open OV2710 at device {INTERACTION_CAM_DEVICE}. "
                "Node will publish empty data."
            )

        # -- Publishers ------------------------------------
        self._pub_image = self.create_publisher(
            Image, "/interaction_camera/image_raw", 10
        )
        self._pub_detected = self.create_publisher(Bool, "/visitor/detected", 10)
        self._pub_face_count = self.create_publisher(Int32, "/visitor/face_count", 10)
        self._pub_qr = self.create_publisher(String, "/visitor/qr_data", 10)

        # -- QR scan throttle ------------------------------
        self._last_qr_scan = 0.0

        # -- Timer: grab + process at camera FPS -----------
        period = 1.0 / INTERACTION_CAM_FPS
        self._timer = self.create_timer(period, self._tick)

        self.get_logger().info("InteractionCameraNode ready.")

    # ------------------------------------------
    #  Main processing tick
    # ------------------------------------------

    def _tick(self) -> None:
        if not self._cap.isOpened():
            return

        ret, frame = self._cap.read()
        if not ret or frame is None:
            return

        # -- Face detection --------------------------------
        faces = self._vision.detect_faces(frame)
        face_count = len(faces)

        detected_msg = Bool()
        detected_msg.data = face_count > 0
        self._pub_detected.publish(detected_msg)

        count_msg = Int32()
        count_msg.data = face_count
        self._pub_face_count.publish(count_msg)

        # -- QR scanning (throttled) -----------------------
        now = _time.monotonic()
        qr_data = None
        if now - self._last_qr_scan >= QR_SCAN_INTERVAL:
            self._last_qr_scan = now
            qr_data = self._vision.decode_qr(frame)

        qr_msg = String()
        qr_msg.data = qr_data if qr_data else ""
        self._pub_qr.publish(qr_msg)

        # -- Lazy image publishing -------------------------
        if self._pub_image.get_subscription_count() > 0:
            try:
                annotated = self._vision.annotate_frame(frame, faces, qr_data)
                img_msg = self._bridge.cv2_to_imgmsg(annotated, encoding="bgr8")
                self._pub_image.publish(img_msg)
            except Exception as e:
                self.get_logger().warn(f"Image publish error: {e}")

    # ------------------------------------------
    #  Cleanup
    # ------------------------------------------

    def destroy_node(self) -> None:
        if self._cap.isOpened():
            self._cap.release()
            self.get_logger().info("OV2710 camera released.")
        super().destroy_node()


# ------------------------------------------
#  Entry Point
# ------------------------------------------

def main(args=None) -> None:
    rclpy.init(args=args)
    node = InteractionCameraNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
