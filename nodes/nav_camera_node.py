# -*- coding: utf-8 -*-
"""
nodes/nav_camera_node.py -- ROS 2 node driving the FIT0701 USB camera
for navigation assistance: corridor obstacle density scoring and
ArUco/AprilTag marker detection.

Publishes
    /nav_camera/image_raw            sensor_msgs/Image  (lazy)
    /nav_camera/obstacle_density     std_msgs/Float32
    /nav_camera/markers              std_msgs/String    (JSON)
"""

import sys
import os
import json

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Float32, String
from cv_bridge import CvBridge

import cv2
import numpy as np

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))
from config import (
    NAV_CAM_DEVICE,
    NAV_CAM_WIDTH,
    NAV_CAM_HEIGHT,
    NAV_CAM_FPS,
)
from core.vision_navigation import NavigationVision


class NavCameraNode(Node):
    """Drives the FIT0701 USB camera for navigation and marker detection."""

    def __init__(self) -> None:
        super().__init__("nav_camera_node")
        self.get_logger().info("NavCameraNode starting ...")

        self._vision = NavigationVision()
        self._bridge = CvBridge()

        # -- Open camera -----------------------------------
        self._cap = cv2.VideoCapture(NAV_CAM_DEVICE, cv2.CAP_V4L2)
        if not self._cap.isOpened():
            self._cap = cv2.VideoCapture(NAV_CAM_DEVICE)

        if self._cap.isOpened():
            self._cap.set(cv2.CAP_PROP_FRAME_WIDTH, NAV_CAM_WIDTH)
            self._cap.set(cv2.CAP_PROP_FRAME_HEIGHT, NAV_CAM_HEIGHT)
            self._cap.set(cv2.CAP_PROP_FPS, NAV_CAM_FPS)
            self.get_logger().info(
                f"  FIT0701 opened: device={NAV_CAM_DEVICE}, "
                f"{NAV_CAM_WIDTH}x{NAV_CAM_HEIGHT}@{NAV_CAM_FPS}fps"
            )
        else:
            self.get_logger().error(
                f"  FAILED to open FIT0701 at device {NAV_CAM_DEVICE}. "
                "Node will publish default values."
            )

        # -- Publishers ------------------------------------
        self._pub_image = self.create_publisher(
            Image, "/nav_camera/image_raw", 10
        )
        self._pub_density = self.create_publisher(
            Float32, "/nav_camera/obstacle_density", 10
        )
        self._pub_markers = self.create_publisher(
            String, "/nav_camera/markers", 10
        )

        # -- Timer: grab + process at camera FPS -----------
        period = 1.0 / NAV_CAM_FPS
        self._timer = self.create_timer(period, self._tick)

        self.get_logger().info("NavCameraNode ready.")

    # ------------------------------------------
    #  Main processing tick
    # ------------------------------------------

    def _tick(self) -> None:
        if not self._cap.isOpened():
            # Publish safe defaults when camera is unavailable
            density_msg = Float32()
            density_msg.data = 0.0
            self._pub_density.publish(density_msg)
            return

        ret, frame = self._cap.read()
        if not ret or frame is None:
            return

        # -- Obstacle density (Canny-based) ----------------
        density = self._vision.compute_obstacle_density(frame)
        density_msg = Float32()
        density_msg.data = density
        self._pub_density.publish(density_msg)

        # -- ArUco marker detection ------------------------
        markers = self._vision.detect_markers(frame)
        markers_json = []
        for m in markers:
            corners = m["corners"]
            # Compute centre of marker for downstream use
            cx = float(np.mean(corners[:, 0]))
            cy = float(np.mean(corners[:, 1]))
            markers_json.append({
                "id": int(m["id"]),
                "cx": round(cx, 1),
                "cy": round(cy, 1),
            })

        markers_msg = String()
        markers_msg.data = json.dumps(markers_json)
        self._pub_markers.publish(markers_msg)

        if markers_json:
            self.get_logger().info(
                f"Markers detected: {[m['id'] for m in markers_json]}",
                throttle_duration_sec=2.0,
            )

        # -- Lazy image publishing -------------------------
        if self._pub_image.get_subscription_count() > 0:
            try:
                annotated = self._vision.annotate_frame(frame, markers)
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
            self.get_logger().info("FIT0701 camera released.")
        super().destroy_node()


# ------------------------------------------
#  Entry Point
# ------------------------------------------

def main(args=None) -> None:
    rclpy.init(args=args)
    node = NavCameraNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
