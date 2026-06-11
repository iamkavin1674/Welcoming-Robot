# -*- coding: utf-8 -*-
"""
nodes/camera_node.py -- Unified single-camera ROS 2 node for the
Logitech C270.  Replaces the dual-camera architecture (OV2710 +
FIT0701) with a single VideoCapture owner and threaded workers.

Threading Model
    Camera Thread (daemon)      -- owns VideoCapture, pushes frame.copy()
    Nav Worker Thread (daemon)  -- obstacle density + optional ArUco
    Interaction Worker (daemon) -- face detection + QR scanning
    ROS Timers                  -- health monitor (1 Hz), image pub (lazy)

Queue Architecture
    camera_thread --> nav_queue (maxsize=2) --> nav_worker
    camera_thread --> interaction_queue (maxsize=2) --> interaction_worker

Race Condition Prevention
    Workers never touch VideoCapture.  Each queue entry is an independent
    frame.copy().  No shared mutable buffers.  queue.Queue is thread-safe.

Publications
    /camera/image_raw                sensor_msgs/Image   (lazy)
    /camera/status                   std_msgs/String     (ONLINE/OFFLINE/RECONNECTING)
    /visitor/detected                std_msgs/Bool
    /visitor/face_count              std_msgs/Int32
    /visitor/qr_data                 std_msgs/String
    /nav_camera/obstacle_density     std_msgs/Float32
    /nav_camera/markers              std_msgs/String     (JSON)

Subscriptions
    /camera/enable_aruco             std_msgs/Bool       (toggle ArUco on/off)
"""

import sys
import os
import enum
import json
import queue
import threading
import time as _time

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Bool, Float32, Int32, String
from cv_bridge import CvBridge

import cv2
import numpy as np

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))
from config import (
    CAMERA_DEVICE,
    CAMERA_WIDTH,
    CAMERA_HEIGHT,
    CAMERA_FPS,
    NAV_VISION_HZ,
    INTERACTION_VISION_HZ,
    QR_SCAN_INTERVAL,
    ARUCO_ENABLED_DEFAULT,
    CAMERA_FRAME_TIMEOUT,
    CAMERA_RECONNECT_INTERVAL,
)
from core.vision_interaction import InteractionVision
from core.vision_navigation import NavigationVision


# --------------------------------------------------
#  Camera health states
# --------------------------------------------------

class _CameraState(enum.Enum):
    ONLINE = "ONLINE"
    OFFLINE = "OFFLINE"
    RECONNECTING = "RECONNECTING"


class CameraNode(Node):
    """Unified single-camera node with threaded vision workers.

    Owns the single Logitech C270 VideoCapture and distributes
    independent frame copies to navigation and interaction worker
    threads via thread-safe queues.
    """

    def __init__(self) -> None:
        super().__init__("camera_node")
        self.get_logger().info("CameraNode starting ...")

        # -- Vision processors (pure OpenCV, no camera ownership) --
        self._nav_vision = NavigationVision()
        self._interaction_vision = InteractionVision()
        self._bridge = CvBridge()

        # -- Camera state ------------------------------------------
        self._cam_state = _CameraState.OFFLINE
        self._last_frame_time: float = 0.0
        self._cap: cv2.VideoCapture = None  # type: ignore
        self._cap_lock = threading.Lock()

        # -- ArUco toggle ------------------------------------------
        self._aruco_enabled: bool = ARUCO_ENABLED_DEFAULT

        # -- Frame queues (producer -> consumer, maxsize=2) --------
        self._nav_queue: queue.Queue = queue.Queue(maxsize=2)
        self._interaction_queue: queue.Queue = queue.Queue(maxsize=2)

        # -- Latest frame for lazy image publishing ----------------
        self._latest_frame: np.ndarray = None  # type: ignore
        self._latest_frame_lock = threading.Lock()

        # -- Shutdown flag -----------------------------------------
        self._shutdown = threading.Event()

        # -- Publishers --------------------------------------------
        self._pub_image = self.create_publisher(
            Image, "/camera/image_raw", 10
        )
        self._pub_status = self.create_publisher(
            String, "/camera/status", 10
        )
        self._pub_detected = self.create_publisher(
            Bool, "/visitor/detected", 10
        )
        self._pub_face_count = self.create_publisher(
            Int32, "/visitor/face_count", 10
        )
        self._pub_qr = self.create_publisher(
            String, "/visitor/qr_data", 10
        )
        self._pub_density = self.create_publisher(
            Float32, "/nav_camera/obstacle_density", 10
        )
        self._pub_markers = self.create_publisher(
            String, "/nav_camera/markers", 10
        )

        # -- Subscribers -------------------------------------------
        self.create_subscription(
            Bool, "/camera/enable_aruco", self._aruco_toggle_cb, 10
        )

        # -- Open camera -------------------------------------------
        self._open_camera()

        # -- Start worker threads ----------------------------------
        self._cam_thread = threading.Thread(
            target=self._camera_loop, daemon=True, name="cam_capture"
        )
        self._nav_thread = threading.Thread(
            target=self._nav_worker_loop, daemon=True, name="nav_worker"
        )
        self._interaction_thread = threading.Thread(
            target=self._interaction_worker_loop, daemon=True,
            name="interaction_worker"
        )

        self._cam_thread.start()
        self._nav_thread.start()
        self._interaction_thread.start()

        # -- ROS timers --------------------------------------------
        # Health monitor at 1 Hz
        self._health_timer = self.create_timer(1.0, self._health_tick)
        # Lazy image publish at 5 Hz (only when subscribers exist)
        self._image_timer = self.create_timer(0.2, self._image_publish_tick)

        self.get_logger().info("CameraNode ready (3 threads started).")

    # ==================================================
    #  Camera Management
    # ==================================================

    def _open_camera(self) -> bool:
        """Attempt to open the Logitech C270.  Returns True on success."""
        with self._cap_lock:
            if self._cap is not None and self._cap.isOpened():
                self._cap.release()

            cap = cv2.VideoCapture(CAMERA_DEVICE, cv2.CAP_V4L2)
            if not cap.isOpened():
                # Fallback for non-Linux dev machines
                cap = cv2.VideoCapture(CAMERA_DEVICE)

            if cap.isOpened():
                cap.set(cv2.CAP_PROP_FRAME_WIDTH, CAMERA_WIDTH)
                cap.set(cv2.CAP_PROP_FRAME_HEIGHT, CAMERA_HEIGHT)
                cap.set(cv2.CAP_PROP_FPS, CAMERA_FPS)
                cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)  # minimise latency
                self._cap = cap
                self._cam_state = _CameraState.ONLINE
                self._last_frame_time = _time.monotonic()
                self.get_logger().info(
                    f"Logitech C270 opened: device={CAMERA_DEVICE}, "
                    f"{CAMERA_WIDTH}x{CAMERA_HEIGHT}@{CAMERA_FPS}fps"
                )
                return True
            else:
                self._cap = None
                self._cam_state = _CameraState.OFFLINE
                self.get_logger().error(
                    f"FAILED to open camera at device {CAMERA_DEVICE}."
                )
                return False

    # ==================================================
    #  Camera Capture Thread
    # ==================================================

    def _camera_loop(self) -> None:
        """Daemon thread: continuously grabs frames and pushes copies
        into worker queues.  Never blocks on queue full — drops oldest.
        """
        period = 1.0 / CAMERA_FPS

        while not self._shutdown.is_set():
            # -- Check camera availability -----------------------
            with self._cap_lock:
                cap = self._cap
                state = self._cam_state

            if cap is None or not cap.isOpened() or state != _CameraState.ONLINE:
                _time.sleep(0.1)
                continue

            # -- Grab frame --------------------------------------
            ret, frame = cap.read()

            if not ret or frame is None:
                _time.sleep(period)
                continue

            self._last_frame_time = _time.monotonic()

            # -- Push copies into worker queues ------------------
            self._enqueue(self._nav_queue, frame)
            self._enqueue(self._interaction_queue, frame)

            # -- Store latest frame for lazy image publish -------
            with self._latest_frame_lock:
                self._latest_frame = frame

            # -- Pace to target FPS ------------------------------
            _time.sleep(period)

    @staticmethod
    def _enqueue(q: queue.Queue, frame: np.ndarray) -> None:
        """Push a frame copy into *q*.  Drop oldest if full."""
        frame_copy = frame.copy()
        try:
            q.put_nowait(frame_copy)
        except queue.Full:
            try:
                q.get_nowait()  # discard oldest
            except queue.Empty:
                pass
            try:
                q.put_nowait(frame_copy)
            except queue.Full:
                pass  # both slots raced away — skip this frame

    # ==================================================
    #  Navigation Worker Thread
    # ==================================================

    def _nav_worker_loop(self) -> None:
        """Daemon thread: processes frames for obstacle density and
        optional ArUco marker detection at NAV_VISION_HZ.
        """
        period = 1.0 / NAV_VISION_HZ

        while not self._shutdown.is_set():
            try:
                frame = self._nav_queue.get(timeout=0.5)
            except queue.Empty:
                # No frame available — publish safe default
                density_msg = Float32()
                density_msg.data = 0.0
                self._pub_density.publish(density_msg)
                continue

            # -- Obstacle density (always runs) ------------------
            density = self._nav_vision.compute_obstacle_density(frame)
            density_msg = Float32()
            density_msg.data = density
            self._pub_density.publish(density_msg)

            # -- ArUco detection (on-demand) ---------------------
            markers_json = []
            if self._aruco_enabled:
                markers = self._nav_vision.detect_markers(frame)
                for m in markers:
                    corners = m["corners"]
                    cx = float(np.mean(corners[:, 0]))
                    cy = float(np.mean(corners[:, 1]))
                    markers_json.append({
                        "id": int(m["id"]),
                        "cx": round(cx, 1),
                        "cy": round(cy, 1),
                    })

                if markers_json:
                    self.get_logger().info(
                        f"Markers: {[m['id'] for m in markers_json]}",
                        throttle_duration_sec=2.0,
                    )

            markers_msg = String()
            markers_msg.data = json.dumps(markers_json)
            self._pub_markers.publish(markers_msg)

            # -- Pace to target rate -----------------------------
            _time.sleep(period)

    # ==================================================
    #  Interaction Worker Thread
    # ==================================================

    def _interaction_worker_loop(self) -> None:
        """Daemon thread: processes frames for face detection (at
        INTERACTION_VISION_HZ) and QR scanning (at QR_SCAN_INTERVAL).
        """
        period = 1.0 / INTERACTION_VISION_HZ
        last_qr_scan: float = 0.0

        while not self._shutdown.is_set():
            try:
                frame = self._interaction_queue.get(timeout=0.5)
            except queue.Empty:
                continue

            # -- Face detection (every tick = 5 Hz) ---------------
            faces = self._interaction_vision.detect_faces(frame)
            face_count = len(faces)

            detected_msg = Bool()
            detected_msg.data = face_count > 0
            self._pub_detected.publish(detected_msg)

            count_msg = Int32()
            count_msg.data = face_count
            self._pub_face_count.publish(count_msg)

            # -- QR scanning (throttled to QR_SCAN_INTERVAL) ------
            now = _time.monotonic()
            qr_data = None
            if now - last_qr_scan >= QR_SCAN_INTERVAL:
                last_qr_scan = now
                qr_data = self._interaction_vision.decode_qr(frame)

            qr_msg = String()
            qr_msg.data = qr_data if qr_data else ""
            self._pub_qr.publish(qr_msg)

            # -- Pace to target rate -----------------------------
            _time.sleep(period)

    # ==================================================
    #  ArUco Toggle Callback
    # ==================================================

    def _aruco_toggle_cb(self, msg: Bool) -> None:
        """Enable/disable ArUco detection at runtime."""
        old = self._aruco_enabled
        self._aruco_enabled = msg.data
        if old != msg.data:
            state_str = "ENABLED" if msg.data else "DISABLED"
            self.get_logger().info(f"ArUco detection {state_str}.")

    # ==================================================
    #  Health Monitor (1 Hz ROS timer)
    # ==================================================

    def _health_tick(self) -> None:
        """Monitor camera health and attempt reconnection if needed."""
        now = _time.monotonic()

        if self._cam_state == _CameraState.ONLINE:
            # Check for frame timeout
            if now - self._last_frame_time > CAMERA_FRAME_TIMEOUT:
                self.get_logger().warn(
                    "Camera frame timeout -- declaring OFFLINE."
                )
                self._cam_state = _CameraState.OFFLINE
                with self._cap_lock:
                    if self._cap is not None:
                        self._cap.release()
                        self._cap = None

        elif self._cam_state in (_CameraState.OFFLINE, _CameraState.RECONNECTING):
            self._cam_state = _CameraState.RECONNECTING
            self.get_logger().info(
                "Attempting camera reconnection ...",
                throttle_duration_sec=CAMERA_RECONNECT_INTERVAL,
            )
            if self._open_camera():
                self.get_logger().info("Camera reconnected -- ONLINE.")
            else:
                self._cam_state = _CameraState.RECONNECTING

        # Publish status
        status_msg = String()
        status_msg.data = self._cam_state.value
        self._pub_status.publish(status_msg)

    # ==================================================
    #  Lazy Image Publish (5 Hz ROS timer)
    # ==================================================

    def _image_publish_tick(self) -> None:
        """Publish raw image only when subscribers are connected."""
        if self._pub_image.get_subscription_count() == 0:
            return

        with self._latest_frame_lock:
            frame = self._latest_frame

        if frame is None:
            return

        try:
            img_msg = self._bridge.cv2_to_imgmsg(frame, encoding="bgr8")
            self._pub_image.publish(img_msg)
        except Exception as e:
            self.get_logger().warn(f"Image publish error: {e}")

    # ==================================================
    #  Cleanup
    # ==================================================

    def destroy_node(self) -> None:
        self.get_logger().info("CameraNode shutting down ...")
        self._shutdown.set()

        # Wait for threads to finish
        for t in (self._cam_thread, self._nav_thread, self._interaction_thread):
            t.join(timeout=2.0)

        with self._cap_lock:
            if self._cap is not None and self._cap.isOpened():
                self._cap.release()
                self.get_logger().info("Camera released.")

        super().destroy_node()


# --------------------------------------------------
#  Entry Point
# --------------------------------------------------

def main(args=None) -> None:
    rclpy.init(args=args)
    node = CameraNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
