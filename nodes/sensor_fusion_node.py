"""
nodes/sensor_fusion_node.py — ROS 2 node that subscribes to raw sensor
topics (8 IR, 8 ultrasonic, 1 ESP32 camera), fuses them via
``core.fusion.SensorFusion``, and publishes danger score + min range.

Subscriptions
    /ir_sensor_0 … /ir_sensor_7         sensor_msgs/Range
    /ultrasonic_0 … /ultrasonic_7       sensor_msgs/Range
    /esp32_camera/image_raw              sensor_msgs/Image

Publications
    /obstacle/danger_score               std_msgs/Float32
    /obstacle/min_range                  std_msgs/Float32
    /obstacle/proximity_factor           std_msgs/Float32
"""

import sys
import os

# pyrefly: ignore [missing-import]
import rclpy
# pyrefly: ignore [missing-import]
from rclpy.node import Node
from sensor_msgs.msg import Range, Image
from std_msgs.msg import Float32
from cv_bridge import CvBridge

# Ensure project root is importable
sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))
from core.fusion import SensorFusion
from config import (
    NUM_IR_SENSORS,
    NUM_ULTRASONIC_SENSORS,
    SENSOR_FUSION_HZ,
    CAMERA_TOPIC,
)


class SensorFusionNode(Node):
    """Fuse IR + ultrasonic + camera into obstacle-awareness topics."""

    def __init__(self) -> None:
        super().__init__("sensor_fusion_node")
        self.get_logger().info("SensorFusionNode starting …")

        self._fusion = SensorFusion()
        self._bridge = CvBridge()

        # ── Publishers ──────────────────────────────
        self._pub_danger = self.create_publisher(Float32, "/obstacle/danger_score", 10)
        self._pub_min_range = self.create_publisher(Float32, "/obstacle/min_range", 10)
        self._pub_proximity = self.create_publisher(
            Float32, "/obstacle/proximity_factor", 10
        )

        # ── IR Subscribers (one per sensor) ─────────
        self._ir_subs = []
        for i in range(NUM_IR_SENSORS):
            topic = f"/ir_sensor_{i}"
            sub = self.create_subscription(
                Range,
                topic,
                lambda msg, idx=i: self._ir_callback(msg, idx),
                10,
            )
            self._ir_subs.append(sub)
            self.get_logger().info(f"  Subscribed to {topic}")

        # ── Ultrasonic Subscribers (one per sensor) ─
        self._us_subs = []
        for i in range(NUM_ULTRASONIC_SENSORS):
            topic = f"/ultrasonic_{i}"
            sub = self.create_subscription(
                Range,
                topic,
                lambda msg, idx=i: self._us_callback(msg, idx),
                10,
            )
            self._us_subs.append(sub)
            self.get_logger().info(f"  Subscribed to {topic}")

        # ── Camera Subscriber ──────────────────────
        self._cam_sub = self.create_subscription(
            Image,
            CAMERA_TOPIC,
            self._camera_callback,
            10,
        )
        self.get_logger().info(f"  Subscribed to {CAMERA_TOPIC}")

        # ── Timer — publish fused outputs at fixed rate ─
        period = 1.0 / SENSOR_FUSION_HZ
        self._timer = self.create_timer(period, self._publish_fused)

        self.get_logger().info("SensorFusionNode ready.")

    # ──────────────────────────────────────────
    #  Callbacks
    # ──────────────────────────────────────────

    def _ir_callback(self, msg: Range, sensor_index: int) -> None:
        self._fusion.update_ir(sensor_index, msg.range)

    def _us_callback(self, msg: Range, sensor_index: int) -> None:
        self._fusion.update_ultrasonic(sensor_index, msg.range)

    def _camera_callback(self, msg: Image) -> None:
        try:
            cv_image = self._bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
            self._fusion.update_camera(cv_image)
        except Exception as e:
            self.get_logger().warn(f"CvBridge error: {e}")

    # ──────────────────────────────────────────
    #  Publish Loop
    # ──────────────────────────────────────────

    def _publish_fused(self) -> None:
        """Publish the fused danger score, min range, and proximity factor."""
        danger_msg = Float32()
        danger_msg.data = self._fusion.get_danger_score()
        self._pub_danger.publish(danger_msg)

        min_range_msg = Float32()
        min_range_msg.data = self._fusion.get_min_range()
        self._pub_min_range.publish(min_range_msg)

        prox_msg = Float32()
        prox_msg.data = self._fusion.get_proximity_factor()
        self._pub_proximity.publish(prox_msg)


# ──────────────────────────────────────────────
#  Entry Point
# ──────────────────────────────────────────────

def main(args=None) -> None:
    rclpy.init(args=args)
    node = SensorFusionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
