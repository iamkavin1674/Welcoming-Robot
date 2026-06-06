# -*- coding: utf-8 -*-
"""
nodes/navigation_node.py -- The main brain: global planner + local
controller + Nav2 action client integration.

State Machine
    IDLE -> PLANNING -> FOLLOWING -> GOAL_REACHED
                    -> RECOVERY ->

Subscriptions
    /map                         nav_msgs/OccupancyGrid
    /odom                        nav_msgs/Odometry
    /goal_pose                   geometry_msgs/PoseStamped
    /obstacle/danger_score       std_msgs/Float32
    /obstacle/min_range          std_msgs/Float32
    /obstacle/proximity_factor   std_msgs/Float32
    /recovery/status             std_msgs/String

Publications
    /cmd_vel_nav                 geometry_msgs/Twist   (via cmd_vel_mux)
    /planned_path                nav_msgs/Path
    /recovery/trigger            std_msgs/Bool

Action Client
    navigate_to_pose             nav2_msgs/NavigateToPose
"""

import sys
import os
import enum
import time

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient

from geometry_msgs.msg import Twist, PoseStamped
from nav_msgs.msg import OccupancyGrid, Odometry, Path
from std_msgs.msg import Float32, Bool, String

from tf2_ros import Buffer, TransformListener
import tf2_ros

import numpy as np
import math

# Nav2 action (optional — imported dynamically if available)
try:
    from nav2_msgs.action import NavigateToPose as Nav2NavigateToPose
    _HAS_NAV2 = True
except ImportError:
    _HAS_NAV2 = False

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))
from config import (
    MAP_FRAME,
    ODOM_FRAME,
    BASE_FRAME,
    OBSTACLE_THRESHOLD,
    UNKNOWN_AS_OBSTACLE,
    INFLATION_RADIUS_CELLS,
    CONTROL_HZ,
    GOAL_TOLERANCE,
    CROSS_TRACK_ERROR_THRESHOLD,
    DANGER_THRESHOLD,
    REPLAN_DANGER_SPIKE,
    REPLAN_COOLDOWN,
    NAV2_ACTION_NAME,
    NAV2_ACTION_TIMEOUT,
    PATH_MIN_POINT_DISTANCE,
    PATH_SMOOTH_WEIGHT_DATA,
    PATH_SMOOTH_WEIGHT_SMOOTH,
    PATH_SMOOTH_TOLERANCE,
    PATH_COLLINEAR_ANGLE_THRESHOLD,
    SLAM_SAVE_MAP_SERVICE,
    GRID_RESOLUTION,
)
from core.astar import astar
from core.omni_controller import OmniController
from utils.geometry import (
    euclidean_distance,
    quaternion_to_yaw,
    world_to_grid,
    grid_to_world,
)
from utils.grid import occupancy_to_matrix, mark_obstacles, inflate_obstacles
from utils.path import (
    prune_path,
    smooth_path,
    simplify_collinear,
    waypoints_to_path_msg,
)


class _State(enum.Enum):
    IDLE = 0
    PLANNING = 1
    FOLLOWING = 2
    GOAL_REACHED = 3
    RECOVERY = 4


class NavigationNode(Node):
    """Main navigation brain — planner + controller + state machine."""

    def __init__(self) -> None:
        super().__init__("navigation_node")
        self.get_logger().info("NavigationNode starting …")

        # ── State ──
        self._state = _State.IDLE
        self._controller = OmniController()

        # Map data (set by subscription)
        self._map_msg: OccupancyGrid | None = None
        self._blocked_grid: np.ndarray | None = None
        self._map_origin_x = 0.0
        self._map_origin_y = 0.0
        self._map_resolution = GRID_RESOLUTION
        self._map_width = 0
        self._map_height = 0

        # Odometry fallback
        self._odom_x = 0.0
        self._odom_y = 0.0
        self._odom_yaw = 0.0

        # Goal
        self._goal_x = 0.0
        self._goal_y = 0.0
        self._goal_pose_msg: PoseStamped | None = None

        # Path
        self._path_world: list = []    # list of (x, y) in metres
        self._path_msg: Path | None = None

        # Sensor fusion inputs
        self._danger_score = 0.0
        self._min_range = float("inf")
        self._proximity_factor = 1.0

        # Replan cooldown
        self._last_replan_time = 0.0

        # ── TF ──
        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)

        # ── Publishers ──
        self._pub_cmd = self.create_publisher(Twist, "/cmd_vel_nav", 10)
        self._pub_path = self.create_publisher(Path, "/planned_path", 10)
        self._pub_recovery_trigger = self.create_publisher(
            Bool, "/recovery/trigger", 10
        )

        # ── Subscribers ──
        self.create_subscription(OccupancyGrid, "/map", self._map_cb, 10)
        self.create_subscription(Odometry, "/odom", self._odom_cb, 10)
        self.create_subscription(PoseStamped, "/goal_pose", self._goal_cb, 10)
        self.create_subscription(
            Float32, "/obstacle/danger_score", self._danger_cb, 10
        )
        self.create_subscription(
            Float32, "/obstacle/min_range", self._min_range_cb, 10
        )
        self.create_subscription(
            Float32, "/obstacle/proximity_factor", self._proximity_cb, 10
        )
        self.create_subscription(
            String, "/recovery/status", self._recovery_status_cb, 10
        )

        # ── Nav2 Action Client ──
        self._nav2_client = None
        if _HAS_NAV2:
            self._nav2_client = ActionClient(
                self, Nav2NavigateToPose, NAV2_ACTION_NAME
            )
            self.get_logger().info(
                f"Nav2 action client created for '{NAV2_ACTION_NAME}'"
            )
        else:
            self.get_logger().warn(
                "nav2_msgs not found — Nav2 action client disabled."
            )

        # ── Control Timer ──
        period = 1.0 / CONTROL_HZ
        self._control_timer = self.create_timer(period, self._control_loop)

        self.get_logger().info("NavigationNode ready.")

    # ══════════════════════════════════════════
    #  Subscription Callbacks
    # ══════════════════════════════════════════

    def _map_cb(self, msg: OccupancyGrid) -> None:
        """Process incoming SLAM map."""
        self._map_msg = msg
        info = msg.info
        self._map_resolution = info.resolution
        self._map_width = info.width
        self._map_height = info.height
        self._map_origin_x = info.origin.position.x
        self._map_origin_y = info.origin.position.y

        # Pre-compute the inflated blocked grid
        grid = occupancy_to_matrix(list(msg.data), info.width, info.height)
        blocked = mark_obstacles(grid, OBSTACLE_THRESHOLD, UNKNOWN_AS_OBSTACLE)
        self._blocked_grid = inflate_obstacles(blocked, INFLATION_RADIUS_CELLS)

    def _odom_cb(self, msg: Odometry) -> None:
        """Fallback pose from odometry."""
        self._odom_x = msg.pose.pose.position.x
        self._odom_y = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        self._odom_yaw = quaternion_to_yaw(q.x, q.y, q.z, q.w)

    def _goal_cb(self, msg: PoseStamped) -> None:
        """Receive a new navigation goal."""
        self._goal_pose_msg = msg
        self._goal_x = msg.pose.position.x
        self._goal_y = msg.pose.position.y
        self.get_logger().info(
            f"New goal received: ({self._goal_x:.2f}, {self._goal_y:.2f})"
        )
        self._state = _State.PLANNING

    def _danger_cb(self, msg: Float32) -> None:
        self._danger_score = msg.data

    def _min_range_cb(self, msg: Float32) -> None:
        self._min_range = msg.data

    def _proximity_cb(self, msg: Float32) -> None:
        self._proximity_factor = msg.data

    def _recovery_status_cb(self, msg: String) -> None:
        if msg.data == "recovered":
            self.get_logger().info("Recovery succeeded — replanning.")
            self._state = _State.PLANNING
        elif msg.data == "failed":
            self.get_logger().error("Recovery failed — aborting goal.")
            self._state = _State.IDLE

    # ══════════════════════════════════════════
    #  Pose from TF (preferred) or Odom (fallback)
    # ══════════════════════════════════════════

    def _get_robot_pose(self):
        """
        Return (x, y, yaw) of the robot in the map frame.
        Tries TF first; falls back to odometry.
        """
        try:
            t = self._tf_buffer.lookup_transform(
                MAP_FRAME, BASE_FRAME, rclpy.time.Time()
            )
            x = t.transform.translation.x
            y = t.transform.translation.y
            q = t.transform.rotation
            yaw = quaternion_to_yaw(q.x, q.y, q.z, q.w)
            return x, y, yaw
        except (
            tf2_ros.LookupException,
            tf2_ros.ConnectivityException,
            tf2_ros.ExtrapolationException,
        ):
            return self._odom_x, self._odom_y, self._odom_yaw

    # ══════════════════════════════════════════
    #  Global Planning
    # ══════════════════════════════════════════

    def _plan_path(self) -> bool:
        """
        Run A* and post-process the path.

        Returns True on success, False on failure.
        """
        if self._blocked_grid is None:
            self.get_logger().warn("No map received yet — cannot plan.")
            return False

        rx, ry, _ = self._get_robot_pose()

        start_cell = world_to_grid(
            rx, ry, self._map_origin_x, self._map_origin_y, self._map_resolution
        )
        goal_cell = world_to_grid(
            self._goal_x,
            self._goal_y,
            self._map_origin_x,
            self._map_origin_y,
            self._map_resolution,
        )

        self.get_logger().info(
            f"Planning: start_cell={start_cell}, goal_cell={goal_cell}"
        )

        cell_path = astar(self._blocked_grid, start_cell, goal_cell)
        if cell_path is None:
            self.get_logger().warn("A* returned no path.")
            return False

        # Convert cell path → world coordinates
        world_path = [
            grid_to_world(
                cx, cy,
                self._map_origin_x, self._map_origin_y, self._map_resolution,
            )
            for cx, cy in cell_path
        ]

        # Post-process
        world_path = prune_path(world_path, PATH_MIN_POINT_DISTANCE)
        world_path = smooth_path(
            world_path, PATH_SMOOTH_WEIGHT_DATA,
            PATH_SMOOTH_WEIGHT_SMOOTH, PATH_SMOOTH_TOLERANCE,
        )
        world_path = simplify_collinear(
            world_path, PATH_COLLINEAR_ANGLE_THRESHOLD
        )

        if len(world_path) < 2:
            self.get_logger().warn("Path too short after processing.")
            return False

        self._path_world = world_path

        # Build and publish the ROS Path message
        stamp = self.get_clock().now().to_msg()
        self._path_msg = waypoints_to_path_msg(world_path, MAP_FRAME, stamp)
        self._pub_path.publish(self._path_msg)

        self.get_logger().info(
            f"Path planned: {len(world_path)} waypoints."
        )
        self._last_replan_time = time.monotonic()
        return True

    # ══════════════════════════════════════════
    #  Nav2 Action Client
    # ══════════════════════════════════════════

    def send_nav2_goal(self, pose_msg: PoseStamped) -> None:
        """
        Send a goal to Nav2's NavigateToPose action server.

        This is an alternative to the built-in planner+controller.
        Call this method to hand off navigation to the full Nav2 stack.
        """
        if self._nav2_client is None:
            self.get_logger().warn("Nav2 action client not available.")
            return

        if not self._nav2_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error("Nav2 action server not available.")
            return

        goal_msg = Nav2NavigateToPose.Goal()
        goal_msg.pose = pose_msg

        self.get_logger().info("Sending goal to Nav2 …")
        future = self._nav2_client.send_goal_async(
            goal_msg, feedback_callback=self._nav2_feedback_cb
        )
        future.add_done_callback(self._nav2_goal_response_cb)

    def _nav2_goal_response_cb(self, future) -> None:
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().warn("Nav2 goal rejected.")
            return
        self.get_logger().info("Nav2 goal accepted.")
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self._nav2_result_cb)

    def _nav2_feedback_cb(self, feedback_msg) -> None:
        fb = feedback_msg.feedback
        self.get_logger().info(
            f"Nav2 feedback — distance remaining: "
            f"{fb.distance_remaining:.2f} m",
            throttle_duration_sec=2.0,
        )

    def _nav2_result_cb(self, future) -> None:
        result = future.result()
        if result.status == 4:  # SUCCEEDED
            self.get_logger().info("Nav2 goal succeeded.")
        else:
            self.get_logger().warn(f"Nav2 goal ended with status {result.status}.")

    # ══════════════════════════════════════════
    #  Control Loop (20 Hz timer)
    # ══════════════════════════════════════════

    def _control_loop(self) -> None:
        """State-machine tick executed at CONTROL_HZ."""

        # ─── IDLE ───
        if self._state == _State.IDLE:
            return

        # ─── PLANNING ───
        if self._state == _State.PLANNING:
            success = self._plan_path()
            if success:
                self._state = _State.FOLLOWING
            else:
                self.get_logger().warn("Planning failed → entering recovery.")
                self._trigger_recovery()
            return

        # ─── FOLLOWING ───
        if self._state == _State.FOLLOWING:
            self._follow_path()
            return

        # ─── GOAL_REACHED ───
        if self._state == _State.GOAL_REACHED:
            self._pub_cmd.publish(Twist())  # zero velocity
            self.get_logger().info("Goal reached. Returning to IDLE.")
            self._state = _State.IDLE
            return

        # ─── RECOVERY ───
        if self._state == _State.RECOVERY:
            # Waiting for /recovery/status callback to transition out
            return

    # ──────────────────────────────────────────
    #  Path Following
    # ──────────────────────────────────────────

    def _follow_path(self) -> None:
        rx, ry, ryaw = self._get_robot_pose()

        # ── Goal check ──
        if self._controller.is_goal_reached(
            rx, ry, self._goal_x, self._goal_y
        ):
            self._pub_cmd.publish(Twist())
            self.get_logger().info(
                f"Goal reached at ({rx:.2f}, {ry:.2f})."
            )
            self._state = _State.GOAL_REACHED
            return

        # ── Emergency stop ──
        if self._danger_score >= DANGER_THRESHOLD:
            self._pub_cmd.publish(Twist())
            self.get_logger().warn("DANGER — emergency stop!")
            self._try_replan()
            return

        # ── Lookahead ──
        target, idx = self._controller.find_lookahead_point(
            rx, ry, self._path_world
        )
        if target is None:
            self.get_logger().warn("No lookahead point — replanning.")
            self._try_replan()
            return

        # ── Cross-track error check ──
        cte = self._cross_track_error(rx, ry)
        if cte > CROSS_TRACK_ERROR_THRESHOLD:
            self.get_logger().warn(
                f"Cross-track error {cte:.2f} m > threshold — replanning."
            )
            self._try_replan()
            return

        # ── Compute and publish velocity ──
        twist = self._controller.compute_velocity(
            rx, ry, ryaw,
            target[0], target[1],
            self._goal_x, self._goal_y,
            self._proximity_factor,
        )
        self._pub_cmd.publish(twist)

    def _cross_track_error(self, rx: float, ry: float) -> float:
        """Minimum distance from robot to any point on the path."""
        if not self._path_world:
            return 0.0
        return min(
            euclidean_distance(rx, ry, px, py)
            for px, py in self._path_world
        )

    # ──────────────────────────────────────────
    #  Replanning & Recovery
    # ──────────────────────────────────────────

    def _try_replan(self) -> None:
        """Replan if cooldown allows, else trigger recovery."""
        now = time.monotonic()
        if now - self._last_replan_time < REPLAN_COOLDOWN:
            return
        success = self._plan_path()
        if not success:
            self._trigger_recovery()

    def _trigger_recovery(self) -> None:
        self.get_logger().warn("Entering RECOVERY state.")
        self._state = _State.RECOVERY
        self._pub_cmd.publish(Twist())  # stop immediately
        trigger = Bool()
        trigger.data = True
        self._pub_recovery_trigger.publish(trigger)

    # ══════════════════════════════════════════
    #  Map Saving (optional — on goal reached)
    # ══════════════════════════════════════════

    def save_slam_map(self, map_name: str = "robot_map") -> None:
        """
        Call slam_toolbox's save_map service (if available).
        This is a convenience method for step 10.
        """
        from slam_toolbox.srv import SaveMap

        client = self.create_client(SaveMap, SLAM_SAVE_MAP_SERVICE)
        if not client.wait_for_service(timeout_sec=3.0):
            self.get_logger().warn("slam_toolbox save_map service not available.")
            return

        request = SaveMap.Request()
        request.name.data = map_name
        future = client.call_async(request)
        self.get_logger().info(f"Map save requested as '{map_name}'.")


# ──────────────────────────────────────────────
#  Entry Point
# ──────────────────────────────────────────────

def main(args=None) -> None:
    rclpy.init(args=args)
    node = NavigationNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
