"""
config.py -- Central configuration for the Autonomous Receptionist Robot.

All tuneable parameters are stored here as module-level constants
grouped by subsystem.  Every other module imports from this file:

    from config import (
        MAX_LINEAR_SPEED, LOOKAHEAD_DISTANCE, ...
    )
"""

from dataclasses import dataclass, field
from typing import List
import math


# ─────────────────────────────────────────────
#  Robot Physical Parameters
# ─────────────────────────────────────────────
ROBOT_RADIUS: float = 0.18          # metres — used for obstacle inflation
MAX_LINEAR_SPEED: float = 0.35      # m/s — absolute cap on vx, vy
MAX_ANGULAR_SPEED: float = 1.0      # rad/s — absolute cap on wz

# ─────────────────────────────────────────────
#  TF Frame IDs
# ─────────────────────────────────────────────
MAP_FRAME: str = "map"
ODOM_FRAME: str = "odom"
BASE_FRAME: str = "base_link"

# ─────────────────────────────────────────────
#  Global Planner (A*)
# ─────────────────────────────────────────────
GRID_RESOLUTION: float = 0.05       # metres per cell (matches SLAM default)
OBSTACLE_THRESHOLD: int = 65        # OccupancyGrid value ≥ this → blocked
UNKNOWN_AS_OBSTACLE: bool = True    # treat −1 cells as blocked
PLANNING_TIMEOUT: float = 5.0       # seconds — give up if A* takes longer
INFLATION_RADIUS_CELLS: int = 4     # inflate obstacles by this many cells

# ─────────────────────────────────────────────
#  Path Post-Processing
# ─────────────────────────────────────────────
PATH_MIN_POINT_DISTANCE: float = 0.03          # metres — prune closer points
PATH_SMOOTH_WEIGHT_DATA: float = 0.1           # how much to keep original path
PATH_SMOOTH_WEIGHT_SMOOTH: float = 0.3         # how much to smooth
PATH_SMOOTH_TOLERANCE: float = 0.00001         # convergence threshold
PATH_COLLINEAR_ANGLE_THRESHOLD: float = 0.15   # radians (~8.6°)

# ─────────────────────────────────────────────
#  Local Controller (Regulated Pure Pursuit – Omni)
# ─────────────────────────────────────────────
CONTROL_HZ: float = 20.0                          # control loop frequency
LOOKAHEAD_DISTANCE: float = 0.30                   # metres
GOAL_TOLERANCE: float = 0.08                       # metres — "arrived" radius
HEADING_KP: float = 1.5                            # proportional gain on yaw error
APPROACH_VELOCITY_SCALING: float = 0.5             # slow down inside this radius
REGULATE_MIN_SPEED: float = 0.05                   # never command less than this (unless stopping)
CROSS_TRACK_ERROR_THRESHOLD: float = 0.40          # metres — replan if exceeded

# ─────────────────────────────────────────────
#  Sensor Fusion
# ─────────────────────────────────────────────
NUM_IR_SENSORS: int = 8
NUM_ULTRASONIC_SENSORS: int = 8

IR_WEIGHT: float = 0.35
ULTRASONIC_WEIGHT: float = 0.40
CAMERA_WEIGHT: float = 0.25

DANGER_THRESHOLD: float = 0.70                # danger score ≥ this → emergency stop
OBSTACLE_STOP_DISTANCE: float = 0.12          # metres — full stop
OBSTACLE_SLOW_DISTANCE: float = 0.45          # metres — begin tapering speed

SENSOR_FUSION_HZ: float = 20.0               # publish rate

# IR / ultrasonic Range message defaults
IR_MIN_RANGE: float = 0.02
IR_MAX_RANGE: float = 0.80
ULTRASONIC_MIN_RANGE: float = 0.02
ULTRASONIC_MAX_RANGE: float = 4.0

# ─────────────────────────────────────────────
#  Camera (Logitech C270 USB -- single camera)
# ─────────────────────────────────────────────
CAMERA_DEVICE: int = 0                        # /dev/video0
CAMERA_WIDTH: int = 640
CAMERA_HEIGHT: int = 480
CAMERA_FPS: int = 15                          # capture rate

# Worker processing rates (Hz)
NAV_VISION_HZ: float = 10.0                   # obstacle density + ArUco
INTERACTION_VISION_HZ: float = 5.0            # face detection
QR_SCAN_INTERVAL: float = 0.5                 # seconds between QR attempts

# Obstacle density (Canny)
NAV_CAM_OBSTACLE_ROI_TOP_RATIO: float = 0.5   # only look at the bottom half
CANNY_LOW: int = 50
CANNY_HIGH: int = 150

# ArUco (on-demand -- disabled by default to save RPi4 CPU)
ARUCO_DICTIONARY: str = "DICT_4X4_50"         # cv2.aruco dictionary name
ARUCO_MARKER_SIZE: float = 0.10               # metres -- for pose estimation
ARUCO_ENABLED_DEFAULT: bool = False            # toggle via /camera/enable_aruco

# Face detection
FACE_DETECTION_MODEL: str = "haarcascade_frontalface_default"
FACE_MIN_SIZE: tuple = (60, 60)               # px -- ignore tiny detections

# Camera health monitoring
CAMERA_FRAME_TIMEOUT: float = 2.0             # seconds before declaring OFFLINE
CAMERA_RECONNECT_INTERVAL: float = 5.0        # seconds between reconnect attempts

# ─────────────────────────────────────────────
#  Receptionist Workflow
# ─────────────────────────────────────────────
GREETING_MESSAGE: str = "Welcome! Please show your QR code."
VISITOR_PRESENCE_COOLDOWN: float = 3.0         # seconds before re-triggering
NAMED_DESTINATIONS: dict = {
    "reception":       {"x": 0.0, "y": 0.0},
    "conference_room": {"x": 5.2, "y": 3.1},
    "hr_room":         {"x": 8.0, "y": 1.5},
    "manager_cabin":   {"x": 10.3, "y": 4.2},
}
QR_WAIT_TIMEOUT: float = 30.0                 # seconds to wait for QR
ARRIVAL_ANNOUNCE_DURATION: float = 2.0         # seconds to show arrival msg


# ─────────────────────────────────────────────
#  Recovery Behaviour
# ─────────────────────────────────────────────
MAX_RECOVERY_ATTEMPTS: int = 3
RECOVERY_ROTATION_SPEED: float = 0.4          # rad/s
RECOVERY_ROTATION_DURATION: float = 3.0       # seconds — one full sweep
RECOVERY_BACKUP_SPEED: float = -0.10          # m/s
RECOVERY_BACKUP_DURATION: float = 1.0         # seconds
RECOVERY_PAUSE_DURATION: float = 1.0          # seconds — wait after stop

# ─────────────────────────────────────────────
#  Nav2 Action Client
# ─────────────────────────────────────────────
NAV2_ACTION_NAME: str = "navigate_to_pose"
NAV2_ACTION_TIMEOUT: float = 300.0            # seconds

# ─────────────────────────────────────────────
#  SLAM Toolbox
# ─────────────────────────────────────────────
SLAM_MAP_TOPIC: str = "/map"
SLAM_SAVE_MAP_SERVICE: str = "/slam_toolbox/save_map"

# ─────────────────────────────────────────────
#  Replanning
# ─────────────────────────────────────────────
REPLAN_DANGER_SPIKE: float = 0.60             # danger score jump triggers replan
REPLAN_COOLDOWN: float = 2.0                  # seconds between replans

# ─────────────────────────────────────────────
#  LLM (Gemini)
# ─────────────────────────────────────────────
GEMINI_MODEL: str = "gemini-2.0-flash"        # fast + cheap model for greetings
GEMINI_MAX_RETRIES: int = 3                   # retry on transient API failures
GEMINI_TIMEOUT: float = 10.0                  # seconds per API call
LLM_RESPONSE_TIMEOUT: float = 10.0           # seconds receptionist waits for LLM
LLM_FALLBACK_GREETING: str = "Welcome! How can I help you today?"

# ─────────────────────────────────────────────
#  TTS (Text-to-Speech)
# ─────────────────────────────────────────────
TTS_RATE: int = 160                           # words per minute
TTS_VOLUME: float = 1.0                       # 0.0 to 1.0


