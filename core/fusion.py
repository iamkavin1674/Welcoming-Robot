# -*- coding: utf-8 -*-
"""
core/fusion.py -- Sensor fusion logic for IR + ultrasonic + navigation camera.

Combines readings from 8 IR rangers, 8 ultrasonic rangers, and the
navigation camera's obstacle density score into a unified danger score
and proximity-based speed multiplier.

No ROS node code here -- just data processing.
"""

import math
from typing import List, Optional

import numpy as np

import sys
import os

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))
from config import (
    NUM_IR_SENSORS,
    NUM_ULTRASONIC_SENSORS,
    IR_WEIGHT,
    ULTRASONIC_WEIGHT,
    CAMERA_WEIGHT,
    IR_MIN_RANGE,
    IR_MAX_RANGE,
    ULTRASONIC_MIN_RANGE,
    ULTRASONIC_MAX_RANGE,
    DANGER_THRESHOLD,
    OBSTACLE_STOP_DISTANCE,
    OBSTACLE_SLOW_DISTANCE,
)


class SensorFusion:
    """
    Fuses IR, ultrasonic, and navigation camera data into
    obstacle-awareness metrics consumed by the navigation controller.
    """

    def __init__(self) -> None:
        # Latest range readings (metres).  Initialised to max range
        # (= "nothing detected") so the robot doesn't freeze on boot.
        self._ir_ranges: List[float] = [IR_MAX_RANGE] * NUM_IR_SENSORS
        self._us_ranges: List[float] = [ULTRASONIC_MAX_RANGE] * NUM_ULTRASONIC_SENSORS

        # Navigation camera obstacle density (0.0 - 1.0)
        # Received as a pre-computed score from the nav camera node.
        self._camera_score: float = 0.0

    # ------------------------------------------
    #  Update Methods
    # ------------------------------------------

    def update_ir(self, sensor_index: int, range_m: float) -> None:
        """
        Store a single IR reading.

        Parameters
        ----------
        sensor_index : 0-based index of the IR sensor.
        range_m : measured distance in metres.
        """
        if 0 <= sensor_index < NUM_IR_SENSORS:
            clamped = max(IR_MIN_RANGE, min(range_m, IR_MAX_RANGE))
            self._ir_ranges[sensor_index] = clamped

    def update_ultrasonic(self, sensor_index: int, range_m: float) -> None:
        """
        Store a single ultrasonic reading.

        Parameters
        ----------
        sensor_index : 0-based index of the ultrasonic sensor.
        range_m : measured distance in metres.
        """
        if 0 <= sensor_index < NUM_ULTRASONIC_SENSORS:
            clamped = max(ULTRASONIC_MIN_RANGE, min(range_m, ULTRASONIC_MAX_RANGE))
            self._us_ranges[sensor_index] = clamped

    def update_nav_camera_score(self, score: float) -> None:
        """
        Accept pre-computed obstacle density from the navigation camera node.

        The nav camera node (FIT0701) runs Canny edge detection and
        computes a 0.0-1.0 obstacle density score. This method stores
        that score for use in the danger score calculation.

        Parameters
        ----------
        score : obstacle density, 0.0 = clear, 1.0 = dense obstacles.
        """
        self._camera_score = max(0.0, min(1.0, score))

    # ------------------------------------------
    #  Query Methods
    # ------------------------------------------

    def get_danger_score(self) -> float:
        """
        Weighted combination of all sensor sources.

        Returns a value in [0.0, 1.0]:
        - 0.0 = all clear
        - >= DANGER_THRESHOLD = imminent collision / emergency stop
        """
        # IR danger: normalise min reading against range window
        ir_min = min(self._ir_ranges)
        ir_danger = 1.0 - _range_normalise(
            ir_min, OBSTACLE_STOP_DISTANCE, IR_MAX_RANGE
        )

        # Ultrasonic danger
        us_min = min(self._us_ranges)
        us_danger = 1.0 - _range_normalise(
            us_min, OBSTACLE_STOP_DISTANCE, ULTRASONIC_MAX_RANGE
        )

        # Weighted sum (camera score comes from nav camera node)
        score = (
            IR_WEIGHT * ir_danger
            + ULTRASONIC_WEIGHT * us_danger
            + CAMERA_WEIGHT * self._camera_score
        )
        return max(0.0, min(1.0, score))

    def get_min_range(self) -> float:
        """Return the smallest distance across all range sensors."""
        return min(min(self._ir_ranges), min(self._us_ranges))

    def get_proximity_factor(self) -> float:
        """
        Speed multiplier based on the nearest obstacle.

        Returns
        -------
        float in [0.0, 1.0]:
            1.0 = clear, full speed allowed.
            0.0 = obstacle at stop distance, must halt.

        The factor linearly tapers between ``OBSTACLE_SLOW_DISTANCE``
        and ``OBSTACLE_STOP_DISTANCE``.
        """
        nearest = self.get_min_range()

        if nearest >= OBSTACLE_SLOW_DISTANCE:
            return 1.0
        if nearest <= OBSTACLE_STOP_DISTANCE:
            return 0.0

        # Linear interpolation
        return (nearest - OBSTACLE_STOP_DISTANCE) / (
            OBSTACLE_SLOW_DISTANCE - OBSTACLE_STOP_DISTANCE
        )

    def is_emergency(self) -> bool:
        """True when the danger score exceeds the threshold."""
        return self.get_danger_score() >= DANGER_THRESHOLD


# ------------------------------------------
#  Private Helpers
# ------------------------------------------

def _range_normalise(value: float, low: float, high: float) -> float:
    """
    Normalise *value* into [0.0, 1.0] given *low* and *high* bounds.
    Values below *low* = 0.0;  values above *high* = 1.0.
    """
    if high <= low:
        return 0.0
    return max(0.0, min(1.0, (value - low) / (high - low)))
