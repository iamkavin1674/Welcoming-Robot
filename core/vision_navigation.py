# -*- coding: utf-8 -*-
"""
core/vision_navigation.py — Pure OpenCV processing for the FIT0701 navigation camera.

Provides ArUco marker detection, single-marker pose estimation,
Canny-based obstacle-density scoring, and frame annotation.

This module has **no ROS imports** — it depends only on OpenCV, NumPy,
and the project-level ``config.py``.  It is consumed by ROS node wrappers
that feed it camera frames and forward results to the navigation stack.
"""

from __future__ import annotations

import os
import sys
from typing import Dict, List, Tuple

import cv2
import numpy as np

# ---------------------------------------------------------------------------
#  Project imports — ensure the repo root is on sys.path so ``config``
#  can be resolved regardless of the working directory.
# ---------------------------------------------------------------------------
sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))
from config import (                       # noqa: E402
    ARUCO_DICTIONARY,
    ARUCO_MARKER_SIZE,
    CANNY_LOW,
    CANNY_HIGH,
    NAV_CAM_OBSTACLE_ROI_TOP_RATIO,
)


class NavigationVision:
    """Computer-vision pipeline for the FIT0701 navigation camera.

    Responsibilities
    ----------------
    * **ArUco marker detection** — locate and identify fiducial markers in
      each frame using the dictionary specified in ``config.ARUCO_DICTIONARY``.
    * **Marker pose estimation** — given camera intrinsics, compute the 6-DoF
      pose of a detected marker (rotation + translation vectors).
    * **Obstacle density scoring** — run Canny edge detection on the bottom
      portion of the frame and return a normalised density score (0.0–1.0).
      This replicates (and supersedes) the camera scoring logic that was
      previously embedded inside ``core/fusion.py``.
    * **Frame annotation** — overlay detected marker outlines and IDs onto
      a copy of the frame for debugging / visualisation.

    All methods are pure functions of their inputs (plus the immutable config
    captured at construction time), making the class easy to unit-test.
    """

    # ------------------------------------------------------------------
    #  Construction
    # ------------------------------------------------------------------

    def __init__(self) -> None:
        """Initialise the ArUco detector and store Canny parameters.

        The ArUco dictionary is looked up by name from ``config.ARUCO_DICTIONARY``
        (e.g. ``"DICT_4X4_100"``).  A ``cv2.aruco.ArucoDetector`` is created
        with the default ``DetectorParameters`` — these can be fine-tuned later
        by mutating ``self._detector_params`` before calling :meth:`detect_markers`.

        Canny edge-detection thresholds and the ROI ratio are cached from
        ``config`` so the caller never needs to pass them explicitly.
        """
        # ArUco setup ─────────────────────────────────────────────────
        dictionary = cv2.aruco.getPredefinedDictionary(
            getattr(cv2.aruco, ARUCO_DICTIONARY)
        )
        self._detector_params = cv2.aruco.DetectorParameters()
        self._detector = cv2.aruco.ArucoDetector(
            dictionary, self._detector_params
        )

        # Canny / obstacle-density parameters ─────────────────────────
        self._canny_low: int = CANNY_LOW
        self._canny_high: int = CANNY_HIGH
        self._roi_top_ratio: float = NAV_CAM_OBSTACLE_ROI_TOP_RATIO

    # ------------------------------------------------------------------
    #  ArUco Detection
    # ------------------------------------------------------------------

    def detect_markers(self, frame: np.ndarray) -> List[Dict]:
        """Detect ArUco markers in *frame*.

        Parameters
        ----------
        frame : np.ndarray
            BGR or grayscale image captured from the FIT0701 camera.

        Returns
        -------
        list[dict]
            Each element is a dictionary with two keys:

            * ``'id'``  (int)          — the numeric marker ID.
            * ``'corners'`` (np.ndarray) — a (4, 2) float32 array of the
              four corner coordinates (pixels) in the image.

            An empty list is returned when no markers are found.

        Notes
        -----
        ``cv2.aruco.ArucoDetector.detectMarkers`` returns *corners* as a
        tuple of arrays shaped ``(1, 4, 2)`` — one array per marker.  This
        method squeezes the leading dimension for convenience.
        """
        corners, ids, _rejected = self._detector.detectMarkers(frame)

        if ids is None or len(ids) == 0:
            return []

        markers: List[Dict] = []
        for i, marker_id in enumerate(ids.flatten()):
            markers.append({
                "id": int(marker_id),
                "corners": corners[i].squeeze(),     # (4, 2)
            })
        return markers

    # ------------------------------------------------------------------
    #  Pose Estimation
    # ------------------------------------------------------------------

    def estimate_marker_pose(
        self,
        corners: np.ndarray,
        marker_size: float,
        camera_matrix: np.ndarray,
        dist_coeffs: np.ndarray,
    ) -> Tuple[np.ndarray, np.ndarray]:
        """Estimate the 6-DoF pose of a single ArUco marker.

        Parameters
        ----------
        corners : np.ndarray
            Corner array for **one** marker, shaped ``(4, 2)`` or
            ``(1, 4, 2)`` (both are accepted — the function will reshape
            if necessary).
        marker_size : float
            Physical side length of the marker in metres.
        camera_matrix : np.ndarray
            3×3 intrinsic camera matrix (focal lengths + principal point).
        dist_coeffs : np.ndarray
            Distortion coefficients (typically 5 or 8 elements).

        Returns
        -------
        tuple[np.ndarray, np.ndarray]
            ``(rvec, tvec)`` — the rotation vector (Rodrigues) and
            translation vector, each shaped ``(1, 1, 3)``.

        Raises
        ------
        ValueError
            If *corners* cannot be reshaped to ``(1, 4, 2)``.
        """
        # cv2.aruco.estimatePoseSingleMarkers expects (N, 4, 2)
        if corners.ndim == 2:
            corners = corners.reshape(1, 4, 2)

        rvecs, tvecs, _obj_points = cv2.aruco.estimatePoseSingleMarkers(
            corners, marker_size, camera_matrix, dist_coeffs
        )
        return rvecs, tvecs

    # ------------------------------------------------------------------
    #  Obstacle Density Scoring
    # ------------------------------------------------------------------

    def compute_obstacle_density(self, frame: np.ndarray) -> float:
        """Score obstacle presence via Canny edge density in the bottom ROI.

        This implements the same algorithm that was previously hard-coded in
        ``core/fusion.py → SensorFusion.update_camera``, extracted here so
        it can be reused and tested independently.

        Algorithm
        ---------
        1. Convert to grayscale (if colour).
        2. Apply Gaussian blur (5 × 5 kernel) to suppress noise.
        3. Run Canny edge detection with thresholds from ``config``.
        4. Crop to the bottom portion of the frame (controlled by
           ``NAV_CAM_OBSTACLE_ROI_TOP_RATIO`` — e.g. 0.5 keeps the bottom
           50 %).
        5. Compute ``edge_pixels / total_roi_pixels``.
        6. Normalise: divide by 0.30 and saturate at 1.0 (i.e. ≥ 30 %
           edge density ⇒ maximum score).

        Parameters
        ----------
        frame : np.ndarray
            BGR or grayscale image.

        Returns
        -------
        float
            Obstacle density score in ``[0.0, 1.0]``.  A value of 0.0 means
            no edges detected; 1.0 means the ROI is saturated with edges,
            indicating a very textured / close obstacle.
        """
        if frame is None or frame.size == 0:
            return 0.0

        # 1. Grayscale
        if len(frame.shape) == 3:
            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        else:
            gray = frame

        # 2. Gaussian blur
        blurred = cv2.GaussianBlur(gray, (5, 5), 0)

        # 3. Canny edge detection
        edges = cv2.Canny(blurred, self._canny_low, self._canny_high)

        # 4. Bottom-portion ROI
        h = edges.shape[0]
        roi_top = int(h * self._roi_top_ratio)
        roi = edges[roi_top:, :]

        # 5. Normalise edge density → [0.0, 1.0]
        total_pixels = roi.size
        if total_pixels == 0:
            return 0.0

        edge_count = int(np.count_nonzero(roi))
        raw_density = edge_count / total_pixels

        # 6. Saturate at 30 % edge density
        return min(raw_density / 0.30, 1.0)

    # ------------------------------------------------------------------
    #  Annotation / Visualisation
    # ------------------------------------------------------------------

    def annotate_frame(
        self,
        frame: np.ndarray,
        markers: List[Dict],
    ) -> np.ndarray:
        """Draw detected markers and their IDs onto a copy of *frame*.

        Parameters
        ----------
        frame : np.ndarray
            The original BGR image (will **not** be modified in place).
        markers : list[dict]
            Marker list as returned by :meth:`detect_markers`.

        Returns
        -------
        np.ndarray
            Annotated copy of the frame with marker outlines drawn in
            green and numeric IDs rendered next to each marker.
        """
        annotated = frame.copy()

        if not markers:
            return annotated

        # Rebuild the arrays that cv2.aruco.drawDetectedMarkers expects:
        #   corners — tuple of (1, 4, 2) arrays
        #   ids     — (N, 1) int32 array
        corners_list = tuple(
            m["corners"].reshape(1, 4, 2).astype(np.float32)
            for m in markers
        )
        ids_array = np.array(
            [[m["id"]] for m in markers], dtype=np.int32
        )

        cv2.aruco.drawDetectedMarkers(annotated, corners_list, ids_array)
        return annotated
