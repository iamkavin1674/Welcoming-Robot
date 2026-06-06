# -*- coding: utf-8 -*-
"""
vision_interaction.py — Pure OpenCV processing for the OV2710 interaction camera.

This module handles real-time computer-vision tasks required by the
autonomous receptionist robot's front-facing OV2710 wide-angle camera:

  • **Face detection** using the Haar cascade classifier to identify
    visitors approaching the reception desk.
  • **QR code scanning** for badge / appointment look-ups so the robot
    can greet visitors by name or direct them to the correct room.
  • **Frame annotation** for live debugging and on-screen display.

Design constraints
──────────────────
* **No ROS 2 imports** — this file is a pure OpenCV + NumPy library so
  it can be unit-tested and profiled without a running ROS 2 graph.
* All tuneable parameters are pulled from the project-wide ``config.py``
  (``FACE_MIN_SIZE``, ``QR_SCAN_INTERVAL``).
* Thread-safety is *not* guaranteed; callers must serialise access if
  the same ``InteractionVision`` instance is shared across threads.
"""

from __future__ import annotations

import os
import sys
from typing import List, Optional, Tuple

import cv2
import numpy as np

# ---------------------------------------------------------------------------
#  Ensure project root is importable so ``from config import …`` works
#  regardless of the working directory or how the module is launched.
# ---------------------------------------------------------------------------
sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))

from config import FACE_MIN_SIZE, QR_SCAN_INTERVAL  # noqa: E402


class InteractionVision:
    """Computer-vision pipeline for the OV2710 interaction camera.

    Provides lightweight, single-frame methods for face detection, QR
    code decoding, and frame annotation.  Designed to run at camera
    frame-rate on a Raspberry Pi 5 without GPU acceleration.

    Attributes
    ----------
    face_cascade : cv2.CascadeClassifier
        Pre-trained Haar cascade for frontal-face detection.
    qr_detector : cv2.QRCodeDetector
        OpenCV QR-code detector / decoder instance.
    face_min_size : int
        Minimum width **and** height (pixels) passed to
        ``detectMultiScale`` so tiny false-positives are ignored.
    qr_scan_interval : float
        Minimum seconds between successive QR decode attempts.
        Useful when the caller wants to throttle CPU-heavy decoding.
    """

    # ── Annotation styling constants ──────────────────────────────────
    _FACE_RECT_COLOUR: Tuple[int, int, int] = (0, 255, 0)   # BGR green
    _FACE_RECT_THICKNESS: int = 2
    _QR_TEXT_COLOUR: Tuple[int, int, int] = (0, 255, 255)    # BGR yellow
    _QR_TEXT_FONT: int = cv2.FONT_HERSHEY_SIMPLEX
    _QR_TEXT_SCALE: float = 0.7
    _QR_TEXT_THICKNESS: int = 2
    _QR_TEXT_ORIGIN: Tuple[int, int] = (10, 30)

    def __init__(self) -> None:
        """Initialise the vision pipeline.

        Loads the Haar frontal-face cascade shipped with OpenCV and
        creates a ``QRCodeDetector``.  Configuration values
        ``FACE_MIN_SIZE`` and ``QR_SCAN_INTERVAL`` are read from
        ``config.py`` at construction time.

        Raises
        ------
        cv2.error
            If the Haar cascade XML cannot be loaded (e.g. OpenCV was
            built without the ``data`` package).
        """
        # -- Face detection ---------------------------------------------------
        cascade_path: str = (
            cv2.data.haarcascades + "haarcascade_frontalface_default.xml"
        )
        self.face_cascade: cv2.CascadeClassifier = cv2.CascadeClassifier(
            cascade_path
        )
        if self.face_cascade.empty():
            raise cv2.error(
                f"Failed to load Haar cascade from {cascade_path!r}"
            )

        # -- QR code detection ------------------------------------------------
        self.qr_detector: cv2.QRCodeDetector = cv2.QRCodeDetector()

        # -- Config values ----------------------------------------------------
        self.face_min_size: tuple = FACE_MIN_SIZE
        self.qr_scan_interval: float = QR_SCAN_INTERVAL

    # -----------------------------------------------------------------
    #  Face detection
    # -----------------------------------------------------------------
    def detect_faces(self, frame: np.ndarray) -> List[Tuple[int, int, int, int]]:
        """Detect frontal faces in *frame* using the Haar cascade.

        The input frame is converted to greyscale and histogram-
        equalised before detection to improve robustness under varying
        lighting conditions (common in lobby / reception environments).

        Parameters
        ----------
        frame : numpy.ndarray
            BGR image as returned by ``cv2.VideoCapture.read()``.

        Returns
        -------
        list of (int, int, int, int)
            Each element is an ``(x, y, w, h)`` bounding box in pixel
            coordinates.  Returns an empty list when no faces are found.
        """
        grey: np.ndarray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        grey = cv2.equalizeHist(grey)

        detections: np.ndarray = self.face_cascade.detectMultiScale(
            grey,
            scaleFactor=1.1,
            minNeighbors=5,
            minSize=self.face_min_size,
        )

        # detectMultiScale returns an empty tuple when nothing is found.
        if len(detections) == 0:
            return []

        return [
            (int(x), int(y), int(w), int(h))
            for (x, y, w, h) in detections
        ]

    # -----------------------------------------------------------------
    #  Visitor presence check
    # -----------------------------------------------------------------
    def is_visitor_present(self, frame: np.ndarray) -> bool:
        """Return ``True`` when at least one face is detected.

        A thin convenience wrapper around :meth:`detect_faces` for use
        in state-machine transitions (e.g. idle → greeting).

        Parameters
        ----------
        frame : numpy.ndarray
            BGR image from the OV2710 camera.

        Returns
        -------
        bool
            ``True`` if one or more faces are detected, ``False``
            otherwise.
        """
        return len(self.detect_faces(frame)) >= 1

    # -----------------------------------------------------------------
    #  QR code decoding
    # -----------------------------------------------------------------
    def decode_qr(self, frame: np.ndarray) -> Optional[str]:
        """Attempt to detect and decode a QR code in *frame*.

        Uses ``cv2.QRCodeDetector.detectAndDecode`` which returns the
        decoded UTF-8 payload when a valid QR code is visible.

        Parameters
        ----------
        frame : numpy.ndarray
            BGR image from the OV2710 camera.

        Returns
        -------
        str or None
            The decoded QR string, or ``None`` if no QR code was found
            or the decoded payload is an empty string.

        Notes
        -----
        This method does **not** enforce ``qr_scan_interval`` itself —
        throttling is the caller's responsibility so the module stays
        stateless with respect to time.
        """
        try:
            data, bbox, straight_qr = self.qr_detector.detectAndDecode(frame)
        except cv2.error:
            # Malformed frame or detector hiccup — treat as "not found".
            return None

        if data and len(data.strip()) > 0:
            return data.strip()

        return None

    # -----------------------------------------------------------------
    #  Frame annotation
    # -----------------------------------------------------------------
    def annotate_frame(
        self,
        frame: np.ndarray,
        faces: List[Tuple[int, int, int, int]],
        qr_data: Optional[str] = None,
    ) -> np.ndarray:
        """Return a copy of *frame* with detection overlays drawn.

        Draws:
        * A green rectangle around every face bounding box.
        * A yellow text overlay in the top-left corner showing the
          decoded QR payload (if any).

        The original frame is **not** mutated.

        Parameters
        ----------
        frame : numpy.ndarray
            BGR image to annotate.
        faces : list of (int, int, int, int)
            Face bounding boxes ``(x, y, w, h)`` as returned by
            :meth:`detect_faces`.
        qr_data : str or None, optional
            Decoded QR string to overlay.  ``None`` (default) means no
            QR text is drawn.

        Returns
        -------
        numpy.ndarray
            Annotated copy of *frame*.
        """
        annotated: np.ndarray = frame.copy()

        # -- Face rectangles --------------------------------------------------
        for (x, y, w, h) in faces:
            cv2.rectangle(
                annotated,
                pt1=(x, y),
                pt2=(x + w, y + h),
                color=self._FACE_RECT_COLOUR,
                thickness=self._FACE_RECT_THICKNESS,
            )

        # -- QR data overlay --------------------------------------------------
        if qr_data is not None:
            cv2.putText(
                annotated,
                f"QR: {qr_data}",
                org=self._QR_TEXT_ORIGIN,
                fontFace=self._QR_TEXT_FONT,
                fontScale=self._QR_TEXT_SCALE,
                color=self._QR_TEXT_COLOUR,
                thickness=self._QR_TEXT_THICKNESS,
                lineType=cv2.LINE_AA,
            )

        return annotated
