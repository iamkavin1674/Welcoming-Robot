# -*- coding: utf-8 -*-
"""
core/tts_engine.py -- Pure Python text-to-speech engine.

This module handles ALL speech synthesis logic:
    - Initialise the TTS backend (pyttsx3 by default)
    - Convert text to speech
    - Play audio through the default output device
    - Block until utterance completes

Architecture Rules
    - This file contains ZERO ROS code.
    - It is called by nodes/tts_node.py, which owns the ROS pub/sub.
    - Swapping pyttsx3 for Piper (or any other engine) requires
      changing only this file.

Usage
    from core.tts_engine import TTSEngine

    engine = TTSEngine()
    engine.speak("Good afternoon! Welcome to our office.")
"""

import sys
import os
import logging

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))
from config import TTS_RATE, TTS_VOLUME

logger = logging.getLogger(__name__)


class TTSEngine:
    """Offline text-to-speech engine using pyttsx3.

    The engine blocks during ``speak()`` until the audio finishes
    playing, so the calling node knows exactly when speech is done.

    If pyttsx3 is unavailable the engine degrades gracefully —
    ``speak()`` logs the text and returns immediately.

    Attributes
    ----------
    _engine : pyttsx3.Engine or None
        The underlying pyttsx3 engine, or None if unavailable.
    _available : bool
        Whether the TTS backend initialised successfully.
    """

    def __init__(
        self,
        rate: int = TTS_RATE,
        volume: float = TTS_VOLUME,
    ) -> None:
        """Initialise the TTS engine.

        Parameters
        ----------
        rate : int
            Speech rate in words per minute.  Default from config.
        volume : float
            Output volume, 0.0 (silent) to 1.0 (maximum).
        """
        self._engine = None
        self._available: bool = False

        try:
            import pyttsx3

            self._engine = pyttsx3.init()
            self._engine.setProperty("rate", rate)
            self._engine.setProperty("volume", max(0.0, min(1.0, volume)))
            self._available = True
            logger.info(
                "TTSEngine initialised (rate=%d wpm, volume=%.1f).",
                rate, volume,
            )
        except ImportError:
            logger.warning(
                "pyttsx3 not installed. "
                "Run: pip install pyttsx3  — "
                "TTS will log text instead of speaking."
            )
        except Exception as exc:
            logger.warning(
                "Failed to initialise pyttsx3: %s — "
                "TTS will log text instead of speaking.",
                exc,
            )

    # --------------------------------------------------
    #  Public API
    # --------------------------------------------------

    def speak(self, text: str) -> None:
        """Convert text to speech and play through speakers.

        This method **blocks** until the utterance finishes.
        If the engine is unavailable, it logs the text and returns.

        Parameters
        ----------
        text : str
            The text to speak.  Empty strings are silently ignored.
        """
        if not text or not text.strip():
            return

        if not self._available or self._engine is None:
            logger.info("[TTS-fallback] Would speak: '%s'", text)
            return

        try:
            self._engine.say(text)
            self._engine.runAndWait()
            logger.info("[TTS] Spoke: '%s'", text)
        except Exception as exc:
            logger.error("TTS playback failed: %s", exc)

    def is_available(self) -> bool:
        """Check whether the TTS backend is ready.

        Returns
        -------
        bool
            True if pyttsx3 initialised successfully.
        """
        return self._available
