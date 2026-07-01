# -*- coding: utf-8 -*-
"""
core/gemini_interface.py -- Pure Python interface to Google Gemini.

This module handles ALL Gemini-related logic:
    - Prompt construction (time-aware receptionist greetings)
    - API communication via the google-generativeai SDK
    - Response parsing and sanitisation
    - Retry logic with exponential backoff
    - Error handling

Architecture Rules
    - This file contains ZERO ROS code.
    - It is called by nodes/llm_node.py, which owns the ROS pub/sub.
    - Swapping Gemini for another LLM requires changing only this file.
    - Future MCP transport can be added here without touching any node.

Usage
    from core.gemini_interface import GeminiInterface

    gemini = GeminiInterface()
    greeting = gemini.generate_greeting("14:15")
"""

import os
import sys
import time
import logging

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))
from config import (
    GEMINI_MODEL,
    GEMINI_MAX_RETRIES,
    GEMINI_TIMEOUT,
    LLM_FALLBACK_GREETING,
)

logger = logging.getLogger(__name__)


# --------------------------------------------------
#  Custom Exception
# --------------------------------------------------

class GeminiError(Exception):
    """Raised when Gemini API call fails after all retries."""
    pass


# --------------------------------------------------
#  Prompt Template
# --------------------------------------------------

_GREETING_PROMPT_TEMPLATE = """\
A visitor has just been detected at the reception area.

Current time: {time}

Generate ONE receptionist greeting.

Requirements:
- Friendly and professional
- Short — maximum 15 words
- Mention the appropriate time of day (morning, afternoon, or evening)
- Return ONLY the greeting text, nothing else
"""


# --------------------------------------------------
#  Gemini Interface
# --------------------------------------------------

class GeminiInterface:
    """Pure Python interface to Google Gemini for generating greetings.

    Attributes
    ----------
    _model : GenerativeModel
        The configured Gemini model instance.
    _max_retries : int
        Number of retry attempts on transient failures.
    _fallback : str
        Greeting returned when Gemini is unavailable.
    """

    def __init__(self, api_key: str = None) -> None:
        """Initialise the Gemini interface.

        Parameters
        ----------
        api_key : str, optional
            Google AI API key.  If not provided, reads from the
            ``GEMINI_API_KEY`` environment variable.

        Raises
        ------
        GeminiError
            If no API key is available.
        """
        self._max_retries: int = GEMINI_MAX_RETRIES
        self._fallback: str = LLM_FALLBACK_GREETING
        self._model = None

        # Resolve API key
        key: str = api_key or os.environ.get("GEMINI_API_KEY", "")
        if not key:
            logger.warning(
                "GEMINI_API_KEY not set. "
                "Gemini calls will return the fallback greeting."
            )
            return

        # Lazy import — avoids ImportError if SDK not installed yet
        try:
            import google.generativeai as genai

            genai.configure(api_key=key)
            self._model = genai.GenerativeModel(GEMINI_MODEL)
            logger.info(
                "GeminiInterface initialised with model '%s'.",
                GEMINI_MODEL,
            )
        except ImportError:
            logger.error(
                "google-generativeai package not installed. "
                "Run: pip install google-generativeai"
            )
        except Exception as exc:
            logger.error("Failed to initialise Gemini: %s", exc)

    # --------------------------------------------------
    #  Public API
    # --------------------------------------------------

    def generate_greeting(self, time_str: str) -> str:
        """Generate a time-aware receptionist greeting.

        Parameters
        ----------
        time_str : str
            Current time in ``HH:MM`` format (24-hour).

        Returns
        -------
        str
            The generated greeting, or the fallback string on failure.
        """
        if self._model is None:
            logger.info("Gemini unavailable — using fallback greeting.")
            return self._fallback

        prompt: str = self._build_prompt(time_str)
        return self._call_with_retries(prompt)

    # --------------------------------------------------
    #  Prompt Construction
    # --------------------------------------------------

    @staticmethod
    def _build_prompt(time_str: str) -> str:
        """Build the greeting prompt with the current time inserted.

        Parameters
        ----------
        time_str : str
            Current time as ``HH:MM``.

        Returns
        -------
        str
            The complete prompt string.
        """
        return _GREETING_PROMPT_TEMPLATE.format(time=time_str)

    # --------------------------------------------------
    #  API Call with Retries
    # --------------------------------------------------

    def _call_with_retries(self, prompt: str) -> str:
        """Call Gemini with exponential-backoff retries.

        Parameters
        ----------
        prompt : str
            The prompt to send.

        Returns
        -------
        str
            The model's response text, or fallback on failure.
        """
        for attempt in range(1, self._max_retries + 1):
            try:
                response = self._model.generate_content(
                    prompt,
                    generation_config={
                        "max_output_tokens": 60,
                        "temperature": 0.7,
                    },
                )
                text: str = self._parse_response(response)
                if text:
                    logger.info(
                        "Gemini greeting (attempt %d): %s",
                        attempt, text,
                    )
                    return text

            except Exception as exc:
                wait: float = 2 ** attempt  # 2, 4, 8 seconds
                logger.warning(
                    "Gemini attempt %d/%d failed: %s. "
                    "Retrying in %.0fs ...",
                    attempt, self._max_retries, exc, wait,
                )
                time.sleep(wait)

        logger.error(
            "All %d Gemini attempts failed — using fallback.",
            self._max_retries,
        )
        return self._fallback

    # --------------------------------------------------
    #  Response Parsing
    # --------------------------------------------------

    @staticmethod
    def _parse_response(response) -> str:
        """Extract and clean the greeting text from Gemini's response.

        Parameters
        ----------
        response
            The ``GenerateContentResponse`` object from the SDK.

        Returns
        -------
        str
            Cleaned greeting text, or empty string if unparseable.
        """
        try:
            raw: str = response.text
        except (AttributeError, ValueError):
            return ""

        # Strip whitespace, surrounding quotes, trailing punctuation artefacts
        cleaned: str = raw.strip().strip('"').strip("'").strip()
        return cleaned
