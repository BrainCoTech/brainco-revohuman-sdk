"""logcolor.py — colors the visualization's log output by level.

Only changes what is shown in the terminal: when the output stream is not a terminal (redirected to a file, a
pipe, CI), the log is byte-for-byte identical to uncolored output.

The only switch is the environment variable ``REVOHUMAN_LOG_COLOR`` (the same one as the SDK command line):

    auto (default)  color when the output stream is a terminal, NO_COLOR is not set, and TERM is not dumb
    always          always color
    never           never color

Any other value is treated as auto. Colors: DEBUG dim, INFO uncolored, WARNING yellow, ERROR red, CRITICAL
bold red. The visualization does not depend on the SDK, so this is a standalone implementation that behaves
the same as the SDK's.
"""

from __future__ import annotations

import logging
import os
import sys
from typing import Mapping, Optional

ENV = "REVOHUMAN_LOG_COLOR"
MODES = ("auto", "always", "never")

RESET = "\033[0m"
#: (minimum level, opening escape), matched from high to low; empty string = no color
_STYLES = (
    (logging.CRITICAL, "\033[1;31m"),
    (logging.ERROR, "\033[31m"),
    (logging.WARNING, "\033[33m"),
    (logging.INFO, ""),
    (logging.NOTSET, "\033[2m"),
)


def mode(environ: Optional[Mapping[str, str]] = None) -> str:
    env = os.environ if environ is None else environ
    raw = env.get(ENV, "").strip().lower()
    return raw if raw in MODES else "auto"


def enabled(stream, environ: Optional[Mapping[str, str]] = None) -> bool:
    env = os.environ if environ is None else environ
    m = mode(env)
    if m != "auto":
        return m == "always"
    if env.get("NO_COLOR", "") != "" or env.get("TERM", "") == "dumb":
        return False
    try:
        return bool(stream.isatty())
    except (AttributeError, ValueError, OSError):
        return False


def style(levelno: int) -> str:
    for floor, prefix in _STYLES:
        if levelno >= floor:
            return prefix
    return ""


class ColorFormatter(logging.Formatter):
    """Colors the whole log record by level; with ``color=False`` the output is exactly the same as
    ``logging.Formatter``."""

    def __init__(self, fmt: Optional[str] = None, datefmt: Optional[str] = None, *, color: bool):
        super().__init__(fmt, datefmt)
        self.color = color

    def format(self, record: logging.LogRecord) -> str:
        text = super().format(record)
        prefix = style(record.levelno) if self.color else ""
        return f"{prefix}{text}{RESET}" if prefix else text


def basic_config(level: int, fmt: str, stream=None) -> None:
    """Same as ``logging.basicConfig(level=level, format=fmt, stream=stream)``, plus coloring by level."""
    stream = sys.stderr if stream is None else stream
    handler = logging.StreamHandler(stream)
    handler.setFormatter(ColorFormatter(fmt, color=enabled(stream)))
    logging.basicConfig(level=level, handlers=[handler])
