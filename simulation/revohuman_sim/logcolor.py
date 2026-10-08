"""logcolor.py — 可视化的日志按级别上色。

只改终端里的显示：输出流不是终端（重定向到文件、管道、CI）时，日志与不上色时逐字节相同。

开关只有环境变量 ``REVOHUMAN_LOG_COLOR``（与 SDK 命令行同一个）：

    auto（默认）  输出流是终端、没设 NO_COLOR、TERM 不是 dumb 时上色
    always        总是上色
    never         从不上色

其他取值按 auto 处理。配色：DEBUG 暗淡、INFO 不上色、WARNING 黄、ERROR 红、CRITICAL 红色加粗。
可视化不依赖 SDK，所以这里是一份独立实现，行为与 SDK 的一致。
"""

from __future__ import annotations

import logging
import os
import sys
from typing import Mapping, Optional

ENV = "REVOHUMAN_LOG_COLOR"
MODES = ("auto", "always", "never")

RESET = "\033[0m"
#: (最低级别, 起始转义)，从高到低匹配；空串 = 不上色
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
    """按级别给整条日志套颜色；``color=False`` 时输出与 ``logging.Formatter`` 完全相同。"""

    def __init__(self, fmt: Optional[str] = None, datefmt: Optional[str] = None, *, color: bool):
        super().__init__(fmt, datefmt)
        self.color = color

    def format(self, record: logging.LogRecord) -> str:
        text = super().format(record)
        prefix = style(record.levelno) if self.color else ""
        return f"{prefix}{text}{RESET}" if prefix else text


def basic_config(level: int, fmt: str, stream=None) -> None:
    """等同 ``logging.basicConfig(level=level, format=fmt, stream=stream)``，只多了按级别上色。"""
    stream = sys.stderr if stream is None else stream
    handler = logging.StreamHandler(stream)
    handler.setFormatter(ColorFormatter(fmt, color=enabled(stream)))
    logging.basicConfig(level=level, handlers=[handler])
