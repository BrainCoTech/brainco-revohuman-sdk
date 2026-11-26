"""Subscribes to the SDK's local broadcast (`revohuman-live`, i.e. `python3 -m revohuman live`).

This one Unix socket is the only link between the visualization and the SDK: this code does not import the SDK
and does not touch the serial ports or the Trackers. The protocol (schema 1) is defined in the module
documentation of the SDK's revohuman/live.py; this side only decodes frames according to it. While the SDK is
not up, it reconnects every 0.5 s; updates start only after connecting and receiving hello; after the SDK
exits (bye / disconnect) the last frame is kept until the SDK comes back.
"""

from __future__ import annotations

import collections
import json
import logging
import os
import socket
import statistics
import struct
import threading
import time

log = logging.getLogger("revohuman_sim")

SCHEMA = 1
SOCKET_ENV = "REVOHUMAN_LIVE_SOCKET"
MAX_MESSAGE_BYTES = 4 << 20
LATENCY_WINDOW_S = 1.0                     # arrival latency = median over this most recent span
                                           # (per-frame values have millisecond-level jitter and would
                                           # jump if shown directly)
_HEADER = struct.Struct(">I")


def socket_candidates(explicit: str | None = None) -> list[str]:
    """Find the socket by the same rules as the SDK: --socket > environment variable >
    $XDG_RUNTIME_DIR/revohuman > /tmp/revohuman-<uid>. Without an explicit path both default locations are
    tried (the SDK falls back to the /tmp one when XDG_RUNTIME_DIR is not writable)."""
    if explicit:
        return [os.path.abspath(explicit)]
    env = os.environ.get(SOCKET_ENV)
    if env:
        return [os.path.abspath(env)]
    out = []
    run = os.environ.get("XDG_RUNTIME_DIR")
    if run:
        out.append(os.path.join(run, "revohuman", "live.sock"))
    out.append(os.path.join("/tmp", f"revohuman-{os.getuid()}", "live.sock"))
    return out


class FrameDecoder:
    """Incremental frame decoder: 4-byte big-endian length + UTF-8 JSON object."""

    def __init__(self, max_bytes: int = MAX_MESSAGE_BYTES):
        self.max_bytes = max_bytes
        self._buf = bytearray()

    def feed(self, data: bytes) -> list:
        self._buf += data
        out = []
        while len(self._buf) >= _HEADER.size:
            (n,) = _HEADER.unpack_from(self._buf)
            if n > self.max_bytes:
                raise ValueError(f"消息长度 {n} 超过上限 {self.max_bytes}")
            end = _HEADER.size + n
            if len(self._buf) < end:
                break
            body = bytes(self._buf[_HEADER.size:end])
            del self._buf[:end]
            msg = json.loads(body.decode("utf-8"))
            if isinstance(msg, dict):
                out.append(msg)
        return out


def encode(msg: dict) -> bytes:
    """Used by the demo broadcast (same framing as the SDK)."""
    body = json.dumps(msg, separators=(",", ":"), ensure_ascii=False, allow_nan=False).encode("utf-8")
    return _HEADER.pack(len(body)) + body


# ----------------------------------------------------------------------------- latest state

class _Stamps:
    """Sampling time / arrival time (wall ns) of one data stream, and the arrival latencies (arrival time −
    sampling time) within the most recent LATENCY_WINDOW_S."""
    __slots__ = ("sample_ns", "recv_ns", "lat")

    def __init__(self):
        self.sample_ns = self.recv_ns = None
        self.lat = collections.deque(maxlen=1024)        # (monotonic seconds, latency ms)

    def feed(self, msg: dict, now: float) -> None:
        sample, recv = _int(msg.get("sample_ns")), _int(msg.get("t_ns"))
        self.sample_ns, self.recv_ns = sample, recv
        if sample is not None and recv is not None:
            self.lat.append((now, (recv - sample) / 1e6))

    def latency_ms(self, now: float):
        vals = [v for t, v in self.lat if now - t <= LATENCY_WINDOW_S]
        return statistics.median(vals) if vals else None


def _int(v):
    return v if isinstance(v, int) and not isinstance(v, bool) else None


class _HandLatest:
    __slots__ = ("q", "deg", "mask", "seq", "count", "t", "tac", "tac_meta", "tac_count", "stamps")

    def __init__(self):
        self.q = self.deg = self.t = self.tac = None
        self.mask = self.seq = self.count = self.tac_count = 0
        self.tac_meta = {}
        self.stamps = _Stamps()


class LiveState:
    """Written by the subscription thread, read by the render loop. ``hello_gen`` is incremented on every
    hello (the subscriber uses it to decide whether to rebuild the scene)."""

    def __init__(self):
        self.lock = threading.Lock()
        self.hello = None
        self.hello_gen = 0
        self.connected = False
        self.path = None                       # current / most recently connected socket
        self.disconnect_reason = ""
        self.error = ""                        # protocol-level problems (unknown schema, etc.)
        self._bye = ""                         # bye reason the SDK sent in this session
                                               # (reported first on disconnect)
        self.status = None
        self.hands: dict = {}
        self.poses: dict = {}
        self.messages = 0

    # ---------------------------------------------------------- subscription thread side

    def on_connect(self, path: str) -> None:
        with self.lock:
            self.path = path

    def on_disconnect(self, reason: str) -> None:
        with self.lock:
            was = self.connected
            self.connected = False
            self.disconnect_reason = reason = self._bye or reason
            self._bye = ""
        if was:
            log.warning("SDK 广播断开（%s），等它重新起来", reason)

    def on_message(self, msg: dict) -> None:
        kind = msg.get("type")
        now = time.monotonic()
        with self.lock:
            self.messages += 1
            if kind == "hello":
                self.hello = msg
                self.hello_gen += 1
                self.connected = True
                self.disconnect_reason = ""
                self.error = ""
                self._bye = ""
                self.status = None
                self.hands = {h: _HandLatest() for h in msg.get("hands") or ()}
                self.poses = {}
            elif kind == "glove":
                lat = self.hands.get(msg.get("hand"))
                if lat is None:
                    return
                lat.q = msg.get("q")
                lat.deg = msg.get("deg")
                lat.mask = int(msg.get("valid") or 0)
                lat.seq = int(msg.get("seq") or 0)
                lat.count += 1
                lat.t = now
                lat.stamps.feed(msg, now)
                tac = msg.get("tactile")
                if tac is not None:
                    lat.tac = tac.get("values")
                    lat.tac_meta = {k: v for k, v in tac.items() if k != "values"}
                    lat.tac_count += 1
            elif kind == "pose":
                role = msg.get("role")
                if role:
                    old = self.poses.get(role) or {}
                    stamps = old.get("stamps") or _Stamps()
                    stamps.feed(msg, now)
                    self.poses[role] = {"p": msg.get("p"), "q": msg.get("q"), "age_s": msg.get("age_s"),
                                        "serial": msg.get("serial"), "t": now, "count": old.get("count", 0) + 1,
                                        "stamps": stamps}
            elif kind == "status":
                self.status = msg
            elif kind == "bye":
                self._bye = f"SDK exited ({msg.get('reason') or 'bye'})"

    def set_error(self, text: str) -> None:
        with self.lock:
            self.error = text

    # ---------------------------------------------------------- render side

    def session(self) -> tuple:
        """(hello, hello_gen, connected)."""
        with self.lock:
            return self.hello, self.hello_gen, self.connected

    def snapshot(self, hand: str) -> dict:
        with self.lock:
            lat = self.hands.get(hand) or _HandLatest()
            st = (self.status or {}).get("gloves", {}).get(hand) or {}
            now = time.monotonic()
            return {"q": None if lat.q is None else list(lat.q), "deg": None if lat.deg is None else list(lat.deg),
                    "mask": lat.mask, "seq": lat.seq, "count": lat.count,
                    "age": None if lat.t is None else now - lat.t,
                    "sample_ns": lat.stamps.sample_ns, "latency_ms": lat.stamps.latency_ms(now),
                    "tac": None if lat.tac is None else list(lat.tac), "tac_meta": dict(lat.tac_meta),
                    "tac_count": lat.tac_count, "error": st.get("error") or ""}

    def pose(self, role: str):
        """(position, rotation_wxyz, age_s); age includes the time elapsed since it was received on this side.
        Returns None if there is no pose."""
        with self.lock:
            p = self.poses.get(role)
            if p is None or p["p"] is None or p["q"] is None:
                return None
            age = (p["age_s"] or 0.0) + (time.monotonic() - p["t"])
            return list(p["p"]), list(p["q"]), age

    def pose_stream(self, role: str) -> dict:
        """Count, latest sampling time and arrival latency of this hand's pose stream (for the overlay). count
        is 0 if no pose has been received."""
        with self.lock:
            p = self.poses.get(role) or {}
            st = p.get("stamps")
            return {"count": p.get("count", 0), "sample_ns": st.sample_ns if st else None,
                    "latency_ms": st.latency_ms(time.monotonic()) if st else None}

    def last_status(self) -> dict | None:
        with self.lock:
            return self.status


# ----------------------------------------------------------------------------- subscription thread

class LiveClient:
    """Background thread: connects to the SDK broadcast, decodes frames, writes LiveState; when disconnected,
    retries every ``retry_s`` seconds."""

    def __init__(self, state: LiveState, path: str | None = None, retry_s: float = 0.5):
        self.state = state
        self.paths = socket_candidates(path)
        self.retry_s = retry_s
        self._stop = threading.Event()
        self._sock = None
        self._thread = threading.Thread(target=self._run, daemon=True, name="revohuman-live-sub")

    def describe(self) -> str:
        return " | ".join(self.paths)

    def start(self) -> "LiveClient":
        self._thread.start()
        return self

    def close(self) -> None:
        self._stop.set()
        s = self._sock
        if s is not None:
            try:
                s.shutdown(socket.SHUT_RDWR)
            except OSError:
                pass
        self._thread.join(timeout=2.0)

    def _connect(self):
        for path in self.paths:
            s = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
            try:
                s.connect(path)
            except OSError:
                s.close()
                continue
            return s, path
        return None, None

    def _run(self) -> None:
        while not self._stop.is_set():
            s, path = self._connect()
            if s is None:
                self._stop.wait(self.retry_s)
                continue
            self._sock = s
            self.state.on_connect(path)
            reason = self._read(s)
            self._sock = None
            try:
                s.close()
            except OSError:
                pass
            if self._stop.is_set():
                return
            self.state.on_disconnect(reason)
            self._stop.wait(self.retry_s)

    def _read(self, s) -> str:
        dec = FrameDecoder()
        s.settimeout(0.5)
        first = True
        while not self._stop.is_set():
            try:
                data = s.recv(1 << 16)
            except socket.timeout:
                continue
            except OSError as exc:
                return f"connection lost ({type(exc).__name__})"
            if not data:
                return "connection closed"
            try:
                msgs = dec.feed(data)
            except ValueError as exc:
                self.state.set_error(f"bad message: {exc}")
                return "protocol error"
            for msg in msgs:
                if first:
                    first = False
                    if msg.get("type") != "hello" or msg.get("schema") != SCHEMA:
                        text = (f"unsupported broadcast schema {msg.get('schema')!r} (viewer speaks {SCHEMA}); "
                                f"update the viewer or the SDK")
                        self.state.set_error(text)
                        log.error("SDK 广播协议版本不符：%s", text)
                        self._stop.wait(2.0)
                        return "schema mismatch"
                self.state.on_message(msg)
        return "closed"
