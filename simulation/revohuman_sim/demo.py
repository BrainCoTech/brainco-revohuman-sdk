"""Demo broadcast: no hardware; sends fake data on a private temporary socket using the same protocol as the
SDK broadcast (schema 1).

The visualization stays a pure subscriber as usual — ``--demo`` goes through exactly the same receive,
scene-building and rendering path as the real SDK; the only difference is that the data is generated here:

  joints   small sine waves, staggered between the two hands;
  tactile  when the product model has tactile, a fake pressure blob that cycles over the five fingers,
           moving from fingertip to finger base (peak ~800 mN);
  tracker  in a z-up world, each hand draws small circles, moves up and down, and slowly rotates the wrist.
           Poses are given in the real Tracker's raw frame (derived backwards via
           ``PalmFromTracker.tracker_raw``), so the demo also goes through the real Tracker's conversion path.
"""

from __future__ import annotations

import math
import os
import shutil
import socket
import tempfile
import threading
import time

from .live_client import SCHEMA, encode

#: Product models used by the demo (same names and peripherals as the SDK's product model table; the product
#: model table for real data exists only in the SDK).
DEMO_PRODUCTS = {
    "P1": ("encoder", "tracker"),
    "PTV1": ("encoder", "tracker", "tactile", "wrist_camera"),
}
#: Encoder index order (matches the SDK joint table; cross-checked in the tests).
DEMO_FINGERS = (("index", ("DIP", "PIP", "MCP", "MPR")), ("middle", ("DIP", "PIP", "MCP", "MPR")),
                ("ring", ("DIP", "PIP", "MCP", "MPR")), ("little", ("DIP", "PIP", "MCP", "MPR")),
                ("thumb", ("DIP", "PIP", "MCP", "CMR", "CMP")))


def demo_joints(hand: str) -> list[dict]:
    out = []
    for finger, joints in DEMO_FINGERS:
        for joint in joints:
            out.append({"index": len(out), "name": f"{hand}_{finger}_{joint}_joint", "finger": finger, "joint": joint})
    return out


def demo_hello(product: str, hands, point_count: int) -> dict:
    sensors = DEMO_PRODUCTS[product]
    has_tac = "tactile" in sensors
    return {
        "type": "hello", "schema": SCHEMA, "sdk": "demo", "pid": os.getpid(),
        "product": {"code": product, "name": f"demo {product}", "sensors": list(sensors), "source": "demo"},
        "hands": list(hands), "joints": {h: demo_joints(h) for h in hands}, "joint_unit": "rad",
        "gloves": {h: {"port": "demo"} for h in hands},
        "tactile": {h: {"variant_has": has_tac, "hand_has": has_tac, "note": "",
                        "point_count": point_count if has_tac else None, "unit": "mN" if has_tac else None}
                    for h in hands},
        "tracker": {"variant_has": True, "enabled": True, "reason": "",
                    "roles": {h: f"DEMO-{h[0].upper()}" for h in hands}, "gate": "flag", "phi_pack_ms": {}},
    }


def demo_q(k_hand: int, n: int, t: float) -> list[float]:
    phase = 0.9 * k_hand
    return [math.radians(25.0 * (0.5 - 0.5 * math.cos(2 * math.pi * 0.15 * t + i * 0.3 + phase))) for i in range(n)]


def demo_tactile(layout, k_hand: int, t: float) -> list[int]:
    from .tactile_layout import FINGERS
    phase = 0.9 * k_hand
    rows_max = {f: len(spec["rows"]) for f, spec in layout.fingers.items()}
    finger = FINGERS[(int(t / 2.0) + int(round(phase))) % len(FINGERS)]
    y_center = ((t / 2.0) % 1.0) * (rows_max[finger] - 1)
    tac = [0] * layout.point_count
    for p in layout.points:
        if p.finger != finger:
            continue
        d = (p.y - y_center) ** 2 / 1.5 + ((p.x - 3.5) / 2.5) ** 2
        tac[p.idx] = max(0, int(800.0 * math.exp(-d) + 6.0 * math.sin(7.0 * t + p.idx)))
    return tac


def palm_target(np, hand: str, t: float):
    s = 1.0 if hand == "left" else -1.0
    ph = 0.0 if hand == "left" else 1.3
    w = 2 * math.pi * 0.12
    p = np.array([0.35 + 0.06 * math.cos(w * t + ph),
                  s * 0.17 + 0.05 * math.sin(w * t + ph),
                  1.05 + 0.04 * math.sin(2 * w * t + ph)])
    yaw = math.radians(20.0) * math.sin(0.7 * w * t + ph)
    roll = s * math.radians(35.0) * math.sin(0.5 * w * t + ph)
    cy, sy, cr, sr = math.cos(yaw), math.sin(yaw), math.cos(roll), math.sin(roll)
    Rz = np.array([[cy, -sy, 0], [sy, cy, 0], [0, 0, 1]])
    Rx = np.array([[1, 0, 0], [0, cr, -sr], [0, sr, cr]])
    return p, Rz @ Rx


class DemoBroadcaster:
    """Demo broadcast (socket in a private temporary directory, deleted when the process exits). ``path`` can
    be connected to after ``start()``."""

    def __init__(self, product: str = "PTV1", hands=("left", "right"), layout=None, rate_hz: float = 100.0,
                 pose_hz: float = 60.0, urdf_dir=None):
        if product not in DEMO_PRODUCTS:
            raise ValueError(f"演示型号只有 {sorted(DEMO_PRODUCTS)}")
        self.product = product
        self.hands = tuple(hands)
        self.layout = layout
        self.rate_hz = rate_hz
        self.pose_hz = pose_hz
        self.urdf_dir = urdf_dir
        n = layout.point_count if layout is not None else 0
        self.hello = demo_hello(product, self.hands, n)
        self.has_tactile = "tactile" in DEMO_PRODUCTS[product] and layout is not None
        self._dir = tempfile.mkdtemp(prefix="revohuman-demo-")
        self.path = os.path.join(self._dir, "live.sock")
        self._stop = threading.Event()
        self._lock = threading.Lock()
        self._clients: list = []
        self._frames = {h: 0 for h in self.hands}
        self._sock = None
        self._threads: list = []
        self._conv = {}

    def start(self) -> "DemoBroadcaster":
        import numpy as np
        from .hand_model import load_model
        from .poses import PalmFromTracker
        self.np = np
        m, _d, _urdf = load_model(self.hands, self.urdf_dir)
        self._conv = {h: PalmFromTracker(np, m, h) for h in self.hands}
        s = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
        s.bind(self.path)
        s.listen(4)
        s.settimeout(0.5)
        self._sock = s
        for fn in (self._accept_loop, self._data_loop, self._status_loop):
            t = threading.Thread(target=fn, daemon=True, name=f"demo-{fn.__name__}")
            t.start()
            self._threads.append(t)
        return self

    def close(self) -> None:
        self._stop.set()
        self._broadcast(encode({"type": "bye", "reason": "demo closed"}))
        for t in self._threads:
            t.join(timeout=1.0)
        with self._lock:
            clients, self._clients = self._clients, []
        for c in clients:
            try:
                c.close()
            except OSError:
                pass
        if self._sock is not None:
            self._sock.close()
        shutil.rmtree(self._dir, ignore_errors=True)

    def _broadcast(self, data: bytes) -> None:
        with self._lock:
            clients = list(self._clients)
        for c in clients:
            try:
                c.sendall(data)
            except OSError:
                with self._lock:
                    if c in self._clients:
                        self._clients.remove(c)
                try:
                    c.close()
                except OSError:
                    pass

    def _status(self) -> dict:
        return {"type": "status", "kit_fatal": None,
                "gloves": {h: {"frames": n, "error": None} for h, n in self._frames.items()},
                "tracker": {"active": True, "fatal": None,
                            "geometry": {"ok": True, "errors": 0, "flags": 0, "pending": 0},
                            "start_gates": []},
                "clients": len(self._clients)}

    def _accept_loop(self) -> None:
        hello = encode(self.hello)
        while not self._stop.is_set():
            try:
                conn, _ = self._sock.accept()
            except socket.timeout:
                continue
            except OSError:
                return
            conn.settimeout(2.0)
            try:
                conn.sendall(hello + encode(self._status()))
            except OSError:
                conn.close()
                continue
            with self._lock:
                self._clients.append(conn)

    def _data_loop(self) -> None:
        t0 = time.monotonic()
        seq = 0
        n = {h: len(self.hello["joints"][h]) for h in self.hands}
        full = {h: (1 << n[h]) - 1 for h in self.hands}
        next_pose = 0.0
        dt = 1.0 / self.rate_hz
        while not self._stop.wait(dt):
            t = time.monotonic() - t0
            seq += 1
            for k, h in enumerate(self.hands):
                q = demo_q(k, n[h], t)
                now_ns = time.time_ns()                       # demo data: sampled at arrival, latency 0
                msg = {"type": "glove", "hand": h, "seq": seq, "t_ns": now_ns, "sample_ns": now_ns,
                       "deg": [round(180.0 + math.degrees(v), 3) for v in q], "q": [round(v, 6) for v in q],
                       "valid": full[h], "tactile": None}
                if self.has_tactile:
                    msg["tactile"] = {"values": demo_tactile(self.layout, k, t), "unit": "mN", "valid": True,
                                      "link_ok": True, "point_count": self.layout.point_count, "calibrated": True}
                self._broadcast(encode(msg))
                self._frames[h] += 1
            if t >= next_pose:
                next_pose = t + 1.0 / self.pose_hz
                for h in self.hands:
                    p, q = self._conv[h].tracker_raw(*palm_target(self.np, h, t))
                    now_ns = time.time_ns()
                    self._broadcast(encode({"type": "pose", "role": h, "serial": self.hello["tracker"]["roles"][h],
                                            "p": list(p), "q": list(q), "age_s": 0.0,
                                            "t_ns": now_ns, "sample_ns": now_ns}))

    def _status_loop(self) -> None:
        while not self._stop.wait(1.0):
            self._broadcast(encode(self._status()))
