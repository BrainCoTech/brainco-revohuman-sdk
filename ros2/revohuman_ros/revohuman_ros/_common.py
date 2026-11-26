from __future__ import annotations
import collections
import json
import logging
import math
import threading
import time
from typing import Optional
from builtin_interfaces.msg import Time
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import JointState
from std_msgs.msg import String
from std_srvs.srv import Trigger
from revohuman import joint_map
from revohuman.codes import format_error
from revohuman_msgs.msg import TactileArray
from revohuman_msgs.srv import Calibrate
DATA_QOS = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT, history=HistoryPolicy.KEEP_LAST, depth=10, durability=DurabilityPolicy.VOLATILE)
STATUS_QOS = QoSProfile(reliability=ReliabilityPolicy.RELIABLE, history=HistoryPolicy.KEEP_LAST, depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)

class _RosLogHandler(logging.Handler):

    def __init__(self, node):
        super().__init__()
        self._logger = node.get_logger()

    def emit(self, record: logging.LogRecord) -> None:
        try:
            msg = self.format(record)
            name = record.name.rsplit('.', 1)[-1]
            text = f'[sdk:{name}] {msg}'
            lv = record.levelno
            if lv >= logging.CRITICAL:
                self._logger.fatal(text)
            elif lv >= logging.ERROR:
                self._logger.error(text)
            elif lv >= logging.WARNING:
                self._logger.warning(text)
            elif lv >= logging.INFO:
                self._logger.info(text)
            else:
                self._logger.debug(text)
        except Exception:
            self.handleError(record)

def install_sdk_log_bridge(node, level: int=logging.INFO, namespace: str='revohuman') -> None:
    logger = logging.getLogger(namespace)
    for h in logger.handlers:
        if isinstance(h, _RosLogHandler):
            h._logger = node.get_logger()
            logger.setLevel(level)
            return
    handler = _RosLogHandler(node)
    handler.setLevel(logging.NOTSET)
    handler.setFormatter(logging.Formatter('%(message)s'))
    logger.addHandler(handler)
    logger.setLevel(level)
PAIR_MODES = ('strict', 'best_effort')
PUBLISH_TACTILE = ('auto', 'off')

def glove_status(glove, counters: Optional[dict]=None, pairing: Optional[dict]=None, extra: Optional[dict]=None) -> str:
    st = glove.stats()
    stream = st.stream or {}
    out = {'frames_by_tier': stream.get('frames_by_tier'), 'capture_gate': stream.get('capture_gate'), 'frames_blocked': stream.get('frames_blocked'), 'effective_rate_hz': st.effective_rate_hz, 'mapper_quality': st.mapper_quality, 'handedness': str(st.handedness) if st.handedness is not None else None, 'stream_counters': {k: stream.get(k) for k in ('frames_received', 'frames_dropped_local', 'device_skipped_total', 'seq_gap_frames', 'queue_len')}, 'tactile_transport': stream.get('tactile_transport'), 'tactile_split': stream.get('tactile_split'), 'tactile_health': stream.get('tactile_health')}
    tare_state = getattr(glove, 'tactile_tare_state', None)
    if callable(tare_state):
        out['tactile_tare'] = tare_state()
    if pairing is not None:
        out['pairing'] = pairing
    if counters:
        out['node'] = dict(counters)
    if extra:
        out.update(extra)
    return json.dumps(out, ensure_ascii=False, default=str)

def tactile_tare_payload(gloves: dict, stamp_ns: int) -> str:
    out = {'stamp_ns': int(stamp_ns)}
    for (hand, glove) in gloves.items():
        out[hand] = glove.tactile_tare_state()
    return json.dumps(out, ensure_ascii=False, default=str)

class TareStatePublisher:

    def __init__(self, node, gloves: dict):
        self._node = node
        self._gloves = gloves
        self._pub = node.create_publisher(String, 'tactile_tare_state', STATUS_QOS)
        self._lock = threading.Lock()
        self._pending = collections.deque()
        self._wake = node.create_guard_condition(self._drain)
        self.published = 0

    def changed(self, *_args) -> None:
        with self._lock:
            self._pending.append(tactile_tare_payload(self._gloves, time.time_ns()))
        try:
            self._wake.trigger()
        except Exception:
            pass

    def _drain(self) -> None:
        while True:
            try:
                data = self._pending.popleft()
            except IndexError:
                return
            msg = String()
            msg.data = data
            try:
                self._pub.publish(msg)
                self.published += 1
            except Exception as exc:
                self._node.get_logger().warning(f'tactile_tare_state 发布失败：{exc}')

def check_choice(name: str, value: str, choices: tuple) -> str:
    if value not in choices:
        raise ValueError(f"参数 {name} 只有 {' | '.join(choices)}，收到 {value!r}")
    return value

def to_stamp(ns: int) -> Time:
    ns = int(ns)
    return Time(sec=ns // 1000000000, nanosec=ns % 1000000000)

def frame_stamp_ns(frame) -> Optional[int]:
    return frame.sample_wall_ns

def tactile_stamp_ns(frame, tactile, joint_stamp_ns: int) -> int:
    if tactile.sample_wall_ns is not None:
        return int(tactile.sample_wall_ns)
    (tt, et) = (tactile.device_tick, frame.device_tick)
    if tt is not None and et is not None:
        return int(joint_stamp_ns + (int(tt) - int(et)) * 1000000)
    return int(joint_stamp_ns)

def joint_positions_rad(frame) -> list[float]:
    deg = [float(v) if frame.joints_valid_mask >> i & 1 else math.nan for (i, v) in enumerate(frame.joints_deg)]
    return list(joint_map.to_radians(deg))

def joint_names(hand: str) -> list[str]:
    return list(joint_map.joint_names(handedness=hand))

def make_joint_state(stamp_ns: int, frame_id: str, names: list[str], positions: list[float]) -> JointState:
    msg = JointState()
    msg.header.stamp = to_stamp(stamp_ns)
    msg.header.frame_id = frame_id
    msg.name = names
    msg.position = positions
    return msg

def make_tactile(frame, tactile, joint_stamp_ns: int, frame_id: str) -> TactileArray:
    msg = TactileArray()
    msg.header.stamp = to_stamp(tactile_stamp_ns(frame, tactile, joint_stamp_ns))
    msg.header.frame_id = frame_id
    msg.frame_stamp = to_stamp(joint_stamp_ns)
    msg.data = [int(v) & 4294967295 for v in tactile.values]
    msg.calibrated = bool(tactile.calibrated)
    return msg

def make_tactile_sample(sample, frame_id: str) -> TactileArray:
    t = sample.tactile
    stamp = to_stamp(sample.sample_wall_ns)
    msg = TactileArray()
    msg.header.stamp = stamp
    msg.header.frame_id = frame_id
    msg.frame_stamp = stamp
    msg.data = [int(v) & 4294967295 for v in t.values]
    msg.calibrated = bool(t.calibrated)
    return msg

def sample_publishable(sample) -> bool:
    return sample is not None and sample.sample_wall_ns is not None and bool(sample.tactile.point_count) and tactile_usable(sample.tactile)

def tactile_usable(tactile) -> bool:
    if tactile is None:
        return False
    u = getattr(tactile, 'usable', None)
    if u is not None:
        return bool(u)
    return bool(tactile.link_ok) and bool(tactile.valid) and (not getattr(tactile, 'fault_reason', None))

def register_glove_services(node, glove, prefix: str='', *, allow_calibration: bool=False, tare_callback_group=None) -> list:

    def calibrate(req, resp):
        try:
            glove.calibrate(target=int(req.target))
            (resp.success, resp.message) = (True, f'calibrated target={req.target}')
        except Exception as exc:
            (resp.success, resp.message) = (False, format_error(exc))
        return resp

    def clear_calibration(_req, resp):
        try:
            glove.clear_calibration()
            (resp.success, resp.message) = (True, 'calibration cleared')
        except Exception as exc:
            (resp.success, resp.message) = (False, format_error(exc))
        return resp

    def selftest(_req, resp):
        try:
            rep = glove.selftest()
            resp.success = bool(rep.ok)
            resp.message = 'ok' if rep.ok else '; '.join(rep.problems)
        except Exception as exc:
            (resp.success, resp.message) = (False, format_error(exc))
        return resp

    def tactile_tare(_req, resp):
        try:
            state = glove.tactile_tare()
            (resp.success, resp.message) = (True, json.dumps(state, ensure_ascii=False, default=str))
        except Exception as exc:
            (resp.success, resp.message) = (False, format_error(exc))
        return resp

    def get_sn(_req, resp):
        try:
            (resp.success, resp.message) = (True, glove.get_sn())
        except Exception as exc:
            (resp.success, resp.message) = (False, format_error(exc))
        return resp
    services = [node.create_service(Trigger, prefix + 'selftest', selftest), node.create_service(Trigger, prefix + 'get_sn', get_sn), node.create_service(Trigger, prefix + 'tactile_tare', tactile_tare, callback_group=tare_callback_group)]
    if allow_calibration:
        services += [node.create_service(Calibrate, prefix + 'calibrate', calibrate), node.create_service(Trigger, prefix + 'clear_calibration', clear_calibration)]
    return services
