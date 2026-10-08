from __future__ import annotations
import json
import logging
import math
from typing import Optional
from builtin_interfaces.msg import Time
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import JointState
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
    out = {'frames_by_tier': stream.get('frames_by_tier'), 'capture_gate': stream.get('capture_gate'), 'frames_blocked': stream.get('frames_blocked'), 'effective_rate_hz': st.effective_rate_hz, 'mapper_quality': st.mapper_quality, 'handedness': str(st.handedness) if st.handedness is not None else None, 'stream_counters': {k: stream.get(k) for k in ('frames_received', 'frames_dropped_local', 'device_skipped_total', 'seq_gap_frames', 'queue_len')}}
    if pairing is not None:
        out['pairing'] = pairing
    if counters:
        out['node'] = dict(counters)
    if extra:
        out.update(extra)
    return json.dumps(out, ensure_ascii=False, default=str)

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

def tactile_usable(tactile) -> bool:
    return tactile is not None and bool(tactile.link_ok) and bool(tactile.valid)

def register_glove_services(node, glove, prefix: str='', *, allow_calibration: bool=False) -> list:

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

    def get_sn(_req, resp):
        try:
            (resp.success, resp.message) = (True, glove.get_sn())
        except Exception as exc:
            (resp.success, resp.message) = (False, format_error(exc))
        return resp
    services = [node.create_service(Trigger, prefix + 'selftest', selftest), node.create_service(Trigger, prefix + 'get_sn', get_sn)]
    if allow_calibration:
        services += [node.create_service(Calibrate, prefix + 'calibrate', calibrate), node.create_service(Trigger, prefix + 'clear_calibration', clear_calibration)]
    return services
