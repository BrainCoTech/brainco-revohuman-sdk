"""Tracker embedding shared by both nodes: runtime-mode session, base station geometry calibration generation,
topics/services and 1 Hz status.

Tracker and glove are captured in the same process; ``header.stamp`` on both sides is the sampling time in
CLOCK_REALTIME nanoseconds, so downstream pairs them directly by ``header.stamp``.

``tracker_mode``: ``required`` (default: if the tracker does not start, the whole node fails to start) /
``auto`` (only a warning if it does not start; the glove still publishes) / ``off`` (glove only, does not
import ``revohuman.tracker``).

The tracker serial number ↔ handedness mapping is decided by the following priority, never guessed from
libsurvive's enumeration order:
① parameters ``tracker_serial`` / ``tracker_left_serial`` / ``tracker_right_serial``;
② the tracker physically attached to the glove unit's HUB (sysfs 28de:2300; the glove unit's handedness is
   decided by the handedness recorded in the MCU firmware / the serial number; before resolving, each MCU
   present that has not been read yet is read once for its firmware-recorded handedness, since on recent
   firmware the serial number does not carry handedness).
After the gloves connect this is checked once more: an auto-resolved tracker must be attached to the HUB of
the unit actually connected this time.

**Two process generations.** The tracking library can be initialized only once per process, and calibration
mode and runtime mode are told apart by environment variables set before startup, so the two modes cannot be
switched within one process:

- Runtime mode (normal): ``TrackerRuntime`` locks the base station geometry, evaluates the geometry gate and
  performs zero-pose calibration;
- Calibration generation: when the mode file in the state directory is ``calibrate``, the node does not
  connect the gloves and runs only one round of base station geometry calibration; when it ends (success /
  failure / abort) it writes the mode back to ``runtime`` and exits with exit code 5, and launch relaunches
  it in runtime mode.

Service ``~/tracker/set_calibration_mode`` (``std_srvs/SetBool``): in runtime mode ``true`` switches to the
calibration generation; in the calibration generation ``false`` aborts calibration (config is rolled back to
its state before calibration).
"""

from __future__ import annotations

import json
import threading
from typing import Callable, Optional

from geometry_msgs.msg import PoseStamped
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import String
from std_srvs.srv import SetBool, Trigger

from revohuman.codes import format_error

from . import _common as C

TRACKER_MODES = ("off", "auto", "required")
# World-frame gate: what to do when an allowlisted base station is moved during capture. error = stop capture
# on the spot (such data cannot be told apart afterwards).
WORLD_GATE_MODES = ("error", "flag", "off")
DEFAULT_WORLD_GATE_MODE = "error"
# Base station geometry gate: geometry invalidated at runtime / geometry missing / base station spacing drift.
# error = reject zero-pose calibration.
GEOMETRY_GATE_MODES = ("error", "flag", "off")
DEFAULT_GEOMETRY_GATE_MODE = "error"
DEFAULT_TRACKER_MODE = "required"
EXIT_CODE_MODE_SWITCH = 5
# 1 Hz status: String+JSON, RELIABLE + TRANSIENT_LOCAL, so a late subscriber still gets the last message.
STATUS_QOS = QoSProfile(
    reliability=ReliabilityPolicy.RELIABLE,
    history=HistoryPolicy.KEEP_LAST,
    depth=1,
    durability=DurabilityPolicy.TRANSIENT_LOCAL,
)
# TrackerRuntime injection points for tests only; the remaining tracker_kwargs go to SurviveTrackerSource as-is.
_RUNTIME_HOOKS = ("probe_fn", "source_factory", "clock")


def declare_tracker_params(node) -> dict:
    """Tracker parameters shared by both nodes (the serial-number parameters are declared by each node)."""
    mode = C.check_choice(
        "tracker_mode", str(node.declare_parameter("tracker_mode", DEFAULT_TRACKER_MODE).value),
        TRACKER_MODES)
    lhs = node.declare_parameter("tracker_lighthouses", [""]).value
    lhs = [str(x).strip() for x in (lhs or []) if str(x).strip()]
    args = node.declare_parameter("tracker_survive_args", [""]).value
    args = [str(x) for x in (args or []) if str(x)]
    gate = float(node.declare_parameter("tracker_liveness_gate_s", -1.0).value)
    # Absolute accuracy budget for the sampling time (milliseconds); negative = not required. When required,
    # tracker_phi_pack_ms must be given as well.
    acc = float(node.declare_parameter("tracker_stamp_accuracy_ms", -1.0).value)
    phi = _parse_phi_pack(node.declare_parameter("tracker_phi_pack_ms", [""]).value or [])
    world = C.check_choice(
        "tracker_world_gate_mode",
        str(node.declare_parameter("tracker_world_gate_mode", DEFAULT_WORLD_GATE_MODE).value),
        WORLD_GATE_MODES)
    geometry = C.check_choice(
        "tracker_geometry_gate_mode",
        str(node.declare_parameter("tracker_geometry_gate_mode", DEFAULT_GEOMETRY_GATE_MODE).value),
        GEOMETRY_GATE_MODES)
    timeout = float(node.declare_parameter("tracker_calibration_timeout_s", 300.0).value)
    return {
        "mode": mode,
        "world_gate_mode": world,
        "geometry_gate_mode": geometry,
        "stamp_accuracy_ms": (None if acc < 0 else acc),
        "phi_pack_ms": phi or None,
        # empty = resolved from the base station allowlist via the identity table (`revohuman-lighthouses scan`
        # writes the identity table)
        "lighthouses": lhs or None,
        # empty = SDK default libsurvive arguments
        "args": args or None,
        # negative = SDK automatic; 0 = off; positive = window upper limit (seconds)
        "liveness_gate_s": (None if gate < 0 else gate),
        "frame_id": str(node.declare_parameter("tracker_frame_id", "world").value),
        # empty = default ($REVOHUMAN_TRACKER_STATE_DIR > $XDG_STATE_HOME/revohuman/tracker >
        # ~/.local/state/revohuman/tracker)
        "state_dir": str(node.declare_parameter("tracker_state_dir", "").value).strip() or None,
        # empty = ~/.config/libsurvive/config.json
        "config_path": str(node.declare_parameter("tracker_config_path", "").value).strip() or None,
        "fence": bool(node.declare_parameter("tracker_fence", True).value),
        "require_preflight": bool(node.declare_parameter("tracker_require_preflight", True).value),
        "calibration_timeout_s": timeout,
    }


def _parse_phi_pack(items) -> dict:
    """``["LHR-XXXXXXXX=0.31"]`` → ``{"LHR-XXXXXXXX": 0.31}``. 0 is a legal calibration value."""
    out = {}
    for raw in items:
        txt = str(raw).strip()
        if not txt:
            continue
        if "=" not in txt:
            raise ValueError(f"参数 tracker_phi_pack_ms 的条目要写成 SERIAL=毫秒，收到 {txt!r}")
        serial, _, val = txt.partition("=")
        serial = serial.strip()
        if not serial:
            raise ValueError(f"参数 tracker_phi_pack_ms 的条目缺序列号：{txt!r}")
        try:
            out[serial] = float(val.strip())
        except ValueError:
            raise ValueError(f"参数 tracker_phi_pack_ms 的 {serial} 取值要是数字，"
                             f"收到 {val.strip()!r}") from None
    return out


def require_serial(node, name: str, mode: str,
                   resolver: Optional[Callable[[], Optional[tuple[str, str]]]] = None) -> Optional[str]:
    """Decides this hand's tracker serial number: parameter > ``resolver()`` (``GloveManager.resolve_tracker``).
    ``tracker_mode != off`` and neither finds one → refuse to start. With ``mode == off`` the parameter is
    only echoed back."""
    val = str(node.declare_parameter(name, "").value).strip()
    if val:
        if mode != "off":
            node.get_logger().info(f"{name}={val}（来源：参数）")
        return val
    if mode == "off":
        return None
    found = None
    if resolver is not None:
        try:
            found = resolver()
        except Exception as exc:  # noqa: BLE001 —— sysfs read failure etc.: treat as "not resolved", say so
            node.get_logger().warning(f"{name} 自动解析失败：{format_error(exc)}")
    if found:
        serial, source = found
        node.get_logger().info(f"{name}={serial}（来源：{source}）")
        return serial
    raise ValueError(
        f"tracker_mode={mode} 需要 {name}，但找不到：\n"
        f"  ① 参数 {name}:=LHR-xxxxxxxx；\n"
        f"  ② tracker 插在该手套单元自己的 HUB 上（sysfs 28de:2300 自动识别；"
        f"手套 MCU 固件登记的手别或序列号要能认出手别）。\n"
        f"只要手套不要 tracker：tracker_mode:=off"
    )


# these connection kwargs injected = mock / unit-test transport layer: same rule as ``GloveManager.connect``,
# do not touch a real serial port to read handedness
INJECTED_IO_KWARGS = ("stream_factory", "vendor_connect", "vendor_connect_factory")


def query_devices_enabled(connect_kwargs: Optional[dict], manager_kwargs: Optional[dict]) -> bool:
    """Whether to query the present gloves for handedness before resolving the tracker
    (``GloveManager.resolve_tracker(query_devices=)``): yes for a real serial port; no when the transport layer
    is injected, unless the unit test injected the query as well (``manager_kwargs``)."""
    if any(k in (connect_kwargs or {}) for k in INJECTED_IO_KWARGS):
        return bool(manager_kwargs)
    return True


def confirm_serial(node, name: str, serial: Optional[str], unit_id: Optional[str],
                   sysfs_root=None) -> None:
    """Check after the gloves connect: the auto-resolved tracker really is attached to the HUB of the glove
    unit connected this time.

    The tracker has to be decided before the glove is opened (the calibration generation never opens the
    glove), while connecting locates the unit by identity once more (the handedness in the shared cache file
    is checked on the spot and re-decided if it does not match); if the two locations do not give the same
    unit, refuse to start: poses attributed to the wrong hand are more dangerous than not starting.
    Not checked: explicitly given by parameter, ``tracker_mode=off``, or the connected unit is no longer in
    sysfs (cannot tell whether it is genuine)."""
    if not serial or not unit_id or str(node.get_parameter(name).value).strip():
        return
    from revohuman import identity
    unit = next((u for u in identity.scan_units(sysfs_root) if u.unit_id == unit_id), None)
    if unit is None or unit.tracker_serial == serial:
        return
    raise RuntimeError(
        f"{name}：启动时解析到的 tracker {serial} 不在这次连上的手套单元 {unit_id} 的 HUB 上"
        f"（那里实挂的是 {unit.tracker_serial or '无'}）——两次定位的手别不一致，拒绝启动；"
        f"确认哪只手套是这只手后用参数 {name}:=LHR-xxxxxxxx 显式指定")


# ---------------------------------------------------------------- mode file


def _state_dir(params: dict) -> str:
    from revohuman.tracker.calibration.lighthouses import default_state_dir
    return str(params.get("state_dir") or default_state_dir())


def calibration_pending(params: dict) -> bool:
    """Whether the mode file in the state directory asks this process to run the calibration generation."""
    from revohuman.tracker.calibration import session
    return session.read_mode(_state_dir(params)) == "calibrate"


# ---------------------------------------------------------------- runtime mode


def create_tracker_runtime(node, roles: dict, params: dict, extra_kwargs: Optional[dict] = None):
    """Constructs ``TrackerRuntime`` (**does not start it**). On failure auto mode returns None and required
    mode raises.

    ``roles`` = ``{"left": serial, "right": serial}``; ``extra_kwargs`` is passed as-is to
    ``SurviveTrackerSource`` (tests inject ``backend=``), except ``probe_fn`` / ``source_factory`` /
    ``clock``, which go to ``TrackerRuntime``."""
    try:
        from revohuman.tracker.calibration import TrackerRuntime   # lazy: requires pysurvive
        extra = dict(extra_kwargs or {})
        hooks = {k: extra.pop(k) for k in _RUNTIME_HOOKS if k in extra}
        skw = dict(stamp_accuracy_ms=params["stamp_accuracy_ms"],
                   world_gate_mode=params["world_gate_mode"])
        if params.get("phi_pack_ms"):
            skw["phi_pack_ms"] = params["phi_pack_ms"]
        skw.update(extra)
        # The node does not use the SDK's start gate and runtime guard for now (the geometry gate is evaluated
        # at 1Hz by TrackerServices); the zero pose is restored from disk, otherwise a respawn (e.g. the
        # exit-code-5 relaunch after calibration) would record a wrong zero pose from the hand posture held at
        # that moment
        return TrackerRuntime(
            roles, registered=params["lighthouses"], state_dir=params["state_dir"],
            config_path=params["config_path"], gate_mode=params["geometry_gate_mode"],
            fence=params["fence"], survive_args=params["args"],
            liveness_gate_s=params["liveness_gate_s"], source_kwargs=skw,
            start_gate=False, guard_interval_s=0, restore=True, auto_reference=False, **hooks)
    except Exception as exc:  # noqa: BLE001 —— ImportError (pysurvive not installed) / invalid parameters
        return _fail(node, params, exc)


def start_tracker_runtime(node, rt, params: dict,
                          before_start: Optional[Callable[[object], None]] = None) -> bool:
    """Starts capture. True = started; False = did not start in auto mode (the caller tears down the tracker
    half); required mode raises. ``before_start(src)`` attaches subscriptions before capture starts:
    subscribing after it has started would lose the first frames."""
    try:
        rt.start(before_start=before_start)
    except Exception as exc:  # noqa: BLE001 —— TrackerBusyError / TrackerStreamDead …
        _fail(node, params, exc)
        return False
    st = rt.startup or {}
    if st.get("fence_note"):
        node.get_logger().info(f"外来基站：{st['fence_note']}")
    restore = st.get("restore") or {}
    if restore.get("status") == "ok":
        node.get_logger().info(f"沿用落盘零位 {rt.reference.calibration_id}")
    return True


def install_world_logger(node, src) -> None:
    """A world-frame change (an allowlisted base station moved / a foreign base station solved into the world
    frame) is only logged at ERROR and recorded in the ``world`` section of ``tracker_status``."""
    def _on_world(source, epoch, detail):
        dets = detail.get('detectors') or []
        if "F" in dets:
            what = "外来基站（不在册）被解进了世界系"
            todo = "请关掉外来基站或移出视野后重启节点；它若是自己人，把序列号加进基站白名单并重新标定"
        else:
            what = "在册基站被搬动"
            todo = "请把基站放回原位或重新标定基站几何后重启节点"
        node.get_logger().error(
            f"世界系已变更（纪元 {epoch}，探测器 {'+'.join(dets) or '未知'}）："
            f"{what}，此刻之后的位姿与之前不在同一个坐标系里。"
            f"gate={detail.get('gate_mode')}（error 档下 tracker 已停采）；{todo}")
    src.on_world_epoch = _on_world


def _fail(node, params: dict, exc: BaseException):
    msg = f"tracker 启动失败：{format_error(exc)}"
    if params["mode"] == "required":
        node.get_logger().error(msg)
        raise exc
    node.get_logger().warning(msg + "（tracker_mode=auto：手套照发，tracker 话题不建）")
    return None


class TrackerServices:
    """The three runtime-mode services (``~/tracker/...``) and the 1 Hz geometry gate evaluation.

    ``set_reference`` (Trigger): call with both hands held in the zero pose; records the zero pose and saves
    it to disk, after which the ``tracker_delta*`` topics start publishing data; with the geometry gate in
    error mode it is rejected when geometry is invalidated at runtime / geometry is missing / spacing drifts.
    ``get_status`` (Trigger): runtime-mode status JSON.
    ``set_calibration_mode`` (SetBool): ``true`` → exit and be relaunched by launch as the calibration
    generation.
    """

    def __init__(self, node, rt, params: dict):
        self.node = node
        self.rt = rt
        self.params = params
        self._last_geo = None
        self.services = [
            node.create_service(Trigger, "tracker/set_reference", self._srv_set_reference),
            node.create_service(Trigger, "tracker/get_status", self._srv_get_status),
            node.create_service(SetBool, "tracker/set_calibration_mode", self._srv_calibration_mode),
        ]

    def close(self) -> None:
        for srv in self.services:
            try:
                self.node.destroy_service(srv)
            except Exception:  # noqa: BLE001
                pass
        self.services = []

    def evaluate_geometry(self) -> Optional[dict]:
        """Advances the geometry gate once (called at 1 Hz) and logs when the verdict changes."""
        try:
            geo = self.rt.geometry(advance=True)
        except Exception as exc:  # noqa: BLE001 —— a detection failure must not bring down capture
            self.node.get_logger().warning(f"基站几何门判定失败：{format_error(exc)}",
                                           throttle_duration_sec=30.0)
            return None
        key = (geo.get("ok"), geo.get("available"), tuple(geo.get("errors") or ()),
               tuple(geo.get("flags") or ()))
        if key != self._last_geo:
            self._last_geo = key
            problems = list(geo.get("errors") or ()) + list(geo.get("flags") or ())
            if geo.get("ok"):
                self.node.get_logger().info(
                    "基站几何门：正常" + (f"（间距 {geo['spacing_m']:.4f} m）"
                                         if geo.get("spacing_m") else ""))
            elif problems:
                self.node.get_logger().warning("基站几何门：" + "；".join(map(str, problems)))
            elif geo.get("reason"):
                self.node.get_logger().warning(f"基站几何门：{geo['reason']}")
        return geo

    def status(self) -> dict:
        st = self.rt.status()
        startup = st.get("startup") or {}
        restore = startup.get("restore") or {}
        return {
            "mode": "runtime",
            "registered_lighthouses": st["registered_lighthouses"],
            "roles": st["roles"],
            "reference": st["reference"],
            "geometry": st["geometry"],
            "geometry_gate_mode": self.params["geometry_gate_mode"],
            "state_dir": self.rt.state_dir,
            "startup": {
                "previous_mode": startup.get("previous_mode"),
                "fence_note": startup.get("fence_note"),
                "geometry_missing": (startup.get("liveness") or {}).get("geometry_missing") or "",
                "floor_offset": startup.get("floor_offset"),
                "restore": {k: restore.get(k) for k in ("status", "detail")} if restore else None,
            },
        }

    def _srv_set_reference(self, _req, resp):
        try:
            res = self.rt.set_reference()
        except Exception as exc:  # noqa: BLE001 —— CalibrationRejected / ValueError (missing pose)
            resp.success, resp.message = False, format_error(exc)
            self.node.get_logger().warning(f"零位标定被拒：{resp.message}")
            return resp
        self.node.get_logger().info(f"零位标定完成：{res['calibration_id']}")
        resp.success, resp.message = True, json.dumps(res, ensure_ascii=False, default=str)
        return resp

    def _srv_get_status(self, _req, resp):
        try:
            resp.success, resp.message = True, json.dumps(self.status(), ensure_ascii=False,
                                                          default=str)
        except Exception as exc:  # noqa: BLE001
            resp.success, resp.message = False, format_error(exc)
        return resp

    def _srv_calibration_mode(self, req, resp):
        if not req.data:
            resp.success, resp.message = True, "已在运行档"
            return resp
        try:
            request_calibration(self.rt.state_dir, self.rt.registered,
                                require_preflight=self.params["require_preflight"])
        except Exception as exc:  # noqa: BLE001 —— CalibrationRejected (preflight check / allowlist empty)
            resp.success, resp.message = False, format_error(exc)
            self.node.get_logger().warning(f"进标定态被拒：{resp.message}")
            return resp
        self.node.get_logger().warning(
            f"切到基站几何标定：本进程以退出码 {EXIT_CODE_MODE_SWITCH} 退出，由 launch 重拉成标定代")
        self.node.exit_code = EXIT_CODE_MODE_SWITCH
        resp.success = True
        resp.message = f"进程将以退出码 {EXIT_CODE_MODE_SWITCH} 退出并以标定档重启"
        return resp


def request_calibration(state_dir, registered, *, require_preflight: bool = True) -> None:
    """Writes the mode file as ``calibrate``. Raises ``CalibrationRejected`` when the list of allowlisted base
    stations is empty or the channel preflight check does not pass."""
    from revohuman.errors import CalibrationRejected
    from revohuman.tracker.calibration import session
    if not registered:
        raise CalibrationRejected("state", [
            "在册基站列表为空：把基站背面 FA 开头的序列号填进基站白名单"
            "（用的是哪份文件、缺什么：`revohuman-lighthouses show`）"])
    if require_preflight:
        session.require_preflight(state_dir, registered)
    session.write_mode(state_dir, "calibrate")


# ---------------------------------------------------------------- calibration generation


class CalibrationGeneration:
    """Calibration generation: this process does not connect the gloves, runs only one round of base station
    geometry calibration and sets ``node.exit_code = 5`` when it ends.

    ``tracker_status`` publishes ``{"mode": "calibrate", "state": ..., "coverage": ..., "hint": ...}``,
    telling the user how to move the tracker; ``~/tracker/get_status`` returns the same JSON.
    """

    def __init__(self, node, trackers: dict, params: dict,
                 calibration_kwargs: Optional[dict] = None):
        from revohuman.tracker.calibration import CalibrationSettings
        self.node = node
        self.params = params
        self.trackers = dict(trackers)
        self._kwargs = dict(calibration_kwargs or {})
        settings = self._kwargs.pop("settings", None)
        self.settings = settings or CalibrationSettings(timeout_s=params["calibration_timeout_s"])
        self._stop = threading.Event()
        self._lock = threading.Lock()
        self._status: dict = {"mode": "calibrate", "state": "starting",
                              "trackers": {s: h.value for s, h in self.trackers.items()}}
        self.pub = node.create_publisher(String, "tracker_status", STATUS_QOS)
        self.services = [
            node.create_service(SetBool, "tracker/set_calibration_mode", self._srv_calibration_mode),
            node.create_service(Trigger, "tracker/get_status", self._srv_get_status),
        ]
        self.timer = node.create_timer(1.0, self.publish)
        self.thread = threading.Thread(target=self._run, name="revohuman-tracker-calibration",
                                       daemon=True)
        node.get_logger().warning(
            f"基站几何标定代：trackers={sorted(self.trackers)}，超时 {self.settings.timeout_s:.0f}s。"
            "请在基站视野内缓慢移动双手，每处停住约 1 秒")
        self.thread.start()

    def status(self) -> dict:
        with self._lock:
            return dict(self._status)

    def _set(self, body: dict) -> None:
        with self._lock:
            self._status = {"mode": "calibrate", "trackers": self._status.get("trackers"), **body}

    def publish(self) -> None:
        msg = String()
        msg.data = json.dumps(self.status(), ensure_ascii=False, default=str)
        try:
            self.pub.publish(msg)
        except Exception:  # noqa: BLE001 —— during shutdown the publisher is already destroyed
            pass

    def _run(self) -> None:
        from revohuman.errors import CalibrationRejected
        from revohuman.tracker.calibration import runner, session
        state_dir = _state_dir(self.params)
        log = self.node.get_logger()
        try:
            result = runner.run_geometry_calibration(
                trackers=self.trackers, registered=self.params["lighthouses"],
                state_dir=state_dir, config_path=self.params["config_path"],
                settings=self.settings, fence=self.params["fence"],
                require_channel_preflight=self.params["require_preflight"],
                survive_args=self.params["args"], should_stop=self._stop.is_set,
                on_status=lambda st: self._set(dict(st)), **self._kwargs)
            self._set(dict(result))
            (log.info if result.get("ok") else log.error)(
                ("基站几何标定完成：" if result.get("ok") else "基站几何标定失败：")
                + str(result.get("detail")))
        except CalibrationRejected as exc:
            self._reset_mode(session, state_dir)
            self._set({"state": "rejected", "ok": False, "stage": exc.stage,
                       "reasons": list(exc.reasons), "detail": str(exc)})
            log.error(f"基站几何标定被拒：{format_error(exc)}")
        except Exception as exc:  # noqa: BLE001
            self._reset_mode(session, state_dir)
            self._set({"state": "failed", "ok": False, "detail": format_error(exc)})
            log.error(f"基站几何标定出错：{format_error(exc)}")
        self.publish()
        log.warning(f"标定代结束：以退出码 {EXIT_CODE_MODE_SWITCH} 退出，由 launch 重拉成运行档")
        self.node.exit_code = EXIT_CODE_MODE_SWITCH

    @staticmethod
    def _reset_mode(session, state_dir) -> None:
        try:
            session.write_mode(state_dir, "runtime")
        except OSError:
            pass

    def _srv_calibration_mode(self, req, resp):
        if req.data:
            resp.success, resp.message = True, "已在标定档"
            return resp
        self._stop.set()
        resp.success, resp.message = True, "已请求中止标定：config 回滚到标定前，随后以运行档重启"
        return resp

    def _srv_get_status(self, _req, resp):
        resp.success, resp.message = True, json.dumps(self.status(), ensure_ascii=False, default=str)
        return resp

    def close(self, timeout_s: float = 10.0) -> None:
        self._stop.set()
        if self.thread.is_alive() and threading.current_thread() is not self.thread:
            self.thread.join(timeout_s)
        for closer in [lambda: self.node.destroy_timer(self.timer),
                       lambda: self.node.destroy_publisher(self.pub)] + [
                (lambda s=s: self.node.destroy_service(s)) for s in self.services]:
            try:
                closer()
            except Exception:  # noqa: BLE001
                pass
        self.services = []


# ---------------------------------------------------------------- messages


def frame_stamp_ns(frame):
    """ROS stamp of a tracker frame = sampling time (CLOCK_REALTIME nanoseconds), on the same time axis as the
    joint stamps."""
    return C.frame_stamp_ns(frame)


def make_pose_stamped(stamp_ns: int, frame_id: str, pose) -> PoseStamped:
    """TrackerPose (meters, quaternion (w,x,y,z)) → geometry_msgs/PoseStamped (x,y,z,w)."""
    msg = PoseStamped()
    msg.header.stamp = C.to_stamp(stamp_ns)
    msg.header.frame_id = frame_id
    px, py, pz = pose.position
    qw, qx, qy, qz = pose.rotation
    msg.pose.position.x, msg.pose.position.y, msg.pose.position.z = px, py, pz
    msg.pose.orientation.x, msg.pose.orientation.y = qx, qy
    msg.pose.orientation.z, msg.pose.orientation.w = qz, qw
    return msg


def delta_frame_id(hand: str) -> str:
    """Frame name for the pose relative to the zero pose: this hand's tracker pose at the moment of zero-pose
    calibration."""
    return f"revohuman_{hand}_tracker_zero"


def status_dict(src, sync=None, counters: Optional[dict] = None) -> dict:
    """1 Hz status of the capture source: the fields needed to check liveness and base station identity.

    The three base station identity fields (``lighthouse_channels`` / ``foreign_lighthouses`` /
    ``missing_lighthouses``) only report and do not judge: in a room with neighboring base stations
    ``foreign`` is often non-empty; seeing a foreign channel ≠ being isolated. Whether a station entered the
    solve is decided by the world frame's foreign base station check (the ``world`` section)."""
    st = src.stats()
    trackers = {}
    for serial, t in st["trackers"].items():
        h = t["health"]
        trackers[serial] = {
            "handedness": t["handedness"],
            "healthy": h["healthy"],
            "pose_rate_hz": round(h["pose_rate_hz"], 1),
            "sync_pulse_rate_hz": round(h["sync_pulse_rate_hz"], 1),
            "pose_ratio_ok": h["pose_ratio_ok"],
            "packet_loss_ok": h["packet_loss_ok"],
            "sync_pulse_ok": h["sync_pulse_ok"],
            "cross_residual_ok": h["cross_residual_ok"],
            "lighthouse_channels": h["lighthouse_channels"],
            "foreign_lighthouses": h["foreign_lighthouses"],
            "missing_lighthouses": h["missing_lighthouses"],
            "mapper_ready": (t.get("mapper") or {}).get("ready"),
            "sample_time_known": (t.get("mapper") or {}).get("sample_time_known"),
            "unmapped_frames": t["source"]["unmapped_frames"],
            "frames_blocked": t["source"]["frames_blocked"],
            "frames_by_tier": t["source"]["frames_by_tier"],
            "capture_gate": t["source"]["capture_gate"],
            # l1_tier is the instantaneous value of the last frame; to judge "whether the whole span stayed in
            # L1", read the cumulative l1_by_tier
            "l1_tier": t["source"]["l1_tier"],
            "l1_by_tier": t["source"]["l1_by_tier"],
        }
    world = dict(st.get("world") or {})
    changes = world.pop("changes", [])
    world["change_count"] = len(changes)
    world["last_change"] = changes[-1] if changes else None
    world.pop("frame", None)
    out = {
        "running": st["running"],
        "dead_reason": st["dead_reason"],
        # World-frame epoch: every recording should note the epoch at its start. When pose_watch != "ok" the
        # detector is not installed, so "no alarm" cannot be read as "not moved".
        "world": world,
        "stream_watchdog": st["stream_watchdog"].get("verdict"),
        "reference_serial": st["reference_serial"],
        "lighthouses": st["lighthouses"],
        "trackers": trackers,
    }
    if sync is not None:
        out["pairing"] = sync.stats()
    if counters:
        out["node"] = dict(counters)
    return out


def status_payload(src, sync=None, counters: Optional[dict] = None,
                   session: Optional[dict] = None) -> str:
    """JSON for ``tracker_status``: capture source status + runtime-mode session (zero pose / geometry gate /
    startup health check)."""
    out = status_dict(src, sync, counters)
    if session is not None:
        out["mode"] = session.get("mode", "runtime")
        out["session"] = {k: v for k, v in session.items() if k != "mode"}
    return json.dumps(out, ensure_ascii=False, default=str)
