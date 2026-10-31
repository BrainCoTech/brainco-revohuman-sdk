"""两节点共用的 tracker 嵌入：运行档会话、基站几何标定代、话题/服务与 1 Hz 状态。

tracker 与手套在同一进程里采集，``header.stamp`` 两边都是采样时刻的 CLOCK_REALTIME 纳秒，
下游按 ``header.stamp`` 直接配对。

``tracker_mode``：``required``（默认：tracker 起不来整个节点启动失败）/ ``auto``（起不来只告警，
手套照发）/ ``off``（纯手套，不 import ``revohuman.tracker``）。

tracker 序列号 ↔ 手别按以下优先级确定，不按 libsurvive 的枚举顺序猜：
① 参数 ``tracker_serial`` / ``tracker_left_serial`` / ``tracker_right_serial``；
② 手套单元 HUB 上实挂的 tracker（sysfs 28de:2300；手套单元的手别由 MCU 序列号定）。

**两代进程**。定位库每个进程只能初始化一次，标定档与运行档靠启动前的环境变量区分，
所以两档不能在同一进程里切换：

- 运行档（常态）：``TrackerRuntime`` 锁定基站几何、做几何门判定与零位标定；
- 标定代：状态目录里的模式文件为 ``calibrate`` 时，节点不连手套，只跑一轮基站几何标定，
  结束（成功 / 失败 / 中止）后把模式写回 ``runtime`` 并以退出码 5 退出，由 launch 重拉成运行档。

服务 ``~/tracker/set_calibration_mode``（``std_srvs/SetBool``）：运行档下 ``true`` 切到标定代；
标定代下 ``false`` 中止标定（config 回滚到标定前）。
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
# 世界系门：采集中在册基站被搬动时的处理。error = 当场停采（这种数据事后分辨不出来）。
WORLD_GATE_MODES = ("error", "flag", "off")
DEFAULT_WORLD_GATE_MODE = "error"
# 基站几何门：运行中几何失效 / 几何缺失 / 基站间距漂移。error = 拒绝零位标定。
GEOMETRY_GATE_MODES = ("error", "flag", "off")
DEFAULT_GEOMETRY_GATE_MODE = "error"
DEFAULT_TRACKER_MODE = "required"
EXIT_CODE_MODE_SWITCH = 5
# 1 Hz 状态：String+JSON，RELIABLE + TRANSIENT_LOCAL，晚起的订阅者能补到最后一帧。
STATUS_QOS = QoSProfile(
    reliability=ReliabilityPolicy.RELIABLE,
    history=HistoryPolicy.KEEP_LAST,
    depth=1,
    durability=DurabilityPolicy.TRANSIENT_LOCAL,
)
# 只给测试用的 TrackerRuntime 注入点；其余 tracker_kwargs 原样传给 SurviveTrackerSource。
_RUNTIME_HOOKS = ("probe_fn", "source_factory", "clock")


def declare_tracker_params(node) -> dict:
    """两节点共用的 tracker 参数（序列号参数各自声明）。"""
    mode = C.check_choice(
        "tracker_mode", str(node.declare_parameter("tracker_mode", DEFAULT_TRACKER_MODE).value),
        TRACKER_MODES)
    lhs = node.declare_parameter("tracker_lighthouses", [""]).value
    lhs = [str(x).strip() for x in (lhs or []) if str(x).strip()]
    args = node.declare_parameter("tracker_survive_args", [""]).value
    args = [str(x) for x in (args or []) if str(x)]
    gate = float(node.declare_parameter("tracker_liveness_gate_s", -1.0).value)
    # 采样时刻的绝对精度预算（毫秒）；负数 = 不要求。要求时必须同时给出 tracker_phi_pack_ms。
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
        # 空 = 由基站白名单经身份表解析（`revohuman-lighthouses scan` 写身份表）
        "lighthouses": lhs or None,
        # 空 = SDK 默认 libsurvive 参数
        "args": args or None,
        # 负数 = SDK 自动档；0 = 关；正数 = 窗上限（秒）
        "liveness_gate_s": (None if gate < 0 else gate),
        "frame_id": str(node.declare_parameter("tracker_frame_id", "world").value),
        # 空 = 默认（$REVOHUMAN_TRACKER_STATE_DIR > $XDG_STATE_HOME/revohuman/tracker >
        # ~/.local/state/revohuman/tracker）
        "state_dir": str(node.declare_parameter("tracker_state_dir", "").value).strip() or None,
        # 空 = ~/.config/libsurvive/config.json
        "config_path": str(node.declare_parameter("tracker_config_path", "").value).strip() or None,
        "fence": bool(node.declare_parameter("tracker_fence", True).value),
        "require_preflight": bool(node.declare_parameter("tracker_require_preflight", True).value),
        "calibration_timeout_s": timeout,
    }


def _parse_phi_pack(items) -> dict:
    """``["LHR-XXXXXXXX=0.31"]`` → ``{"LHR-XXXXXXXX": 0.31}``。0 是合法标定值。"""
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
    """确定该手的 tracker 序列号：参数 > ``resolver()``（``identity.resolve_tracker_serial``）。
    ``tracker_mode != off`` 而都找不到 → 拒绝启动。``mode == off`` 时只回显参数。"""
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
        except Exception as exc:  # noqa: BLE001 —— sysfs 读失败等，按「没解析到」处理并说明
            node.get_logger().warning(f"{name} 自动解析失败：{format_error(exc)}")
    if found:
        serial, source = found
        node.get_logger().info(f"{name}={serial}（来源：{source}）")
        return serial
    raise ValueError(
        f"tracker_mode={mode} 需要 {name}，但找不到：\n"
        f"  ① 参数 {name}:=LHR-xxxxxxxx；\n"
        f"  ② tracker 插在该手套单元自己的 HUB 上（sysfs 28de:2300 自动识别；"
        f"手套 MCU 序列号要能认出手别）。\n"
        f"只要手套不要 tracker：tracker_mode:=off"
    )


# ---------------------------------------------------------------- 模式文件


def _state_dir(params: dict) -> str:
    from revohuman.tracker.calibration.lighthouses import default_state_dir
    return str(params.get("state_dir") or default_state_dir())


def calibration_pending(params: dict) -> bool:
    """状态目录里的模式文件是否要求本进程跑标定代。"""
    from revohuman.tracker.calibration import session
    return session.read_mode(_state_dir(params)) == "calibrate"


# ---------------------------------------------------------------- 运行档


def create_tracker_runtime(node, roles: dict, params: dict, extra_kwargs: Optional[dict] = None):
    """构造 ``TrackerRuntime``（**不 start**）。失败时 auto 档返回 None，required 档抛出。

    ``roles`` = ``{"left": serial, "right": serial}``；``extra_kwargs`` 原样传给
    ``SurviveTrackerSource``（测试塞 ``backend=``），其中 ``probe_fn`` / ``source_factory`` /
    ``clock`` 交给 ``TrackerRuntime``。"""
    try:
        from revohuman.tracker.calibration import TrackerRuntime   # 惰性：需要 pysurvive
        extra = dict(extra_kwargs or {})
        hooks = {k: extra.pop(k) for k in _RUNTIME_HOOKS if k in extra}
        skw = dict(stamp_accuracy_ms=params["stamp_accuracy_ms"],
                   world_gate_mode=params["world_gate_mode"])
        if params.get("phi_pack_ms"):
            skw["phi_pack_ms"] = params["phi_pack_ms"]
        skw.update(extra)
        # 节点暂不接 SDK 的启动门与运行期守卫（几何门由 TrackerServices 1Hz 判）；零位沿用落盘值，
        # 否则 respawn（如标定后退出码 5 重拉）时会按当时的手势记下一个错的零位
        return TrackerRuntime(
            roles, registered=params["lighthouses"], state_dir=params["state_dir"],
            config_path=params["config_path"], gate_mode=params["geometry_gate_mode"],
            fence=params["fence"], survive_args=params["args"],
            liveness_gate_s=params["liveness_gate_s"], source_kwargs=skw,
            start_gate=False, guard_interval_s=0, restore=True, auto_reference=False, **hooks)
    except Exception as exc:  # noqa: BLE001 —— ImportError（没装 pysurvive）/ 参数不合法
        return _fail(node, params, exc)


def start_tracker_runtime(node, rt, params: dict,
                          before_start: Optional[Callable[[object], None]] = None) -> bool:
    """起采集。True = 起来了；False = auto 档下没起来（调用方拆掉 tracker 那半）；required 档抛出。
    ``before_start(src)`` 在采集开始前挂订阅：起完再订阅会丢掉开头的帧。"""
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
    """世界系变更（在册基站被搬动 / 外来基站被解进世界系）只打 ERROR 日志并记进
    ``tracker_status`` 的 ``world`` 段。"""
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
    """运行档的三个服务（``~/tracker/...``）与 1 Hz 几何门判定。

    ``set_reference``（Trigger）：双手摆在零位时调用，记零位并落盘，之后 ``tracker_delta*``
    话题开始出数；几何门 error 档下运行中几何失效 / 几何缺失 / 间距漂移时拒绝。
    ``get_status``（Trigger）：运行档状态 JSON。
    ``set_calibration_mode``（SetBool）：``true`` → 退出并由 launch 重拉成标定代。
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
        """推进一次几何门（1 Hz 调用），结论变化时打日志。"""
        try:
            geo = self.rt.geometry(advance=True)
        except Exception as exc:  # noqa: BLE001 —— 探测失败不拖垮采集
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
        except Exception as exc:  # noqa: BLE001 —— CalibrationRejected / ValueError（缺位姿）
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
        except Exception as exc:  # noqa: BLE001 —— CalibrationRejected（预检 / 在册基站为空）
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
    """把模式文件写成 ``calibrate``。在册基站为空、或信道预检不通过时抛 ``CalibrationRejected``。"""
    from revohuman.errors import CalibrationRejected
    from revohuman.tracker.calibration import session
    if not registered:
        raise CalibrationRejected("state", [
            "在册基站列表为空：把基站背面 FA 开头的序列号填进基站白名单"
            "（用的是哪份文件、缺什么：`revohuman-lighthouses show`）"])
    if require_preflight:
        session.require_preflight(state_dir, registered)
    session.write_mode(state_dir, "calibrate")


# ---------------------------------------------------------------- 标定代


class CalibrationGeneration:
    """标定代：本进程不连手套，只跑一轮基站几何标定，结束后置 ``node.exit_code = 5``。

    ``tracker_status`` 发 ``{"mode": "calibrate", "state": ..., "coverage": ..., "hint": ...}``，
    提示用户如何移动 tracker；``~/tracker/get_status`` 返回同一份 JSON。
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
        except Exception:  # noqa: BLE001 —— 收尾期 publisher 已销毁
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


# ---------------------------------------------------------------- 消息


def frame_stamp_ns(frame):
    """tracker 帧的 ROS 戳 = 采样时刻（CLOCK_REALTIME 纳秒），与关节戳同一时间轴。"""
    return C.frame_stamp_ns(frame)


def make_pose_stamped(stamp_ns: int, frame_id: str, pose) -> PoseStamped:
    """TrackerPose（米，四元数 (w,x,y,z)）→ geometry_msgs/PoseStamped（x,y,z,w）。"""
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
    """零位相对位姿的坐标系名：该手 tracker 在零位标定时刻的位姿。"""
    return f"revohuman_{hand}_tracker_zero"


def status_dict(src, sync=None, counters: Optional[dict] = None) -> dict:
    """采集源的 1 Hz 状态：判活与基站身份要看的字段。

    基站身份三项（``lighthouse_channels`` / ``foreign_lighthouses`` / ``missing_lighthouses``）
    只报不判：有邻居基站的房间里 ``foreign`` 常非空——看见外来信道 ≠ 已被隔离，
    有没有进解算以世界系的外来基站判定（``world`` 段）为准。"""
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
            # l1_tier 是最后一帧的瞬时值；判「整段是否都在 L1」读累计的 l1_by_tier
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
        # 世界系纪元：每段录制都该记下开始时的 epoch。pose_watch != "ok" 时探测器没装上，
        # 不能把「没报警」读成「没搬动」。
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
    """``tracker_status`` 的 JSON：采集源状态 + 运行档会话（零位 / 几何门 / 启动体检）。"""
    out = status_dict(src, sync, counters)
    if session is not None:
        out["mode"] = session.get("mode", "runtime")
        out["session"] = {k: v for k, v in session.items() if k != "mode"}
    return json.dumps(out, ensure_ascii=False, default=str)
