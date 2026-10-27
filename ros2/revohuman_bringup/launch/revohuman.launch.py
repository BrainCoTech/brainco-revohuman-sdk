"""revohuman.launch.py —— RevoHuman SDK 的**唯一** ROS2 入口。

    ros2 launch revohuman_bringup revohuman.launch.py                 # 双手（默认）
    ros2 launch revohuman_bringup revohuman.launch.py mode:=left      # 只起左手
    ros2 launch revohuman_bringup revohuman.launch.py tracker_mode:=off
    ros2 launch revohuman_bringup revohuman.launch.py allow_calibration:=true  # 才有 ~/calibrate
    ros2 launch revohuman_bringup revohuman.launch.py product:=PTV1   # 声明型号（默认 auto 识别）

`mode` 决定拉起哪个节点，别的参数一律透传：

    mode:=pair   → revohuman_ros/bimanual_node，命名空间 /revohuman/pair
    mode:=left   → revohuman_ros/glove_node  hand:=left，命名空间 /revohuman/left
    mode:=right  → revohuman_ros/glove_node  hand:=right，命名空间 /revohuman/right

优先级 **命令行 > params_file > 节点默认值**。实现方式是「**空串 = 不覆盖**」：
每个 launch 参数的默认值都是空串，只有你显式写了才会进覆盖字典，
所以 yaml 里配好的值不会被 launch 的默认值悄悄顶掉。

`mode:=pair` 与两个 `mode:=left|right` 不要同时起：一只手套只能被一个进程打开
（SDK 侧对串口 flock 独占），双开的现象是后起的那个启动失败而不是两个都正常。

**退出码 5 = 切换 tracker 模式**：基站几何标定与运行档不能在同一进程里切换（libsurvive 的
ctx 是进程级单例），节点以退出码 5 退出后由本文件原样重拉；其它退出码不重拉。
为防止坏状态下无限重拉，60 s 内超过 10 次就停止重拉并报错。
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, OpaqueFunction, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

import os
import time

# 节点退出码：切换 tracker 模式（标定代 ↔ 运行档），launch 重拉
EXIT_CODE_MODE_SWITCH = 5

# (launch 参数名, 节点参数名, 类型)。类型必须与节点 declare_parameter 的默认值**同型**，
# 否则 rclpy 在声明时抛 InvalidParameterTypeException（例如 rate_hz 写 200 而不是 200.0）。
_PASSTHROUGH = [
    # —— 手套本体
    ("set_name",                  "set_name",                  str),
    ("registry_path",             "registry_path",             str),
    ("rate_hz",                   "rate_hz",                   float),
    ("publish_tactile",           "publish_tactile",           str),
    ("require_tactile",           "require_tactile",           bool),
    ("allow_calibration",         "allow_calibration",         bool),
    # —— 产品型号（P1 / PTV1 识别）
    ("product",                   "product",                   str),
    ("product_rules_path",        "product_rules_path",        str),
    ("product_strict",            "product_strict",            bool),
    # —— 只有 mode:=pair 用得上（glove_node 不声明，多传无害，rclpy 直接忽略未声明的覆盖）
    ("pair_mode",                 "pair_mode",                 str),
    ("pair_accuracy_ms",          "pair_accuracy_ms",          float),
    ("tracker_pair_accuracy_ms",  "tracker_pair_accuracy_ms",  float),
    # —— tracker
    ("tracker_mode",              "tracker_mode",              str),
    ("tracker_serial",            "tracker_serial",            str),
    ("tracker_left_serial",       "tracker_left_serial",       str),
    ("tracker_right_serial",      "tracker_right_serial",      str),
    ("tracker_lighthouses",       "tracker_lighthouses",       "str_list"),
    ("tracker_world_gate_mode",   "tracker_world_gate_mode",   str),
    ("tracker_stamp_accuracy_ms", "tracker_stamp_accuracy_ms", float),
    ("tracker_liveness_gate_s",   "tracker_liveness_gate_s",   float),
    ("tracker_frame_id",          "tracker_frame_id",          str),
    ("tracker_phi_pack_ms",       "tracker_phi_pack_ms",       "str_list"),
    ("tracker_survive_args",      "tracker_survive_args",      "str_list"),
    # —— 基站几何标定与零位
    ("tracker_state_dir",             "tracker_state_dir",             str),
    ("tracker_config_path",           "tracker_config_path",           str),
    ("tracker_geometry_gate_mode",    "tracker_geometry_gate_mode",    str),
    ("tracker_fence",                 "tracker_fence",                 bool),
    ("tracker_require_preflight",     "tracker_require_preflight",     bool),
    ("tracker_calibration_timeout_s", "tracker_calibration_timeout_s", float),
    # —— 腕部相机（设备/HUB 三项单手用 camera_*、双手用 camera_left_* / camera_right_*）
    ("camera_mode",               "camera_mode",               str),
    ("camera_device",             "camera_device",             str),
    ("camera_meta_device",        "camera_meta_device",        str),
    ("camera_usb_hub",            "camera_usb_hub",            str),
    ("camera_left_device",        "camera_left_device",        str),
    ("camera_left_meta_device",   "camera_left_meta_device",   str),
    ("camera_left_usb_hub",       "camera_left_usb_hub",       str),
    ("camera_right_device",       "camera_right_device",       str),
    ("camera_right_meta_device",  "camera_right_meta_device",  str),
    ("camera_right_usb_hub",      "camera_right_usb_hub",      str),
    ("camera_width",              "camera_width",              int),
    ("camera_height",             "camera_height",             int),
    ("camera_fps",                "camera_fps",                int),
    ("camera_fourcc",             "camera_fourcc",             str),
    ("camera_exposure",           "camera_exposure",           int),
    ("camera_buffers",            "camera_buffers",            int),
    ("camera_use_metadata",       "camera_use_metadata",       bool),
    ("camera_zero_copy",          "camera_zero_copy",          bool),
    ("camera_frame_id",           "camera_frame_id",           str),
    ("camera_pts_offset_ms",      "camera_pts_offset_ms",      str),
    ("camera_readout_ms",         "camera_readout_ms",         str),
    ("camera_controls",           "camera_controls",           "str_list"),
    ("camera_strict_controls",    "camera_strict_controls",    bool),
]

_MODES = {
    # mode: (可执行文件, 默认命名空间, 额外固定参数)
    "pair":  ("bimanual_node", "/revohuman/pair",  {}),
    "left":  ("glove_node",    "/revohuman/left",  {"hand": "left"}),
    "right": ("glove_node",    "/revohuman/right", {"hand": "right"}),
}


def _cast(raw: str, kind):
    if kind is bool:
        low = raw.strip().lower()
        if low not in ("true", "false", "1", "0", "yes", "no"):
            raise ValueError(f"期望布尔值，收到 {raw!r}")
        return low in ("true", "1", "yes")
    if kind == "str_list":
        return [x.strip() for x in raw.split(",") if x.strip()]
    return kind(raw)


def resolve_node_spec(arg):
    """把 launch 参数解析成 `(executable, namespace, overrides)`。

    `arg(name) -> str` 由调用方提供（launch 里是 `LaunchConfiguration(...).perform`，
    单测里是查表）。**纯函数，不碰 ROS**——mode 分派与「空串=不覆盖」这两条
    规则的单测都打在这上面。"""
    mode = arg("mode")
    if mode not in _MODES:
        raise RuntimeError(f"mode 只能是 {'/'.join(_MODES)}，收到 {mode!r}")
    executable, default_ns, fixed = _MODES[mode]

    overrides = dict(fixed)
    for launch_name, node_name, kind in _PASSTHROUGH:
        raw = arg(launch_name)
        if raw == "":
            continue                    # 空串 = 不覆盖，交给 params_file / 节点默认值
        try:
            overrides[node_name] = _cast(raw, kind)
        except ValueError as exc:
            raise RuntimeError(f"launch 参数 {launch_name}:={raw!r} 解析失败：{exc}") from exc
    return executable, (arg("namespace") or default_ns), overrides


class RespawnPolicy:
    """退出码 5 才重拉；``window_s`` 内超过 ``max_respawns`` 次就不再重拉。纯逻辑，单测直接打。"""

    def __init__(self, max_respawns: int = 10, window_s: float = 60.0, clock=time.monotonic):
        self.max_respawns = max_respawns
        self.window_s = window_s
        self._clock = clock
        self._history: list = []

    def decide(self, returncode) -> tuple:
        """→ ``(是否重拉, 说明)``。"""
        if returncode != EXIT_CODE_MODE_SWITCH:
            return False, f"退出码 {returncode}，不重拉"
        now = self._clock()
        self._history = [t for t in self._history if now - t < self.window_s]
        if len(self._history) >= self.max_respawns:
            return False, (f"{self.window_s:.0f} s 内已重拉 {len(self._history)} 次，停止重拉；"
                           f"检查 tracker 状态目录里的 survive_mode 与节点日志")
        self._history.append(now)
        return True, "切换 tracker 模式，重拉节点"


def _node_with_respawn(make_node, policy: RespawnPolicy) -> list:
    node = make_node()

    def on_exit(event, _context):
        ok, why = policy.decide(event.returncode)
        if not ok:
            return [LogInfo(msg=f"[revohuman] {why}")] if event.returncode == EXIT_CODE_MODE_SWITCH else None
        return [LogInfo(msg=f"[revohuman] {why}")] + _node_with_respawn(make_node, policy)

    return [node, RegisterEventHandler(OnProcessExit(target_action=node, on_exit=on_exit))]


def _launch_setup(context, *_a, **_kw):
    def arg(name):
        return LaunchConfiguration(name).perform(context).strip()

    executable, namespace, overrides = resolve_node_spec(arg)

    params_file = arg("params_file")
    if not params_file:
        params_file = os.path.join(
            get_package_share_directory("revohuman_bringup"), "config", "revohuman.yaml")
    if not os.path.isfile(params_file):
        raise RuntimeError(f"params_file 不存在：{params_file}")
    log_level = arg("log_level")

    def make_node():
        return Node(
            package="revohuman_ros",
            executable=executable,
            name=executable,
            namespace=namespace,
            output="screen",
            emulate_tty=True,
            arguments=["--ros-args", "--log-level", log_level],
            # 后者盖前者：params_file 先铺底，命令行覆盖字典最后落
            parameters=[params_file, overrides],
        )

    return _node_with_respawn(make_node, RespawnPolicy())


def generate_launch_description() -> LaunchDescription:
    args = [
        DeclareLaunchArgument("mode", default_value="pair",
                              choices=list(_MODES),
                              description="pair=双手 bimanual_node；left/right=单手 glove_node"),
        DeclareLaunchArgument("namespace", default_value="",
                              description="覆盖默认命名空间（默认 /revohuman/{pair,left,right}）"),
        DeclareLaunchArgument("params_file", default_value="",
                              description="参数 yaml（默认用本包 config/revohuman.yaml）"),
        DeclareLaunchArgument("log_level", default_value="info",
                              description="节点日志级别 debug|info|warn|error"),
    ]
    args += [DeclareLaunchArgument(name, default_value="",
                                   description="留空 = 用 params_file / 节点默认值")
             for name, _n, _k in _PASSTHROUGH]
    return LaunchDescription(args + [OpaqueFunction(function=_launch_setup)])
