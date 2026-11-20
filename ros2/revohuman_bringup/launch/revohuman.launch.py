"""revohuman.launch.py: the **only** ROS2 entry point of the RevoHuman SDK.

    ros2 launch revohuman_bringup revohuman.launch.py                 # two hands (default)
    ros2 launch revohuman_bringup revohuman.launch.py mode:=left      # left hand only
    ros2 launch revohuman_bringup revohuman.launch.py tracker_mode:=off
    ros2 launch revohuman_bringup revohuman.launch.py allow_calibration:=true  # needed for ~/calibrate
    ros2 launch revohuman_bringup revohuman.launch.py product:=PTV1   # declare product model (default: auto)
    ros2 launch revohuman_bringup revohuman.launch.py button:=true    # two-hand button → button_event

`mode` decides which node is started; all other parameters are passed through:

    mode:=pair   → revohuman_ros/bimanual_node, namespace /revohuman/pair
    mode:=left   → revohuman_ros/glove_node  hand:=left, namespace /revohuman/left
    mode:=right  → revohuman_ros/glove_node  hand:=right, namespace /revohuman/right

Precedence: **command line > params_file > node defaults**. It is implemented as
"**empty string = no override**": every launch argument defaults to an empty string, and only the ones
you set explicitly go into the override dict, so values configured in the yaml are not silently replaced
by launch defaults.

Do not run `mode:=pair` together with two `mode:=left|right`: a glove can only be opened by one process
(the SDK holds an exclusive flock on the serial port); running both makes the one started later fail to
start, rather than both working.

**Exit code 5 = switch Tracker mode**: base station geometry calibration and run mode cannot be switched
within one process (the libsurvive ctx is a per-process singleton); after the node exits with code 5,
this file restarts it unchanged; other exit codes are not restarted.
To prevent endless restarts in a bad state, more than 10 restarts within 60 s stop the restarts with an error.
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, OpaqueFunction, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

import os
import time

# Node exit code: switch Tracker mode (calibration mode ↔ run mode); launch restarts the node
EXIT_CODE_MODE_SWITCH = 5

# (launch argument name, node parameter name, type). The type must **match** the type of the node's
# declare_parameter default, otherwise rclpy raises InvalidParameterTypeException at declaration
# (e.g. rate_hz written as 200 instead of 200.0).
_PASSTHROUGH = [
    # —— Glove
    ("rate_hz",                   "rate_hz",                   float),
    ("publish_tactile",           "publish_tactile",           str),
    ("require_tactile",           "require_tactile",           bool),
    ("allow_calibration",         "allow_calibration",         bool),
    # —— Product model (P1 / PTV1 identification)
    ("product",                   "product",                   str),
    ("product_rules_path",        "product_rules_path",        str),
    ("product_strict",            "product_strict",            bool),
    # —— Only used with mode:=pair (glove_node does not declare them; passing them is harmless,
    #    rclpy simply ignores undeclared overrides)
    ("button",                    "button",                    bool),
    # Split tactile (declared by both single-hand and two-hand nodes)
    ("split_tactile",             "split_tactile",             bool),
    ("pair_mode",                 "pair_mode",                 str),
    ("pair_accuracy_ms",          "pair_accuracy_ms",          float),
    ("tracker_pair_accuracy_ms",  "tracker_pair_accuracy_ms",  float),
    # —— Tracker
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
    # —— Base station geometry calibration and zero pose
    ("tracker_state_dir",             "tracker_state_dir",             str),
    ("tracker_config_path",           "tracker_config_path",           str),
    ("tracker_geometry_gate_mode",    "tracker_geometry_gate_mode",    str),
    ("tracker_fence",                 "tracker_fence",                 bool),
    ("tracker_require_preflight",     "tracker_require_preflight",     bool),
    ("tracker_calibration_timeout_s", "tracker_calibration_timeout_s", float),
    # —— Wrist camera (the three device/HUB items use camera_* for a single hand,
    #    camera_left_* / camera_right_* for two hands)
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
    ("camera_model",              "camera_model",              str),
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
    # mode: (executable, default namespace, extra fixed parameters)
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
    """Resolve the launch arguments into `(executable, namespace, overrides)`.

    `arg(name) -> str` is provided by the caller (in launch it is `LaunchConfiguration(...).perform`,
    in unit tests a table lookup). **Pure function, does not touch ROS**: the unit tests for both rules,
    mode dispatch and "empty string = no override", target this function."""
    mode = arg("mode")
    if mode not in _MODES:
        raise RuntimeError(f"mode 只能是 {'/'.join(_MODES)}，收到 {mode!r}")
    executable, default_ns, fixed = _MODES[mode]

    overrides = dict(fixed)
    for launch_name, node_name, kind in _PASSTHROUGH:
        raw = arg(launch_name)
        if raw == "":
            continue                    # empty string = no override; left to params_file / node defaults
        try:
            overrides[node_name] = _cast(raw, kind)
        except ValueError as exc:
            raise RuntimeError(f"launch 参数 {launch_name}:={raw!r} 解析失败：{exc}") from exc
    return executable, (arg("namespace") or default_ns), overrides


class RespawnPolicy:
    """Restart only on exit code 5; stop restarting after more than ``max_respawns`` within ``window_s``.
    Pure logic, unit tested directly."""

    def __init__(self, max_respawns: int = 10, window_s: float = 60.0, clock=time.monotonic):
        self.max_respawns = max_respawns
        self.window_s = window_s
        self._clock = clock
        self._history: list = []

    def decide(self, returncode) -> tuple:
        """→ ``(whether to restart, explanation)``."""
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
            # later entries override earlier ones: params_file is the base, the command-line override
            # dict is applied last
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
