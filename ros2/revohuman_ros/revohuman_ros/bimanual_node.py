"""bimanual_node — both hands, hardware-synchronized (ns=/revohuman/pair).

Topics (all under the node namespace):

- ``joint_states`` (left 21 + right 21 = 42 joints; ``header.stamp`` is the sampling time of the left-hand
  frame)
- ``tactile_left`` / ``tactile_right``: created only if both hands have tactile; when either side is missing
  tactile for a cycle, the whole cycle (including ``joint_states``) is not published, and publishing resumes
  automatically after recovery
- ``status`` (String+JSON, 1 Hz): per-hand capture counters and capture gate edges, pairing statistics,
  product model
- ``tracker_pose_left`` / ``tracker_pose_right`` / ``tracker_delta_left`` / ``tracker_delta_right`` /
  ``tracker_status``: when trackers are present (trackers and gloves are paired separately; downstream aligns
  them by ``header.stamp``)
- ``wrist_camera_{left,right}/image/compressed`` / ``wrist_camera_status``: when wrist cameras are present
- ``button_event`` (String+JSON): with ``button:=true``, one message per valid glove button press (see below)

The two hands have different product models → startup fails. Pairs that do not reach ``pair_accuracy_ms`` are
not produced; an unsatisfiable budget raises ``ValueError`` at startup. Services ``left/…`` and ``right/…``,
one set each.

Glove buttons: ``button`` (bool, default false, consistent with SDK ``connect_pair(button=False)``). When
enabled, both gloves request in-stream button reporting when their streams open, and each valid press
publishes one ``button_event`` (JSON: ``hand``, ``mode`` (``start`` / ``end``, the glove-side state after
toggling, for reference only), ``press_sequence``, ``press_count``, ``missed_presses``, ``recv_wall_ns``,
``sample_wall_ns``, ``seq``, ``generation``, ``event_id``). QoS = RELIABLE / KEEP_LAST(10) / **VOLATILE**,
deliberately not latched: a subscriber that arrives late or reconnects does not receive old presses, so a UI
refresh does not replay the previous button press as a start / stop. If the firmware does not support it,
capture continues normally, just without events (``button.<hand>.supported`` in ``status`` is false). The
node only forwards events and does not touch recording: start / stop is decided by the subscriber.
"""

from __future__ import annotations

import json
import sys
import threading
import time
from collections import deque

import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import CompressedImage, JointState
from std_msgs.msg import String

from revohuman import GloveManager, Handedness, TactileUndetermined, UnsupportedFeatureError
from revohuman_msgs.msg import TactileArray

from . import _camera as K
from . import _common as C
from . import _product as P
from . import _tracker as T
from .glove_node import shutdown_node, spin_until_exit

# button events: reliable delivery, not latched (see the module docstring). One message per press; a depth of
# 10 is enough to cover both hands pressing at once
BUTTON_EVENT_QOS = QoSProfile(
    reliability=ReliabilityPolicy.RELIABLE,
    history=HistoryPolicy.KEEP_LAST,
    depth=10,
    durability=DurabilityPolicy.VOLATILE,
)


class BimanualNode(Node):
    def __init__(self, *, connect_kwargs: dict | None = None, tracker_kwargs: dict | None = None,
                 camera_kwargs: dict | None = None, calibration_kwargs: dict | None = None,
                 manager_kwargs: dict | None = None,
                 node_name: str = "bimanual_node", namespace: str | None = "/revohuman/pair",
                 **node_kwargs):
        super().__init__(node_name, namespace=namespace, **node_kwargs)
        # when not None, main() leaves the main loop and uses it as the process exit code (5 = switch tracker
        # mode, relaunched by launch)
        self.exit_code = None
        self.tracker_rt = None
        self.tracker_calibration = None
        self.tracker_services = None
        self.product_identity = None
        C.install_sdk_log_bridge(self)
        try:
            self._init(connect_kwargs, tracker_kwargs, camera_kwargs, calibration_kwargs,
                       manager_kwargs)
        except Exception:
            self.destroy_node()
            raise

    def _init(self, connect_kwargs: dict | None, tracker_kwargs: dict | None,
              camera_kwargs: dict | None = None, calibration_kwargs: dict | None = None,
              manager_kwargs: dict | None = None) -> None:
        self.rate_hz = float(self.declare_parameter("rate_hz", 200.0).value)
        self.pair_mode = C.check_choice(
            "pair_mode", self.declare_parameter("pair_mode", "strict").value, C.PAIR_MODES)
        # pairing accuracy budget (milliseconds). Negative = follow pair_mode: strict → 0.0 (only accept pairs
        # from the same sampling instant under hardware sync); best_effort → 1.0 (allows the absolute
        # accuracies of the two sides to add up)
        pa = float(self.declare_parameter("pair_accuracy_ms", -1.0).value)
        self.pair_accuracy_ms = pa if pa >= 0 else (0.0 if self.pair_mode == "strict" else 1.0)
        # the trackers are two independently clocked IMUs, so this does not follow pair_mode;
        # negative = no budget
        tpa = float(self.declare_parameter("tracker_pair_accuracy_ms", 1.0).value)
        self.tracker_pair_accuracy_ms = tpa if tpa >= 0 else None
        self.publish_tactile = C.check_choice(
            "publish_tactile", self.declare_parameter("publish_tactile", "auto").value, C.PUBLISH_TACTILE)
        self.require_tactile = bool(self.declare_parameter("require_tactile", False).value)
        self.allow_calibration = bool(self.declare_parameter("allow_calibration", False).value)
        # in-stream button reporting: with the default false, stream opening and frame reading are byte-for-byte
        # identical to running without buttons; if the firmware does not support it, capture continues normally
        self.button_enabled = bool(self.declare_parameter("button", False).value)
        # split tactile (default false): when enabled and supported by the firmware, tactile is published per
        # sample (module rate, about 215Hz; header.stamp = the sample's sampling time, frame_stamp no longer
        # matches a joint frame), joints stay at 200Hz; a hand whose firmware lacks support automatically uses
        # combined frames
        self.split_tactile = bool(self.declare_parameter("split_tactile", False).value)
        left_port = self.declare_parameter("left_port", "").value or None
        right_port = self.declare_parameter("right_port", "").value or None
        sysfs_root = self.declare_parameter("sysfs_root", "").value or None
        self.frame_id = self.declare_parameter("frame_id", "revohuman_pair").value
        # manager_kwargs is only for injection by unit tests
        mgr = GloveManager(sysfs_root=sysfs_root, **(manager_kwargs or {}))
        query = T.query_devices_enabled(connect_kwargs, manager_kwargs)
        self.product_params = P.declare_product_params(self)
        self.tracker_params = T.declare_tracker_params(self)
        self.camera_params = K.declare_camera_params(self)
        self.camera_hands = {hand: K.declare_camera_hand_params(self, f"camera_{hand}_")
                             for hand in ("left", "right")}
        mode = self.tracker_params["mode"]

        # before resolving, query handedness from any present glove not queried yet (on recent firmware the
        # serial number carries no handedness); the left-hand call already queries all present gloves
        def _resolver(hand):
            return lambda: mgr.resolve_tracker(hand, query_devices=query)

        self.tracker_left_serial = T.require_serial(
            self, "tracker_left_serial", mode, resolver=_resolver(Handedness.LEFT))
        self.tracker_right_serial = T.require_serial(
            self, "tracker_right_serial", mode, resolver=_resolver(Handedness.RIGHT))
        if mode != "off" and self.tracker_left_serial == self.tracker_right_serial:
            raise ValueError("tracker_left_serial 与 tracker_right_serial 不能是同一只")

        # —— base station geometry calibration generation: this process does not connect the gloves and exits
        #    with exit code 5 when calibration ends ——
        if mode != "off" and T.calibration_pending(self.tracker_params):
            self.tracker_calibration = T.CalibrationGeneration(
                self, {self.tracker_left_serial: Handedness.LEFT,
                       self.tracker_right_serial: Handedness.RIGHT},
                self.tracker_params, calibration_kwargs)
            return

        kwargs = dict(
            rate_hz=self.rate_hz, require_tactile=self.require_tactile,
            sync=True if self.pair_mode == "strict" else "best-effort",
            button=self.button_enabled,
            split_tactile=self.split_tactile,
        )
        kwargs.update(connect_kwargs or {})
        # VariantMismatch / TactileUnsupported / TactileUndetermined are raised right here → startup fails
        self.pair = mgr.connect_pair(left_port=left_port, right_port=right_port, **kwargs)
        try:
            T.confirm_serial(self, "tracker_left_serial", self.tracker_left_serial,
                             self.pair.left.unit_id, sysfs_root=sysfs_root)
            T.confirm_serial(self, "tracker_right_serial", self.tracker_right_serial,
                             self.pair.right.unit_id, sysfs_root=sysfs_root)
        except Exception:
            self.pair.close()
            raise

        # —— product model: decides which sensors are enabled by default ——
        self.product_identity = P.identify_connected(
            self, self.product_params,
            {Handedness.LEFT: self.pair.left, Handedness.RIGHT: self.pair.right},
            sysfs_root=sysfs_root)
        self.camera_params["mode"] = P.resolve_camera_mode(self.camera_params["mode"],
                                                           self.product_identity)
        K.check_camera_params(self.camera_params)
        P.check_sensors(self, self.product_params, self.product_identity,
                        tactile=self.publish_tactile == "auto",
                        camera_mode=self.camera_params["mode"])
        hints = {hand: P.camera_hint(self.product_identity, hand) for hand in ("left", "right")}
        for hand, hint in hints.items():
            if hint.get("usb_ids") and not self.camera_hands[hand].get("usb_ids"):
                self.camera_hands[hand]["usb_ids"] = hint["usb_ids"]
        self._camera_hub_resolver = lambda hand: (
            lambda: hints[hand].get("usb_hub") or K.resolve_hand_hub(
                hand, sysfs_root=sysfs_root, warn=self.get_logger().warning))

        self.names = C.joint_names("left") + C.joint_names("right")
        self.pub_joints = self.create_publisher(JointState, "joint_states", C.DATA_QOS)
        self.pub_tactile_left = self.pub_tactile_right = None
        both_tactile = False
        if self.publish_tactile == "auto":
            try:
                both_tactile = bool(self.pair.left.has_tactile) and bool(self.pair.right.has_tactile)
            except TactileUndetermined as exc:
                self.pair.close()
                raise RuntimeError(f"触觉型号未判定，拒绝启动：{exc}") from exc
            if both_tactile:
                self.pub_tactile_left = self.create_publisher(TactileArray, "tactile_left", C.DATA_QOS)
                self.pub_tactile_right = self.create_publisher(TactileArray, "tactile_right", C.DATA_QOS)
        self._tactile_gate_open = True      # paired tactile gate, current state (kept by _on_gate edge callback)
        # bimanual sampling alignment warnings (skew beyond budget / recovered): the callback fires in the SDK's
        # frame-input thread and only records; status carries them out every second. Must be set up before the
        # synchronizer: frames may arrive as soon as the subscription is created.
        self._skew_lock = threading.Lock()
        self.skew_alarms = 0
        self._skew_recent: deque = deque(maxlen=8)
        # tactile gate = both hands have tactile and tactile is to be published; with publish_tactile=off the
        # gate is not enabled (joints are still published)
        self.syn = self.pair.synchronizer(
            mode="interpolate", tactile_gate=both_tactile,
            pair_accuracy_ms=self.pair_accuracy_ms,
            on_tactile_gate=self._on_gate,
            on_skew_alarm=self._on_skew_alarm)
        self.get_logger().info(
            f"left={self.pair.left.port} right={self.pair.right.port} "
            f"触觉型号={self.pair.left.tactile_state.value}/{self.pair.right.tactile_state.value} "
            f"tactile_topics={'on' if both_tactile else 'off'} pair_mode={self.pair_mode} "
            f"synchronized={self.pair.synchronized} pair_accuracy_ms={self.pair_accuracy_ms} "
            f"calibration_srv={'on' if self.allow_calibration else 'off'} "
            f"button={'on' if self.button_enabled else 'off'} "
            f"split_tactile={'on' if self.split_tactile else 'off'}")

        self.pub_status = self.create_publisher(String, "status", C.STATUS_QOS)
        self._status_timer = self.create_timer(1.0, self._publish_status)

        self.pairs_published = 0
        self.pairs_no_stamp = 0
        self.tactile_published = 0
        # the pairing thread and both hands' split dispatch threads all increment tactile_published;
        # += is not atomic
        self._count_lock = threading.Lock()
        # split tactile: number of tactile messages published per sample (per hand); the combined-frame path is
        # still counted in tactile_published (once per pair)
        self.tactile_samples_published = {"left": 0, "right": 0}
        self._tactile_sample_subs = {}
        if self.split_tactile and self.pub_tactile_left is not None:
            for hand, glove in (("left", self.pair.left), ("right", self.pair.right)):
                self._tactile_sample_subs[hand] = glove.subscribe_tactile(
                    callback=lambda sample, h=hand: self._on_tactile_sample(h, sample))
        self._tare_group = ReentrantCallbackGroup()
        self._services = (
            C.register_glove_services(self, self.pair.left, "left/",
                                      allow_calibration=self.allow_calibration,
                                      tare_callback_group=self._tare_group)
            + C.register_glove_services(self, self.pair.right, "right/",
                                        allow_calibration=self.allow_calibration,
                                        tare_callback_group=self._tare_group))
        # tactile tare state (latched, published on change plus once at startup): the callbacks of the two hands
        # come in from their own reader threads and only queue; publishing is done serially in the executor
        self._tare_state = C.TareStatePublisher(self, {"left": self.pair.left, "right": self.pair.right})
        for glove in (self.pair.left, self.pair.right):
            glove.on_tactile_tare = self._on_tactile_tare
        self._tare_state.changed()
        self._sub = self.syn.subscribe(callback=self._on_pair)

        # —— buttons: only forward events; the subscriber decides start / stop from recording state ——
        self.pub_button = None
        self._button_subs = {}
        self.button_published = 0
        self.button_publish_errors = 0
        self._button_lock = threading.Lock()
        if self.button_enabled:
            self._button_bringup()

        # —— tracker: same process, same time axis ——
        self.tracker_src = self.tracker_syn = None
        self._tracker_sub = self._tracker_timer = None
        self.pub_tracker_left = self.pub_tracker_right = self.pub_tracker_status = None
        self.pub_tracker_delta_left = self.pub_tracker_delta_right = None
        self.tracker_pairs_published = 0
        self.tracker_delta_pairs_published = 0
        self.tracker_pairs_no_stamp = 0
        if mode != "off":
            self._tracker_bringup(tracker_kwargs)

        # —— wrist cameras: one per hand, no left/right pairing ——
        self.cameras = {}
        self._camera_subs = {}
        self.pub_camera = {}
        self.pub_camera_status = None
        self._camera_timer = None
        self.camera_published = {"left": 0, "right": 0}
        self.camera_publish_errors = {"left": 0, "right": 0}
        fid = self.camera_params["frame_id"]
        self.camera_frame_ids = {hand: (f"{fid}_{hand}" if fid else f"revohuman_{hand}_wrist_camera")
                                 for hand in ("left", "right")}
        if self.camera_params["mode"] != "off":
            self._camera_bringup(camera_kwargs)

    # ------------------------------------------------------------ buttons

    def _button_bringup(self) -> None:
        self.pub_button = self.create_publisher(String, "button_event", BUTTON_EVENT_QOS)
        for hand, glove in (("left", self.pair.left), ("right", self.pair.right)):
            try:
                self._button_subs[hand] = glove.button.subscribe(
                    callback=(lambda ev, h=hand: self._on_button(h, ev)))
            except UnsupportedFeatureError as exc:
                # the firmware was found not to support it when the stream opened: this hand has no button
                # events, data continues normally
                self.get_logger().warning(f"{hand} 手套按键不可用：{exc}")

    # runs in the callback thread of this hand's button subscription (the SDK uses one thread per
    # subscription); the two hands may come in concurrently
    def _on_button(self, hand: str, ev) -> None:
        with self._button_lock:                 # event_id monotonic, unique: hand callbacks are concurrent
            self.button_published += 1
            event_id = self.button_published
        body = {
            "hand": hand, "mode": ev.mode.value, "press_sequence": ev.press_sequence,
            "press_count": ev.press_count, "missed_presses": ev.missed_presses,
            "recv_wall_ns": ev.recv_wall_ns, "sample_wall_ns": ev.sample_wall_ns,
            "seq": ev.seq, "generation": ev.generation,
            # node-local monotonic number: subscribers deduplicate by it (the same message is not processed twice
            # because of a reconnect)
            "event_id": event_id,
        }
        msg = String()
        msg.data = json.dumps(body, ensure_ascii=False)
        try:
            self.pub_button.publish(msg)
        except Exception as exc:  # noqa: BLE001 —— publisher already destroyed during shutdown, etc.
            with self._button_lock:
                self.button_publish_errors += 1
            self.get_logger().warning(f"{hand} 按键事件发布失败：{exc!r}", throttle_duration_sec=5.0)
            return
        level = self.get_logger().warning if ev.missed_presses else self.get_logger().info
        level(f"按键 {hand}：{ev.mode.value} press_sequence={ev.press_sequence} "
              f"press_count={ev.press_count} event_id={event_id}")

    def _button_status(self) -> dict:
        body = {"enabled": self.button_enabled, "published": self.button_published,
                "publish_errors": self.button_publish_errors}
        for hand, glove in (("left", self.pair.left), ("right", self.pair.right)):
            b = glove.button
            sub = self._button_subs.get(hand)
            body[hand] = {"supported": b.supported, "events_total": b.events_total,
                          "missed_presses_total": b.missed_presses_total,
                          "subscribed": sub is not None and not sub.closed}
        return body

    # ------------------------------------------------------------ tracker

    def _tracker_bringup(self, tracker_kwargs) -> None:
        rt = T.create_tracker_runtime(
            self, {"left": self.tracker_left_serial, "right": self.tracker_right_serial},
            self.tracker_params, tracker_kwargs)
        if rt is None:
            return
        self.tracker_rt = rt
        self.pub_tracker_left = self.create_publisher(PoseStamped, "tracker_pose_left", C.DATA_QOS)
        self.pub_tracker_right = self.create_publisher(PoseStamped, "tracker_pose_right", C.DATA_QOS)
        self.pub_tracker_delta_left = self.create_publisher(PoseStamped, "tracker_delta_left", C.DATA_QOS)
        self.pub_tracker_delta_right = self.create_publisher(PoseStamped, "tracker_delta_right", C.DATA_QOS)
        self.pub_tracker_status = self.create_publisher(String, "tracker_status", T.STATUS_QOS)

        def attach(src):
            from revohuman.tracker import BimanualTrackerSynchronizer
            T.install_world_logger(self, src)
            self.tracker_syn = BimanualTrackerSynchronizer(
                src.hand(Handedness.LEFT), src.hand(Handedness.RIGHT), mode="interpolate",
                pair_accuracy_ms=self.tracker_pair_accuracy_ms)
            # subscribe before capture starts: subscribing after it has started only receives the tail
            self._tracker_sub = self.tracker_syn.subscribe(callback=self._on_tracker_pair)

        if not T.start_tracker_runtime(self, rt, self.tracker_params, before_start=attach):
            self._tracker_teardown()
            return
        self.tracker_src = rt.source
        self.tracker_services = T.TrackerServices(self, rt, self.tracker_params)
        self._tracker_timer = self.create_timer(1.0, self._tracker_tick)
        self.get_logger().info(
            f"tracker left={self.tracker_left_serial} right={self.tracker_right_serial} "
            f"lighthouses={list(rt.registered)} frame_id={self.tracker_params['frame_id']}")

    def _tracker_teardown(self) -> None:
        for closer in (lambda: self.tracker_calibration.close(),
                       lambda: self.tracker_services.close(),
                       lambda: self._tracker_sub.close(), lambda: self.tracker_syn.close(),
                       lambda: self.tracker_rt.close(),
                       lambda: self.destroy_timer(self._tracker_timer),
                       lambda: self.destroy_publisher(self.pub_tracker_left),
                       lambda: self.destroy_publisher(self.pub_tracker_right),
                       lambda: self.destroy_publisher(self.pub_tracker_delta_left),
                       lambda: self.destroy_publisher(self.pub_tracker_delta_right),
                       lambda: self.destroy_publisher(self.pub_tracker_status)):
            try:
                closer()
            except Exception:  # noqa: BLE001
                pass
        self.tracker_calibration = self.tracker_services = None
        self.tracker_src = self.tracker_rt = self.tracker_syn = None
        self._tracker_sub = self._tracker_timer = None
        self.pub_tracker_left = self.pub_tracker_right = self.pub_tracker_status = None
        self.pub_tracker_delta_left = self.pub_tracker_delta_right = None

    # runs in the callback thread of the tracker synchronizer's Subscription
    def _on_tracker_pair(self, pf) -> None:
        if pf.left.pose is None or pf.right.pose is None:
            return
        sl = T.frame_stamp_ns(pf.left)
        sr = T.frame_stamp_ns(pf.right)
        if sl is None or sr is None:
            self.tracker_pairs_no_stamp += 1
            return
        fid = self.tracker_params["frame_id"]
        self.pub_tracker_left.publish(T.make_pose_stamped(sl, fid, pf.left.pose))
        self.pub_tracker_right.publish(T.make_pose_stamped(sr, fid, pf.right.pose))
        self.tracker_pairs_published += 1
        rt = self.tracker_rt
        if rt is None or self.pub_tracker_delta_left is None:
            return
        # zero-pose-relative poses of both hands are published together or not at all: if only one hand was
        # calibrated (cannot happen) or a pose is invalid, the whole pair is not published
        ref = rt.reference
        dl = ref.delta("left", pf.left.pose)
        dr = ref.delta("right", pf.right.pose)
        if dl is None or dr is None:
            return
        self.pub_tracker_delta_left.publish(T.make_pose_stamped(sl, T.delta_frame_id("left"), dl))
        self.pub_tracker_delta_right.publish(T.make_pose_stamped(sr, T.delta_frame_id("right"), dr))
        self.tracker_delta_pairs_published += 1

    def _tracker_tick(self) -> None:
        if self.tracker_services is None:
            return
        self.tracker_services.evaluate_geometry()
        msg = String()
        msg.data = T.status_payload(
            self.tracker_src, sync=self.tracker_syn,
            counters={"tracker_pairs_published": self.tracker_pairs_published,
                      "tracker_delta_pairs_published": self.tracker_delta_pairs_published,
                      "tracker_pairs_no_stamp": self.tracker_pairs_no_stamp,
                      "pair_mode": self.pair_mode,
                      "pair_accuracy_ms": self.pair_accuracy_ms,
                      "tracker_pair_accuracy_ms": self.tracker_pair_accuracy_ms},
            session=self.tracker_services.status())
        self.pub_tracker_status.publish(msg)

    # ------------------------------------------------------------ wrist camera

    def _camera_bringup(self, camera_kwargs) -> None:
        for hand in ("left", "right"):
            cam = K.create_camera(self, self.camera_params, self.camera_hands[hand], hand,
                                  hub_resolver=self._camera_hub_resolver(hand), extra_kwargs=camera_kwargs)
            if cam is None:
                continue        # auto mode: no camera for this hand, the other one still starts
            pub = self.create_publisher(CompressedImage, f"wrist_camera_{hand}/image/compressed", C.DATA_QOS)
            sub = cam.subscribe(callback=(lambda frame, h=hand: self._on_camera_frame(h, frame)),
                                loaned=self.camera_params["zero_copy"])
            if not K.start_camera(self, cam, self.camera_params, hand):
                for closer in (sub.close, cam.stop, lambda: self.destroy_publisher(pub)):
                    try:
                        closer()
                    except Exception:  # noqa: BLE001
                        pass
                continue
            self.cameras[hand] = cam
            self._camera_subs[hand] = sub
            self.pub_camera[hand] = pub
            self.get_logger().info(
                f"wrist_camera_{hand}={cam.resolved.get('device')} meta={cam.resolved.get('meta_device')} "
                f"hub={cam.resolved.get('hub')} {self.camera_params['width']}x{self.camera_params['height']} "
                f"{self.camera_params['fourcc']}@{self.camera_params['fps']} "
                f"zero_copy={'on' if self.camera_params['zero_copy'] else 'off'} "
                f"frame_id={self.camera_frame_ids[hand]}")
        if self.cameras:
            self.pub_camera_status = self.create_publisher(String, "wrist_camera_status", T.STATUS_QOS)
            self._camera_timer = self.create_timer(1.0, self._publish_camera_status)

    def _camera_teardown(self) -> None:
        subs = list(getattr(self, "_camera_subs", {}).values())
        cams = list(getattr(self, "cameras", {}).values())
        pubs = list(getattr(self, "pub_camera", {}).values())
        closers = ([s.close for s in subs] + [c.stop for c in cams]
                   + [lambda: self.destroy_timer(self._camera_timer)]
                   + [(lambda p=p: self.destroy_publisher(p)) for p in pubs]
                   + [lambda: self.destroy_publisher(self.pub_camera_status)])
        for closer in closers:
            try:
                closer()
            except Exception:  # noqa: BLE001
                pass
        self.cameras, self._camera_subs, self.pub_camera = {}, {}, {}
        self._camera_timer = self.pub_camera_status = None

    # runs in the dispatch thread of this hand's camera subscription. A zero-copy frame must be returned whether
    # or not publishing succeeded
    def _on_camera_frame(self, hand: str, frame) -> None:
        try:
            self.pub_camera[hand].publish(K.make_compressed_image(frame, self.camera_frame_ids[hand]))
            self.camera_published[hand] += 1
        except Exception as exc:  # noqa: BLE001 —— publisher already destroyed during shutdown, etc.
            self.camera_publish_errors[hand] += 1
            self.get_logger().warning(f"{hand} 相机帧发布失败：{exc!r}", throttle_duration_sec=5.0)
        finally:
            release = getattr(frame, "release", None)
            if release is not None:
                release()

    def _publish_camera_status(self) -> None:
        body = {"zero_copy": self.camera_params["zero_copy"]}
        for hand, cam in self.cameras.items():
            body[hand] = K.status_dict(
                cam, counters={"camera_published": self.camera_published[hand],
                               "camera_publish_errors": self.camera_publish_errors[hand],
                               "frame_id": self.camera_frame_ids[hand]})
        msg = String()
        msg.data = json.dumps(body, ensure_ascii=False, default=str)
        self.pub_camera_status.publish(msg)

    # ------------------------------------------------------------ glove

    def _on_gate(self, is_open: bool, reason) -> None:
        self._tactile_gate_open = bool(is_open)     # split tactile: all-or-nothing check for per-sample tactile
        if is_open:
            self.get_logger().info("触觉门重开：合并话题恢复发布")
        else:
            self.get_logger().warning(f"触觉门关闭（{reason}）：joint_states 与两条触觉话题整拍停发")

    def _on_skew_alarm(self, msg: str) -> None:
        # the SDK side already logs this (N2205 WARNING / N2206 INFO); not repeated here
        with self._skew_lock:
            self.skew_alarms += 1
            self._skew_recent.append({"wall_ns": time.time_ns(), "msg": msg})

    def _skew_status(self) -> dict:
        with self._skew_lock:
            return {"count": self.skew_alarms, "recent": list(self._skew_recent)}

    def _on_pair(self, pf) -> None:
        # accuracy filtering happens at the SDK's pairing output (pair_accuracy_ms); every pair here meets it
        stamp = C.frame_stamp_ns(pf.left)      # the merged frame's stamp is the left hand's sampling time
        if stamp is None:
            self.pairs_no_stamp += 1
            return
        if self.pub_tactile_left is not None:
            # the tactile gate already guarantees both sides are present; check again here, and if either side is
            # unavailable the whole cycle is not published (never half a set)
            if not (C.tactile_usable(pf.left.tactile) and C.tactile_usable(pf.right.tactile)):
                return
        self.pub_joints.publish(C.make_joint_state(
            stamp, self.frame_id, self.names,
            C.joint_positions_rad(pf.left) + C.joint_positions_rad(pf.right)))
        self.pairs_published += 1
        if self.pub_tactile_left is not None:
            # for a split hand, tactile goes through _on_tactile_sample (per sample); the tactile on the frame is
            # only the latest state and is not published
            sent = False
            if self.pair.left.tactile_transport != "split":
                self.pub_tactile_left.publish(
                    C.make_tactile(pf.left, pf.left.tactile, stamp, self.frame_id))
                sent = True
            if self.pair.right.tactile_transport != "split":
                self.pub_tactile_right.publish(
                    C.make_tactile(pf.right, pf.right.tactile, stamp, self.frame_id))
                sent = True
            if sent:
                with self._count_lock:
                    self.tactile_published += 1

    # runs in each glove's dispatch thread (split tactile); an rclpy publisher may be called across threads
    def _on_tactile_sample(self, hand: str, sample) -> None:
        if not C.sample_publishable(sample):
            return
        # all-or-nothing across both hands for product models with tactile: while the paired tactile
        # gate is closed (either hand's tactile unavailable), joints are not published and neither is tactile,
        # the same rule as the combined-frame path
        if not self._tactile_gate_open:
            return
        pub = self.pub_tactile_left if hand == "left" else self.pub_tactile_right
        pub.publish(C.make_tactile_sample(sample, self.frame_id))
        with self._count_lock:
            self.tactile_samples_published[hand] += 1
            self.tactile_published += 1

    def _on_tactile_tare(self, glove, state) -> None:
        hand = "left" if glove is self.pair.left else "right"
        hw = state.get("hardware") or {}
        self.get_logger().info(
            f"{hand} 触觉去皮状态：tared={state.get('tared')} origin={state.get('origin')} "
            f"outcome={state.get('outcome')} in_progress={state.get('in_progress')} "
            f"seq={hw.get('sequence')} state={hw.get('state')} message={state.get('message')}")
        self._tare_state.changed()

    def _publish_status(self) -> None:
        msg = String()
        counters = {"pairs_published": self.pairs_published,
                    "pairs_no_stamp": self.pairs_no_stamp,
                    "tactile_published": self.tactile_published,
                    "tactile_samples_published": dict(self.tactile_samples_published),
                    "tactile_transport": {"left": self.pair.left.tactile_transport,
                                          "right": self.pair.right.tactile_transport},
                    "pair_mode": self.pair_mode,
                    "pair_accuracy_ms": self.pair_accuracy_ms}
        body = json.loads(C.glove_status(self.pair.left, counters=counters,
                                         pairing=self.syn.stats(),
                                         extra=P.status_extra(self.product_identity)))
        # one tier record per hand: the right hand dropping a tier also makes pairs disappear
        body["right"] = json.loads(C.glove_status(self.pair.right))
        body["skew_alarms"] = self._skew_status()
        body["button"] = self._button_status()
        msg.data = json.dumps(body, ensure_ascii=False, default=str)
        self.pub_status.publish(msg)

    def destroy_node(self):
        self._camera_teardown()
        self._tracker_teardown()
        closers = [s.close for s in getattr(self, "_tactile_sample_subs", {}).values()]
        closers += [s.close for s in getattr(self, "_button_subs", {}).values()]
        if getattr(self, "pub_button", None) is not None:
            closers.append(lambda: self.destroy_publisher(self.pub_button))
        for closer in closers + [lambda: self.destroy_timer(self._status_timer),
                                 lambda: self.destroy_publisher(self.pub_status),
                                 lambda: self._sub.close(), lambda: self.syn.close(),
                                 lambda: self.pair.close()]:
            try:
                closer()
            except Exception:  # noqa: BLE001
                pass
        return super().destroy_node()


def main(argv=None) -> int:
    rclpy.init(args=argv)
    node = None
    code = 0
    try:
        node = BimanualNode()
        code = spin_until_exit(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    except Exception as exc:  # noqa: BLE001
        if rclpy.ok():
            print(f"bimanual_node 启动失败：{type(exc).__name__}: {exc}", file=sys.stderr)
            code = 1
    finally:
        shutdown_node(node)
    return code


if __name__ == "__main__":
    sys.exit(main())
