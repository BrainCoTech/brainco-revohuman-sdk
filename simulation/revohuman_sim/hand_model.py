"""Dual-hand URDF assembly + forward kinematics.

The model is the hand model in models/ at the root of the source bundle (one dual-hand .urdf under urdf/ +
meshes/), or the directory given by --urdf-dir; if the model directory is missing or incomplete, the
visualization explains this at startup and exits (`MissingModelError`).
Both hands hang in the same MuJoCo model, each connected to world through a free joint (URDF
`type="floating"`): the caller writes the palm_link pose into qpos every frame, so the same model can be
paired with two MjData —

  fixed-base view: both palms placed at home_pose (left (0, +s/2, 0), right (0, −s/2, 0), R=I), fingers only;
  Cartesian view:  both palms placed at the 6D poses given by the Trackers.

The finger joint angles in the two MjData are bit-for-bit identical; only the palm_link poses differ.

Joint angles: the SDK broadcast has already converted the firmware-corrected angles into URDF joint angles
(rad), and the joint table (names, order, encoder indices) is also sent with hello — differences between
product models (P1 / PTV1) are maintained in one place only, in the SDK; this code does not know any
product model.
"""

from __future__ import annotations

import os
import pathlib
import xml.etree.ElementTree as ET

from .paths import CACHE_ROOT, DEFAULT_URDF_DIR

HANDS = ("left", "right")
BOTH_SPACING_M = 0.30          # palm spacing of the two hands (= origin y of left_right_joint in the
                               # dual-hand URDF)
URDF_CACHE = CACHE_ROOT / "hand_urdf"
HAND_RGBA = "1 1 1 1"           # hand mesh color (the URDF in the model package is not modified; overridden
                                # at assembly time)


class MissingModelError(FileNotFoundError):
    """Hand model (URDF + meshes) missing or incomplete. The message is the full explanation for the user; the
    entry point prints it and exits."""


def hands_of(hand: str) -> tuple[str, ...]:
    """Value of --hands → hands to draw; both = ("left", "right")."""
    return HANDS if hand == "both" else (hand,)


def urdf_file(pkg_dir: pathlib.Path) -> pathlib.Path:
    """The dual-hand URDF in the model directory: urdf/ must contain exactly one .urdf (the file name is not
    checked, so switching models needs no code change)."""
    found = sorted((pathlib.Path(pkg_dir) / "urdf").glob("*.urdf"))
    if len(found) != 1:
        what = "没有 .urdf 文件" if not found else f"有 {len(found)} 份 .urdf，只能留一份"
        raise MissingModelError(f"手部模型目录不完整：{pkg_dir}（urdf/ 下{what}）")
    return found[0]


def resolve_urdf_dir(urdf_dir: pathlib.Path | None = None) -> pathlib.Path:
    """Find the hand model directory (urdf/ + meshes/); if it is missing or incomplete, raise
    MissingModelError with a message that says what to do."""
    if urdf_dir is None and not (DEFAULT_URDF_DIR / "meshes").is_dir():
        raise MissingModelError(
            f"找不到手部模型目录：{DEFAULT_URDF_DIR}\n"
            "  源码包根目录的 models/ 里应有手部模型（urdf/ 与 meshes/）；放在别处时用 --urdf-dir 指定所在目录。")
    urdf_dir = pathlib.Path(urdf_dir or DEFAULT_URDF_DIR)
    if not (urdf_dir / "meshes").is_dir():
        raise MissingModelError(f"手部模型目录不完整：{urdf_dir}（要有 urdf/ 与 meshes/）")
    urdf_file(urdf_dir)
    return urdf_dir


def root_joint_name(hand: str, hands) -> str:
    """Name of the joint connecting palm_link to world: world_to_palm for one hand, world_to_{hand}_palm for
    two."""
    return "world_to_palm" if len(tuple(hands)) == 1 else f"world_to_{hand}_palm"


def home_position(hand: str, hands, spacing_m: float = BOTH_SPACING_M) -> tuple[float, float, float]:
    """Position of the palm_link frame origin in the fixed-base view: one hand at the origin; two hands at
    left +s/2, right −s/2 (y axis)."""
    if len(tuple(hands)) == 1:
        return (0.0, 0.0, 0.0)
    return (0.0, spacing_m / 2.0 if hand == "left" else -spacing_m / 2.0, 0.0)


def build_hand_urdf(pkg_dir: pathlib.Path, hands, out_path: pathlib.Path,
                    spacing_m: float = BOTH_SPACING_M, floating: bool = True) -> pathlib.Path:
    """From the dual-hand URDF keep the hands listed in hands, each attached to an explicit world link
    (otherwise MuJoCo merges the root link into world and the palm no longer has its own body), and insert a
    <mujoco><compiler> extension so the STL files can be found.
    The fixed left_palm→right_palm joint in the URDF is dropped: each palm hangs from world, and the right
    palm cannot have two parents.
    With floating=True, world→palm_link is a free joint (qpos 7-D: position + quaternion wxyz); otherwise it
    is a fixed joint placed at home."""
    hands = tuple(hands)
    tree = ET.parse(urdf_file(pkg_dir))
    root = tree.getroot()
    prefixes = tuple(h + "_" for h in hands)
    for link in list(root.findall("link")):
        if not link.get("name").startswith(prefixes):
            root.remove(link)
    for joint in list(root.findall("joint")):
        p = joint.find("parent").get("link")
        c = joint.find("child").get("link")
        pp = next((x for x in prefixes if p.startswith(x)), None)
        cp = next((x for x in prefixes if c.startswith(x)), None)
        if pp is None or cp is None or pp != cp:      # the cross-hand left_right_joint is also dropped here
            root.remove(joint)
    for mesh in root.iter("mesh"):
        mesh.set("filename", mesh.get("filename").split("/")[-1])
    # Model package URDF colors come from the CAD export (recent model versions are all black, invisible on a
    # black background): paint everything a light color.
    for color in root.iter("color"):
        color.set("rgba", HAND_RGBA)
    root.insert(0, ET.Element("link", name="world"))
    for k, hand in enumerate(hands):
        wj = ET.Element("joint", name=root_joint_name(hand, hands), type="floating" if floating else "fixed")
        ET.SubElement(wj, "parent", link="world")
        ET.SubElement(wj, "child", link=f"{hand}_palm_link")
        if not floating:
            x, y, z = home_position(hand, hands, spacing_m)
            ET.SubElement(wj, "origin", xyz=f"{x:.6g} {y:.6g} {z:.6g}", rpy="0 0 0")
        root.insert(1 + k, wj)
    mj = ET.Element("mujoco")
    ET.SubElement(mj, "compiler", meshdir=str((pkg_dir / "meshes").resolve()),
                  discardvisual="false", balanceinertia="true", strippath="true", fusestatic="false")
    root.insert(0, mj)
    out_path.parent.mkdir(parents=True, exist_ok=True)
    tmp = out_path.with_name(f".{out_path.name}.{os.getpid()}.tmp")
    tree.write(tmp)
    os.replace(tmp, out_path)                        # atomic replace: even if two processes assemble the same
                                                     # file at once, no half-written file is read
    return out_path


def load_model(hands, urdf_dir: pathlib.Path | None = None, spacing_m: float = BOTH_SPACING_M,
               cache_dir: pathlib.Path = URDF_CACHE, floating: bool = True):
    """Assemble the URDF and load it as (MjModel, MjData, urdf path). With floating, the palm_links in MjData
    are already placed at home."""
    import mujoco
    pkg = resolve_urdf_dir(urdf_dir)
    hands = tuple(hands)
    base = f"hand_{hands[0]}" if len(hands) == 1 else "hands_both"
    name = f"{base}_{'free' if floating else f'fixed_{spacing_m:g}'}.urdf"
    urdf = build_hand_urdf(pkg, hands, cache_dir / name, spacing_m=spacing_m, floating=floating)
    m = mujoco.MjModel.from_xml_path(str(urdf))
    d = mujoco.MjData(m)
    if floating:
        for hand in hands:
            set_root(m, d, root_joint_name(hand, hands), home_position(hand, hands, spacing_m))
    mujoco.mj_forward(m, d)
    return m, d, urdf


def root_qposadr(m, joint_name: str) -> int:
    """qpos start address of a free joint; raises ValueError if the model has no such joint or it is not a
    free joint."""
    import mujoco
    j = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_JOINT, joint_name)
    if j < 0 or int(m.jnt_type[j]) != int(mujoco.mjtJoint.mjJNT_FREE):
        raise ValueError(f"模型里没有自由关节 {joint_name}")
    return int(m.jnt_qposadr[j])


def set_root(m, d, joint_name: str, pos, quat_wxyz=(1.0, 0.0, 0.0, 0.0)) -> None:
    """Place palm_link (free joint) at the world-frame pose (pos, quat wxyz). Does not forward."""
    adr = root_qposadr(m, joint_name)
    d.qpos[adr:adr + 3] = [float(v) for v in pos]
    d.qpos[adr + 3:adr + 7] = [float(v) for v in quat_wxyz]


class HandModel:
    """The joints of one hand in a MuJoCo model (possibly shared with the other hand); the joint table comes
    from the SDK's hello.

    ``joints`` is this hand's ``[{index, name, finger, joint}]`` from hello: ``name`` is the full URDF joint
    name, ``index`` is its index in the encoder array / valid-bit mask, and the list order is the order of
    ``q`` in glove messages.
    update() writes only this hand's slice of qpos and does not forward; after both hands are written, the
    caller calls forward() once.
    Joints whose bit in the valid-bit mask is invalid (or whose q is null) keep the previous frame's valid
    value; joints that have never received a valid value stay at the zero pose.
    self.d is the fixed-base MjData (the tactile heatmap computes FK from it); the other one (Cartesian view)
    syncs the finger joints via write(d)."""

    def __init__(self, m, d, hand: str, joints):
        import mujoco
        self.mujoco = mujoco
        self.m, self.d, self.hand = m, d, hand
        self.specs = [dict(j) for j in joints]
        if not self.specs:
            raise ValueError(f"{hand} 手的关节表是空的")
        self.qadr = []
        missing = []
        for spec in self.specs:
            j = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_JOINT, spec["name"])
            if j < 0:
                missing.append(spec["name"])
            self.qadr.append(int(m.jnt_qposadr[j]) if j >= 0 else -1)
        if missing:
            raise ValueError(f"SDK 下发的关节在 URDF 里不存在：{missing}")
        n = len(self.specs)
        self.held_deg = [None] * n
        self.seen = [False] * n
        self.q = [0.0] * n

    @property
    def tag(self) -> str:
        return self.hand[0].upper()

    def update(self, q, deg, mask: int) -> None:
        """One frame: q in joint-table order (rad, may contain None), deg by encoder index (degrees, display
        only)."""
        if q is not None:
            for k, spec in enumerate(self.specs):
                idx = int(spec["index"])
                if not (mask >> idx & 1) or k >= len(q) or q[k] is None:
                    continue
                self.q[k] = float(q[k])
                if deg is not None and idx < len(deg):
                    self.held_deg[k] = deg[idx]
                self.seen[k] = True
        self.write(self.d)

    def write(self, d) -> None:
        """Write the current joint angles into any MjData (of the same model). Does not forward."""
        for adr, q in zip(self.qadr, self.q):
            d.qpos[adr] = q

    def forward(self) -> None:
        self.mujoco.mj_forward(self.m, self.d)

    def invalid(self) -> list[str]:
        """Joints that have never received a valid value (short names)."""
        return [self.short(s) for s, ok in zip(self.specs, self.seen) if not ok]

    @staticmethod
    def short(spec: dict) -> str:
        return f"{str(spec.get('finger', ''))[:3]} {spec.get('joint', '')}"

    def joint_rows(self) -> list[tuple[str, float | None, float, bool]]:
        """Overlay joint table: (short name, firmware reading °, URDF angle °, ever valid)."""
        import math
        return [(self.short(s), self.held_deg[k], math.degrees(self.q[k]), self.seen[k])
                for k, s in enumerate(self.specs)]
