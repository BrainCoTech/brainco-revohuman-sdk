"""双手 URDF 组装 + 正运动学。

模型取源码包根目录的 description/（urdf/ 下一份双手 .urdf + meshes/），或 --urdf-dir 指定的目录；
模型目录不在或不完整时，可视化在启动时就给出说明并退出（`MissingModelError`）。
两只手挂在同一个 MuJoCo 模型里，各自以一个自由关节（URDF `type="floating"`）接到 world：
palm_link 位姿由调用方逐帧写进 qpos，于是同一个模型可以配两份 MjData——

  固定基座视图：两掌摆在 home_pose（左 (0, +s/2, 0)、右 (0, −s/2, 0)，R=I），只看手指；
  笛卡尔视图：  两掌摆在 tracker 给出的 6D 位姿上。

两份 MjData 的手指关节角逐位相同，只有 palm_link 位姿不同。

关节角：SDK 广播已经把固件修正角换成了 URDF 关节角（rad），关节表（名字、顺序、编码器下标）也随
hello 下发——型号（P1 / PTV1）的差别只在 SDK 里维护一处，这里不认识任何型号。
"""

from __future__ import annotations

import os
import pathlib
import xml.etree.ElementTree as ET

from .paths import CACHE_ROOT, DEFAULT_URDF_DIR

HANDS = ("left", "right")
BOTH_SPACING_M = 0.30          # 双手掌心间距（= 双手 URDF 里 left_right_joint 的 origin y）
URDF_CACHE = CACHE_ROOT / "hand_urdf"


class MissingModelError(FileNotFoundError):
    """手部模型（URDF + 网格）不在或不完整。消息是给用户看的完整说明，入口直接打印后退出。"""


def hands_of(hand: str) -> tuple[str, ...]:
    """--hands 的取值 → 要画的手；both = ("left", "right")。"""
    return HANDS if hand == "both" else (hand,)


def urdf_file(pkg_dir: pathlib.Path) -> pathlib.Path:
    """模型目录里那份双手 URDF：urdf/ 下必须恰好一份 .urdf（不认文件名，换模型不用改代码）。"""
    found = sorted((pathlib.Path(pkg_dir) / "urdf").glob("*.urdf"))
    if len(found) != 1:
        what = "没有 .urdf 文件" if not found else f"有 {len(found)} 份 .urdf，只能留一份"
        raise MissingModelError(f"手部模型目录不完整：{pkg_dir}（urdf/ 下{what}）")
    return found[0]


def resolve_urdf_dir(urdf_dir: pathlib.Path | None = None) -> pathlib.Path:
    """找手部模型目录（urdf/ + meshes/）；不在或不完整时抛 MissingModelError，消息里写明怎么办。"""
    if urdf_dir is None and not DEFAULT_URDF_DIR.exists():
        raise MissingModelError(
            f"找不到手部模型目录：{DEFAULT_URDF_DIR}\n"
            "  源码包根目录的 description/ 里应有 urdf/ 与 meshes/；放在别处时用 --urdf-dir 指定所在目录。")
    urdf_dir = pathlib.Path(urdf_dir or DEFAULT_URDF_DIR)
    if not (urdf_dir / "meshes").is_dir():
        raise MissingModelError(f"手部模型目录不完整：{urdf_dir}（要有 urdf/ 与 meshes/）")
    urdf_file(urdf_dir)
    return urdf_dir


def root_joint_name(hand: str, hands) -> str:
    """palm_link 接 world 的那根关节名：单手 world_to_palm，双手 world_to_{hand}_palm。"""
    return "world_to_palm" if len(tuple(hands)) == 1 else f"world_to_{hand}_palm"


def home_position(hand: str, hands, spacing_m: float = BOTH_SPACING_M) -> tuple[float, float, float]:
    """固定基座视图里的 palm_link 坐标原点位置：单手在原点，双手左 +s/2、右 −s/2（y 轴）。"""
    if len(tuple(hands)) == 1:
        return (0.0, 0.0, 0.0)
    return (0.0, spacing_m / 2.0 if hand == "left" else -spacing_m / 2.0, 0.0)


def build_hand_urdf(pkg_dir: pathlib.Path, hands, out_path: pathlib.Path,
                    spacing_m: float = BOTH_SPACING_M, floating: bool = True) -> pathlib.Path:
    """从双手 URDF 里留下 hands 里那几只手，各自挂到显式 world link 上（否则 MuJoCo 会把根 link 吞进
    world，手掌就没有自己的 body 了），并塞入 <mujoco><compiler> 扩展让 STL 能被找到。
    URDF 里那根 left_palm→right_palm 的固定关节被丢掉：两掌各自挂 world，右掌不能有两个父节点。
    floating=True 时 world→palm_link 是自由关节（qpos 7 维：位置 + 四元数 wxyz），否则是摆在 home 位的固定关节。"""
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
        if pp is None or cp is None or pp != cp:      # 跨手的 left_right_joint 也在这里丢掉
            root.remove(joint)
    for mesh in root.iter("mesh"):
        mesh.set("filename", mesh.get("filename").split("/")[-1])
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
    os.replace(tmp, out_path)                        # 原子替换：两个进程同时组装同一份也读不到半截文件
    return out_path


def load_model(hands, urdf_dir: pathlib.Path | None = None, spacing_m: float = BOTH_SPACING_M,
               cache_dir: pathlib.Path = URDF_CACHE, floating: bool = True):
    """组装 URDF 并加载成 (MjModel, MjData, urdf 路径)。floating 时 MjData 的 palm_link 已摆到 home 位。"""
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
    """自由关节 qpos 起始地址；模型里没有这根关节或不是自由关节就抛 ValueError。"""
    import mujoco
    j = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_JOINT, joint_name)
    if j < 0 or int(m.jnt_type[j]) != int(mujoco.mjtJoint.mjJNT_FREE):
        raise ValueError(f"模型里没有自由关节 {joint_name}")
    return int(m.jnt_qposadr[j])


def set_root(m, d, joint_name: str, pos, quat_wxyz=(1.0, 0.0, 0.0, 0.0)) -> None:
    """把 palm_link（自由关节）摆到世界系位姿 (pos, quat wxyz)。不 forward。"""
    adr = root_qposadr(m, joint_name)
    d.qpos[adr:adr + 3] = [float(v) for v in pos]
    d.qpos[adr + 3:adr + 7] = [float(v) for v in quat_wxyz]


class HandModel:
    """一只手在（可能与另一只手共用的）MuJoCo 模型里的那些关节，关节表来自 SDK 的 hello。

    ``joints`` 是 hello 里这只手的 ``[{index, name, finger, joint}]``：``name`` 是 URDF 关节全名，
    ``index`` 是它在编码器数组 / 有效位掩码里的下标，列表顺序就是 glove 消息里 ``q`` 的顺序。
    update() 只写本手那段 qpos，不 forward；两只手都写完后由调用方 forward() 一次。
    有效位掩码里无效的位（或 q 为 null）保持上一帧的有效值，还没收到过有效值的保持零姿态。
    self.d 是固定基座那份 MjData（触觉热力图按它算 FK）；另一份（笛卡尔视图）用 write(d) 同步手指关节。"""

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
        """一帧：q 按关节表顺序（rad，可含 None），deg 按编码器下标（度，仅显示用）。"""
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
        """把当前关节角写进任意一份（同模型的）MjData。不 forward。"""
        for adr, q in zip(self.qadr, self.q):
            d.qpos[adr] = q

    def forward(self) -> None:
        self.mujoco.mj_forward(self.m, self.d)

    def invalid(self) -> list[str]:
        """还没收到过有效值的关节（短名）。"""
        return [self.short(s) for s, ok in zip(self.specs, self.seen) if not ok]

    @staticmethod
    def short(spec: dict) -> str:
        return f"{str(spec.get('finger', ''))[:3]} {spec.get('joint', '')}"

    def joint_rows(self) -> list[tuple[str, float | None, float, bool]]:
        """覆盖层关节表：(短名, 固件读数°, URDF 角°, 有效过)。"""
        import math
        return [(self.short(s), self.held_deg[k], math.degrees(self.q[k]), self.seen[k])
                for k, s in enumerate(self.specs)]
