#!/usr/bin/env python3
"""手套可视化入口（免安装）：把 simulation/ 放进 sys.path，再交给 revohuman_sim.app。

    python3 simulation/viewer.py --demo
    python3 simulation/viewer.py --help

依赖（simulation/requirements.txt：mujoco / glfw / numpy / PyYAML）请用 simulation/run_viewer.sh
装进隔离 venv，或在你自己的环境里 `pip install -r simulation/requirements.txt`。不需要装 SDK：
数据来自 SDK 的本机广播（`revohuman-live`，在 SDK 的环境里起）。
"""

import pathlib
import sys

SIM_DIR = pathlib.Path(__file__).resolve().parent
if str(SIM_DIR) not in sys.path:
    sys.path.insert(0, str(SIM_DIR))

try:
    from revohuman_sim.app import main  # noqa: E402
except ModuleNotFoundError as exc:          # 在自己的环境里直接跑、还没装 requirements.txt
    if exc.name not in {"yaml", "numpy", "mujoco", "glfw"}:
        raise
    print(f"[viewer] 缺依赖 {exc.name}：请用 simulation/run_viewer.sh 启动（自动建隔离 venv 并装依赖），"
          f"或在你自己的环境里 pip install -r simulation/requirements.txt", file=sys.stderr)
    sys.exit(2)

if __name__ == "__main__":
    sys.exit(main())
