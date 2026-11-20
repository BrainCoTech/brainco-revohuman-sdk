#!/usr/bin/env python3
"""Glove visualization entry point (no install needed): puts simulation/ on sys.path, then hands over to
revohuman_sim.app.

    python3 simulation/viewer.py --demo
    python3 simulation/viewer.py --help

Install the dependencies (simulation/requirements.txt: mujoco / glfw / numpy / PyYAML) into an isolated venv
with simulation/run_viewer.sh, or run `pip install -r simulation/requirements.txt` in your own environment.
The SDK does not need to be installed: the data comes from the SDK's local broadcast (`revohuman-live`,
started in the SDK's environment).
"""

import pathlib
import sys

SIM_DIR = pathlib.Path(__file__).resolve().parent
if str(SIM_DIR) not in sys.path:
    sys.path.insert(0, str(SIM_DIR))

try:
    from revohuman_sim.app import main  # noqa: E402
except ModuleNotFoundError as exc:          # run directly in your own environment, requirements.txt not
                                            # installed yet
    if exc.name not in {"yaml", "numpy", "mujoco", "glfw"}:
        raise
    print(f"[viewer] 缺依赖 {exc.name}：请用 simulation/run_viewer.sh 启动（自动建隔离 venv 并装依赖），"
          f"或在你自己的环境里 pip install -r simulation/requirements.txt", file=sys.stderr)
    sys.exit(2)

if __name__ == "__main__":
    sys.exit(main())
