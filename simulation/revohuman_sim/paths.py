"""仓库内的固定位置与本机缓存目录。"""

from __future__ import annotations

import os
import pathlib

PKG_DIR = pathlib.Path(__file__).resolve().parent
ASSETS_DIR = PKG_DIR / "assets"
SIM_DIR = PKG_DIR.parent
REPO_ROOT = SIM_DIR.parent
DEFAULT_URDF_DIR = REPO_ROOT / "description"          # 手部模型 urdf/ + meshes/
CACHE_ROOT = pathlib.Path(os.environ.get("XDG_CACHE_HOME") or "~/.cache").expanduser() / "revohuman"
