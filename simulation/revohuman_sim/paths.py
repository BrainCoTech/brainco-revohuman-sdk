"""Fixed locations inside the repository and the local cache directory."""

from __future__ import annotations

import os
import pathlib

PKG_DIR = pathlib.Path(__file__).resolve().parent
ASSETS_DIR = PKG_DIR / "assets"
SIM_DIR = PKG_DIR.parent
REPO_ROOT = SIM_DIR.parent
MODELS_ROOT = REPO_ROOT / "models"                  # hand geometry model registry: one model package per
                                                    # product (model.yaml + urdf/ + meshes/)
DEFAULT_PRODUCT = "PTV1"                            # which product's hand is drawn by default (the one with
                                                    # tactile; both have the same finger mechanism)


def model_dir(product: str = DEFAULT_PRODUCT, models_root: pathlib.Path = MODELS_ROOT) -> pathlib.Path:
    """Model package directory of this product in the model registry (contains urdf/ and meshes/). If the
    registry is missing, unreadable, or has no such product, returns models_root itself — it has no meshes/,
    so the caller reports it as "not found / incomplete" instead of raising at import time."""
    import yaml
    try:
        for yml in sorted(pathlib.Path(models_root).glob("*/model.yaml")):
            d = yaml.safe_load(yml.read_text(encoding="utf-8")) or {}
            if d.get("product") == product:
                return yml.parent
    except (OSError, yaml.YAMLError):
        pass
    return pathlib.Path(models_root)


DEFAULT_URDF_DIR = model_dir()                      # hand model urdf/ + meshes/
CACHE_ROOT = pathlib.Path(os.environ.get("XDG_CACHE_HOME") or "~/.cache").expanduser() / "revohuman"
