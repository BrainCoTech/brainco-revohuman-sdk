#!/usr/bin/env bash
# Glove visualization launcher: on first run it creates an isolated venv and installs the dependencies
# (simulation/requirements.txt) automatically; after that it opens the window directly.
# The visualization only subscribes to the SDK's local broadcast and does not install the SDK itself;
# the SDK runs in its own environment:
#
#   revohuman-live                              # terminal 1 (SDK environment): open gloves, start broadcast
#                                               #   (same as python3 -m revohuman live)
#   simulation/run_viewer.sh                    # terminal 2: open the window, update once broadcast arrives
#   simulation/run_viewer.sh --demo             # try it without gloves first (built-in demo broadcast)
#   simulation/run_viewer.sh --help             # all arguments
#
# Environment variables:
#   REVOHUMAN_SIM_VENV=<dir>        venv location (default simulation/.venv)
#   REVOHUMAN_SIM_PYTHON=<python>   interpreter used to create the venv (default python3, requires >= 3.10)
#   REVOHUMAN_SIM_PRIME=0           do not set the NVIDIA PRIME variables (set by default: on dual-GPU
#                                   laptops the window uses the discrete GPU)
#   REVOHUMAN_LIVE_SOCKET=<path>    socket of the SDK broadcast (set the same value on both sides; by
#                                   default it is found by a fixed rule, usually no need to set it)
#   PIP_INDEX_URL=<url>             pip mirror (pip's own variable, takes effect as usual)
set -eo pipefail

SIM_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
VENV="${REVOHUMAN_SIM_VENV:-${SIM_DIR}/.venv}"
PY="${REVOHUMAN_SIM_PYTHON:-python3}"
REQ="${SIM_DIR}/requirements.txt"

die() { echo "[run_viewer] $*" >&2; exit 2; }

command -v "${PY}" >/dev/null 2>&1 || die "找不到 ${PY}；装 Python >= 3.10 后重试，或用 REVOHUMAN_SIM_PYTHON 指定解释器。"
"${PY}" -c 'import sys; sys.exit(0 if sys.version_info >= (3, 10) else 1)' \
    || die "$("${PY}" -V 2>&1) 太旧，需要 Python >= 3.10（可用 REVOHUMAN_SIM_PYTHON 指定别的解释器）。"

make_venv() {
    if "${PY}" -m venv "${VENV}" 2>/dev/null; then
        return 0
    fi
    rm -rf "${VENV}"
    if "${PY}" -m virtualenv --version >/dev/null 2>&1; then
        "${PY}" -m virtualenv "${VENV}" && return 0
    fi
    rm -rf "${VENV}"
    cat >&2 <<EOF
[run_viewer] 建不了 venv：${PY} 缺 venv/ensurepip 模块。任选其一：
  Debian/Ubuntu:  sudo apt install python3-venv
  或:             ${PY} -m pip install --user virtualenv
  或:             REVOHUMAN_SIM_PYTHON=<别的 python3.10+> $0 $*
EOF
    exit 2
}

VPY="${VENV}/bin/python"
if [[ ! -x "${VPY}" ]]; then
    echo "[run_viewer] 建隔离环境 ${VENV}"
    make_venv "$@"
fi

# Dependencies follow requirements.txt: reinstall only when it has changed (or nothing is installed yet)
STAMP="${VENV}/.revohuman_viewer.stamp"
want="$(sha256sum "${REQ}" | cut -d' ' -f1)"
if [[ "$(cat "${STAMP}" 2>/dev/null)" != "${want}" ]]; then
    echo "[run_viewer] 安装依赖（${REQ}）到 ${VENV}"
    "${VPY}" -m pip install --disable-pip-version-check -q --upgrade pip >/dev/null 2>&1 || true
    "${VPY}" -m pip install --disable-pip-version-check -q -r "${REQ}" \
        || die "依赖安装失败（网络 / 镜像？可设 PIP_INDEX_URL 换源后重试）。"
    echo "${want}" > "${STAMP}"
fi

# Offscreen (--headless) uses EGL by default: no display needed, and GLX is not involved
for a in "$@"; do
    if [[ "${a}" == "--headless" ]]; then
        export MUJOCO_GL="${MUJOCO_GL:-egl}"
        break
    fi
done

# Dual-GPU (NVIDIA PRIME) laptops: the window uses the discrete GPU; without the NVIDIA driver these two
# variables have no side effects
if [[ "${REVOHUMAN_SIM_PRIME:-1}" == "1" && -z "${MUJOCO_GL:-}" ]] && command -v nvidia-smi >/dev/null 2>&1; then
    export __NV_PRIME_RENDER_OFFLOAD="${__NV_PRIME_RENDER_OFFLOAD:-1}"
    export __GLX_VENDOR_LIBRARY_NAME="${__GLX_VENDOR_LIBRARY_NAME:-nvidia}"
fi

exec "${VPY}" "${SIM_DIR}/viewer.py" "$@"
