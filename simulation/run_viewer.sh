#!/usr/bin/env bash
# 手套可视化启动器：首次运行自动建隔离 venv 并装依赖（simulation/requirements.txt），之后直接起窗口。
# 可视化只订阅 SDK 的本机广播，不装 SDK 本体；SDK 在它自己的环境里起：
#
#   revohuman-live                              # 终端 1（SDK 的环境）：打开手套、开始广播
#                                               #   （同 python3 -m revohuman live）
#   simulation/run_viewer.sh                    # 终端 2：起窗口，等到广播就开始更新
#   simulation/run_viewer.sh --demo             # 不接手套先看效果（内置演示广播）
#   simulation/run_viewer.sh --help             # 全部参数
#
# 环境变量：
#   REVOHUMAN_SIM_VENV=<dir>        venv 位置（默认 simulation/.venv）
#   REVOHUMAN_SIM_PYTHON=<python>   用哪个解释器建 venv（默认 python3，要求 >= 3.10）
#   REVOHUMAN_SIM_PRIME=0           不设 NVIDIA PRIME 变量（默认设：双显卡笔记本上让窗口走独显）
#   REVOHUMAN_LIVE_SOCKET=<path>    SDK 广播的 socket（两边设同一个值；默认按固定规则找，一般不用设）
#   PIP_INDEX_URL=<url>             pip 镜像（pip 自己的变量，照常生效）
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

# 依赖随 requirements.txt 走：它变了（或还没装过）才重装
STAMP="${VENV}/.revohuman_viewer.stamp"
want="$(sha256sum "${REQ}" | cut -d' ' -f1)"
if [[ "$(cat "${STAMP}" 2>/dev/null)" != "${want}" ]]; then
    echo "[run_viewer] 安装依赖（${REQ}）到 ${VENV}"
    "${VPY}" -m pip install --disable-pip-version-check -q --upgrade pip >/dev/null 2>&1 || true
    "${VPY}" -m pip install --disable-pip-version-check -q -r "${REQ}" \
        || die "依赖安装失败（网络 / 镜像？可设 PIP_INDEX_URL 换源后重试）。"
    echo "${want}" > "${STAMP}"
fi

# 离屏（--headless）默认走 EGL：不要显示器、也不经过 GLX
for a in "$@"; do
    if [[ "${a}" == "--headless" ]]; then
        export MUJOCO_GL="${MUJOCO_GL:-egl}"
        break
    fi
done

# 双显卡（NVIDIA PRIME）笔记本：窗口跟独显走；没有 NVIDIA 驱动时这两个变量无副作用
if [[ "${REVOHUMAN_SIM_PRIME:-1}" == "1" && -z "${MUJOCO_GL:-}" ]] && command -v nvidia-smi >/dev/null 2>&1; then
    export __NV_PRIME_RENDER_OFFLOAD="${__NV_PRIME_RENDER_OFFLOAD:-1}"
    export __GLX_VENDOR_LIBRARY_NAME="${__GLX_VENDOR_LIBRARY_NAME:-nvidia}"
fi

exec "${VPY}" "${SIM_DIR}/viewer.py" "$@"
