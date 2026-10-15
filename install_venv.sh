#!/usr/bin/env bash
# 一键把 RevoHuman SDK 装进独立的 Python 虚拟环境（不含 ROS 2）：SDK 连同录制、相机图像解码、基站蓝牙、
# 三维可视化四个 extra，随附的 Tracker 驱动 pysurvive，三维可视化的依赖；再按需装系统软件包、
# Tracker 设备规则，把当前用户加入 dialout 组。
#
#   ./install_venv.sh                  装到 ~/.venvs/revohuman_sdk
#   ./install_venv.sh --mirror cn      国内网络：PyPI 改用阿里云镜像
#
# 需要 sudo 的步骤执行前逐项询问。已经装好的部分重新执行时跳过，可以放心重复执行。
# 完整说明见使用手册《01-环境安装与系统要求》第 11 节。

# 用 sh 执行时转给 bash（下面用到 bash 的语法）；source 进当前 shell 会在出错时连 shell 一起退出，不允许
if [ -n "${ZSH_VERSION:-}" ] || { [ -z "${BASH_VERSION:-}" ] && [ ! -f "$0" ]; }; then
    echo "install_venv.sh: 请直接执行 ./install_venv.sh（或 bash install_venv.sh）" >&2
    return 1 2>/dev/null || exit 1
fi
if [ -z "${BASH_VERSION:-}" ]; then exec bash "$0" "$@"; fi
if [[ "${BASH_SOURCE[0]}" != "$0" ]]; then
    echo "install_venv.sh: 请直接执行 ./install_venv.sh，不要用 source" >&2
    return 1 2>/dev/null || exit 1
fi

set -eo pipefail

VERSION="0.1.1"
SCRIPT_DIR="$(dirname "$(readlink -f "${BASH_SOURCE[0]}")")"
VENV="${REVOHUMAN_VENV:-${HOME}/.venvs/revohuman_sdk}"
WHEELS=""
PY_ARG=""
MIRROR=""
NO_TRACKER=0
NO_SYSTEM=0
FORCE=0
YES=0
DRY_RUN=0
EXTRAS="recording,camera,ble,viewer"
# 三维可视化用的 mujoco 限定在发布前测试过的版本范围内（第 4、5 步都按它装，已装的版本不在范围内会被替换），
# 与 simulation/requirements.txt、pyproject.toml（viewer）一致
MUJOCO_MIN="3.1"
MUJOCO_BELOW="3.13"
MUJOCO_REQ="mujoco>=${MUJOCO_MIN},<${MUJOCO_BELOW}"
PROMPT_NAME="revohuman_sdk"
CN_INDEX="https://mirrors.aliyun.com/pypi/simple/"
MIN_FREE_MB=1536
UDEV_RULE_NAME="81-revohuman-tracker.rules"
LOG=""
NEED_RELOGIN=0
UDEV_INSTALLED=0
# 只给测试用：把读写的主机系统文件（/etc、/lib、/usr/lib、/run、/proc、/sys、/usr/share 下的）换到另一个根目录下
SYSROOT="${REVOHUMAN_INSTALL_SYSROOT:-}"

usage() {
    cat <<EOF
用法：install_venv.sh [选项]

把 RevoHuman SDK ${VERSION} 与除 ROS 2 以外的全部依赖装进一个独立的 Python 虚拟环境
（激活后提示符前显示 (${PROMPT_NAME})）。在解开的源码包里执行；发布文件（两个 wheel、SHA256SUMS）
默认在源码包的上一级目录，也就是解开源码包的那个目录。

依次执行：
  1. 系统依赖：缺少时用 apt 安装（OpenGL / EGL 运行库、GLib、窗口用的 X11 库、bluez、python3.10-venv），
     安装 Tracker 设备规则，把当前用户加入 dialout 组。需要 sudo 的步骤执行前逐项询问
  2. 建虚拟环境
  3. 按 SHA256SUMS 校验并安装随附的 pysurvive
  4. 安装 SDK，含 ${EXTRAS} 四个 extra
  5. 安装三维可视化的依赖（simulation/requirements.txt；mujoco 装 ${MUJOCO_MIN} 及以上、${MUJOCO_BELOW} 以下的版本）
  6. 自检：导入各模块，确认命令行工具能启动、可视化能出图

选项：
  --venv 目录         虚拟环境的位置（默认 ~/.venvs/revohuman_sdk；也可用环境变量 REVOHUMAN_VENV 指定）
  --wheels 目录       发布文件（wheel 与 SHA256SUMS）所在的目录（默认：源码包的上一级目录）
  --python 解释器     建虚拟环境用的 Python，必须是 CPython 3.10（默认依次找 python3.10、python3）
  --mirror cn|official  PyPI 源。cn：阿里云镜像（国内网络用）；默认沿用 pip 自己的设置（PIP_INDEX_URL 等）
  --no-tracker        不装 Tracker 驱动 pysurvive 与 Tracker 设备规则
  --no-system         不改动系统：不装 apt 软件包与设备规则，不加 dialout 组，只检查并提示
  --force             删掉已有的虚拟环境，重新建
  -y, --yes           需要 sudo 的步骤不逐项询问，直接执行
  --dry-run           只做检查并打印要执行的命令，不改动系统、不建虚拟环境
  -h, --help          显示本帮助

装好之后：
  source ~/.venvs/revohuman_sdk/bin/activate     每个新终端先激活，之后 revohuman-live 等命令可直接用
  simulation/run_sim_host.sh                     一条命令拉起广播与三维可视化（自动使用这个虚拟环境）
EOF
}

die() { printf '\ninstall_venv.sh: %s\n' "$*" >&2; exit 1; }

while [[ $# -gt 0 ]]; do
    case "$1" in
        --*=*) set -- "${1%%=*}" "${1#*=}" "${@:2}"; continue ;;
        --venv) VENV="${2:?--venv 需要参数}"; shift 2 ;;
        --wheels) WHEELS="${2:?--wheels 需要参数}"; shift 2 ;;
        --python) PY_ARG="${2:?--python 需要参数}"; shift 2 ;;
        --mirror) MIRROR="${2:?--mirror 需要参数（cn 或 official）}"; shift 2 ;;
        --no-tracker) NO_TRACKER=1; shift ;;
        --no-system) NO_SYSTEM=1; shift ;;
        --force) FORCE=1; shift ;;
        -y|--yes) YES=1; shift ;;
        --dry-run) DRY_RUN=1; shift ;;
        -h|--help) usage; exit 0 ;;
        *) die "不认识的参数 $1（用 --help 查看用法）" ;;
    esac
done
case "${MIRROR}" in ""|official|cn) ;; *) die "--mirror 只能是 cn 或 official，收到的是「${MIRROR}」" ;; esac
[[ "${VENV}" == /* ]] || VENV="${PWD}/${VENV}"
VENV="${VENV%/}"
[[ -n "${VENV}" ]] || die "--venv 不能是根目录"

# ---------- 输出与询问 ----------
WARNINGS=()
say()  { printf '\n[%s] %s\n' "$1" "$2"; }
info() { printf '    %s\n' "$*"; }
ok()   { printf '    ✓ %s\n' "$*"; }
warn() { printf '    ! %s\n' "$*" >&2; WARNINGS+=("$*"); }
print_warnings() {
    [[ ${#WARNINGS[@]} -gt 0 ]] || return 0
    echo; echo "注意："; printf '    ! %s\n' "${WARNINGS[@]}"
}

# 按 POSIX sh 的规则给参数加引号：打印给用户照抄的命令用它
shq() {
    local a out=""
    for a in "$@"; do
        if [[ "${a}" =~ ^[A-Za-z0-9_./:=@%+,-]+$ ]]; then out+="${a} "
        else out+="'${a//\'/\'\\\'\'}' "; fi
    done
    printf '%s' "${out% }"
}

# confirm 问题：同意返回 0。--yes 直接同意；非交互运行又没给 --yes 时不同意（由调用处打印手动执行的命令）
confirm() {
    if [[ ${DRY_RUN} == 1 ]]; then info "$1（--dry-run：只打印命令）"; return 0; fi
    if [[ ${YES} == 1 ]]; then info "$1（--yes：直接执行）"; return 0; fi
    if [[ ! -t 0 ]]; then info "$1（非交互运行，也没有给 --yes：跳过）"; return 1; fi
    local reply
    read -r -p "    $1 [Y/n] " reply || return 1
    [[ -z "${reply}" || "${reply}" == [Yy]* || "${reply}" == 是* ]]
}

# 用 sudo 执行一条命令并先把它打印出来；as_root 失败即退出，try_root 只返回结果。
# 打印到 stderr：调用处可能丢掉命令本身的输出（例如 as_root tee 文件 >/dev/null）
try_root() {
    info "\$ sudo $(shq "$@")" >&2
    [[ ${DRY_RUN} == 0 ]] || return 0
    command -v sudo >/dev/null || { warn "这一步需要 sudo，本机没有 sudo 命令"; return 1; }
    sudo "$@"
}
as_root() { try_root "$@" || die "命令没有成功：sudo $(shq "$@")（没有 sudo 权限时请让管理员执行）"; }

# 以普通用户执行一条命令并先把它打印出来；--dry-run 只打印
run() {
    info "\$ $(shq "$@")"
    [[ ${DRY_RUN} == 0 ]] || return 0
    "$@"
}

norm_arch() { case "$1" in x86_64|amd64) echo x86_64 ;; aarch64|arm64) echo aarch64 ;; *) echo "$1" ;; esac; }
osr() { awk -F= -v k="$1" '$1 == k { gsub(/"/, "", $2); print $2 }' "${SYSROOT}/etc/os-release" 2>/dev/null || true; }
has_apt() { command -v apt-get >/dev/null && command -v dpkg-query >/dev/null; }
dpkg_installed() { [[ "$(dpkg-query -W -f='${Status}' "$1" 2>/dev/null || true)" == *"install ok installed"* ]]; }

TEE_PID=""
TMP=""
cleanup() {
    [[ -z "${TMP}" || ! -d "${TMP}" ]] || rm -rf -- "${TMP}"
    if [[ -n "${TEE_PID}" ]]; then exec 1>&- 2>&-; wait "${TEE_PID}" 2>/dev/null || true; fi
}
trap cleanup EXIT
trap 'exit 130' INT
trap 'exit 143' TERM

# ---------- 准备：运行条件、发布文件、Python ----------
preflight() {
    [[ "$(id -u)" != 0 ]] || die "不要用 root 或 sudo 运行本脚本：虚拟环境要建在普通用户的家目录里、归普通用户所有。
    以普通用户执行 ./install_venv.sh，需要 sudo 的步骤脚本会单独询问"
    [[ "$(uname -s)" == Linux ]] || die "只支持 Linux 主机"
    ARCH="$(norm_arch "$(uname -m)")"
    [[ "${ARCH}" == x86_64 || "${ARCH}" == aarch64 ]] || die "不支持的 CPU 架构 $(uname -m)：只提供 x86_64 与 aarch64 的 wheel"

    # 同一用户同时只跑一个 install_venv.sh（并发安装会往同一个虚拟环境里写）；--dry-run 不改动什么，不用排队
    local lock="${XDG_RUNTIME_DIR:-/tmp}/revohuman-install_venv-$(id -u).lock"
    if [[ ${DRY_RUN} == 0 ]] && command -v flock >/dev/null && { : >>"${lock}"; } 2>/dev/null; then
        exec 9>>"${lock}"
        flock -n 9 || die "另一个 install_venv.sh 正在运行（锁文件 ${lock}），等它结束后再执行"
    fi

    # 全部输出另存一份日志，出问题时把它发给技术支持
    local logdir="${XDG_STATE_HOME:-${HOME}/.local/state}/revohuman-venv"
    if mkdir -p "${logdir}" 2>/dev/null && [[ -w "${logdir}" ]]; then
        LOG="${logdir}/install-$(date +%Y%m%d-%H%M%S).log"
        exec > >(tee -ai --output-error=warn "${LOG}") 2>&1
        TEE_PID=$!
    fi
    echo "RevoHuman ${VERSION} 一键安装到虚拟环境（$(date '+%F %T')）"
    [[ -z "${LOG}" ]] || echo "日志：${LOG}"
    [[ ${DRY_RUN} == 0 ]] || echo "--dry-run：只检查、打印命令，不改动系统、不建虚拟环境"
    if grep -qsi microsoft "${SYSROOT}/proc/sys/kernel/osrelease"; then
        warn "检测到 WSL：手套与 Tracker 要先用 usbipd 接进 WSL 才看得到"
    fi
    say 准备 "运行条件与发布文件"
    info "虚拟环境：${VENV}"
    TMP="$(mktemp -d "${TMPDIR:-/tmp}/revohuman-venv.XXXXXX")" || die "建不了临时目录（${TMPDIR:-/tmp} 不可写？）"
    # 解开的源码包里没有 pyproject.toml；源码仓库里有，SDK 从源码安装
    if [[ -f "${SCRIPT_DIR}/pyproject.toml" && -f "${SCRIPT_DIR}/revohuman/__init__.py" ]]; then
        SRC_MODE=1
        VERSION="$(sed -n 's/^version *= *"\(.*\)"/\1/p' "${SCRIPT_DIR}/pyproject.toml" | head -n 1)"
        [[ -n "${VERSION}" ]] || die "读不出 ${SCRIPT_DIR}/pyproject.toml 里的版本号"
        info "在源码目录里运行：SDK ${VERSION} 从 ${SCRIPT_DIR} 安装"
    else
        SRC_MODE=0
        [[ -f "${SCRIPT_DIR}/simulation/requirements.txt" ]] \
            || die "${SCRIPT_DIR} 里没有 simulation/requirements.txt：install_venv.sh 要在解开的源码包（revohuman-${VERSION}/）里运行"
    fi
    check_space
}

check_space() {   # 虚拟环境装完约 1 GB（OpenCV、MuJoCo、NumPy 占大头）
    local d="${VENV}" avail
    while [[ ! -d "${d}" && "${d}" == /* && "${d}" != / ]]; do d="$(dirname "${d}")"; done
    avail="$(df --output=avail -B1M "${d}" 2>/dev/null | tail -n 1 | tr -d ' ' || true)"
    [[ "${avail}" =~ ^[0-9]+$ ]] || return 0
    (( avail < MIN_FREE_MB )) || return 0
    # 虚拟环境已经装好、只是重新执行一遍（多半什么都不用装）时不拦
    if [[ -f "${VENV}/pyvenv.cfg" && ${FORCE} == 0 ]]; then
        warn "${d} 所在磁盘只剩 ${avail} MB，需要重装依赖时可能不够"
        return 0
    fi
    die "${d} 所在磁盘只剩 ${avail} MB，虚拟环境装完约需 1 GB（再留下载缓存的余量，至少 ${MIN_FREE_MB} MB）。
    清理磁盘，或用 --venv 换到空间够的磁盘上"
}

sum_of() { awk -v f="$1" '$2 == f { print $1; exit }' <<<"${SUMS_TEXT}"; }

# 发布文件：二进制 wheel 与 pysurvive 都按 SHA256SUMS 校验
find_files() {
    WHEEL=""; PYS=""; SUM_WHEEL=""; SUM_PYS=""; SUMS_TEXT=""
    local want_pys=$(( NO_TRACKER == 0 ))
    if [[ ${SRC_MODE} == 1 && -z "${WHEELS}" ]]; then
        (( want_pys )) || return 0
        die "在源码目录里运行时，随附的 pysurvive 要从发布文件里装：用 --wheels 指定 pysurvive-…-cp310-cp310-linux_${ARCH}.whl 所在的目录
    （RevoHuman ${VERSION} 发布页的附件），或者加 --no-tracker 不装 Tracker 驱动"
    fi
    [[ -n "${WHEELS}" ]] || WHEELS="${SCRIPT_DIR}/.."
    [[ -d "${WHEELS}" ]] || die "--wheels 指定的 ${WHEELS} 不是目录"
    WHEELS="$(cd "${WHEELS}" && pwd -P)"
    info "发布文件目录：${WHEELS}"

    local f dups=()
    for f in "${WHEELS}"/SHA256SUMS?*; do [[ -f "${f}" ]] && dups+=("$(basename "${f}")"); done
    if [[ -f "${WHEELS}/SHA256SUMS" ]]; then
        # 兼容 Windows 换行与 sha256sum 的二进制模式标记（文件名前的 *）
        SUMS_TEXT="$(tr -d '\r' <"${WHEELS}/SHA256SUMS" | awk 'NF >= 2 { f = $2; sub(/^\*/, "", f); print $1, f }')"
    elif [[ ${SRC_MODE} == 0 ]]; then
        die "${WHEELS} 里没有 SHA256SUMS。${dups[*]:+目录里有 ${dups[*]}，像是浏览器重复下载时改了名，确认是 ${VERSION} 的之后改回 SHA256SUMS。}
    发布文件要和解开的源码包放在同一个目录里，例如：
        ~/revohuman-release/SHA256SUMS
        ~/revohuman-release/revohuman-${VERSION}-cp310-cp310-linux_${ARCH}.whl
        ~/revohuman-release/pysurvive-…-cp310-cp310-linux_${ARCH}.whl
        ~/revohuman-release/revohuman-${VERSION}/install_venv.sh   ← 本脚本
    这几个文件都在 RevoHuman ${VERSION} 发布页（Release）的附件里；放在别的目录时用 --wheels 指定"
    fi

    if [[ ${SRC_MODE} == 0 ]]; then
        WHEEL="revohuman-${VERSION}-cp310-cp310-linux_${ARCH}.whl"
        SUM_WHEEL="$(sum_of "${WHEEL}")"
        [[ -n "${SUM_WHEEL}" ]] || die "${WHEELS}/SHA256SUMS 里没有 ${WHEEL}：它不是 ${VERSION} 的，或这个版本没有提供 ${ARCH} 的 wheel。
    从 RevoHuman ${VERSION} 发布页（Release）下载 SHA256SUMS 放进这个目录后重新执行"
    fi
    if (( want_pys )); then
        local cand=()
        if [[ -n "${SUMS_TEXT}" ]]; then
            mapfile -t cand < <(awk -v a="${ARCH}" '$2 ~ ("^pysurvive-.*-cp310-cp310-linux_" a "\\.whl$") { print $2 }' <<<"${SUMS_TEXT}")
        else
            for f in "${WHEELS}"/pysurvive-*-cp310-cp310-linux_"${ARCH}".whl; do [[ -f "${f}" ]] && cand+=("$(basename "${f}")"); done
        fi
        [[ ${#cand[@]} == 1 ]] || die "${WHEELS}$([[ -n "${SUMS_TEXT}" ]] && printf '/SHA256SUMS') 里应当恰好有一个 linux_${ARCH} 的 pysurvive wheel，实际：${cand[*]:-没有}"
        PYS="${cand[0]}"
        SUM_PYS="$(sum_of "${PYS}")"
        PYS_VER="${PYS#pysurvive-}"; PYS_VER="${PYS_VER%%-*}"
    fi

    local missing=() bad=() want got
    for f in ${WHEEL:+"${WHEEL}"} ${PYS:+"${PYS}"}; do
        if [[ ! -e "${WHEELS}/${f}" ]]; then missing+=("${f}"); continue; fi
        if [[ ! -f "${WHEELS}/${f}" || ! -r "${WHEELS}/${f}" ]]; then bad+=("${f}（读不了）"); continue; fi
        want="$(sum_of "${f}")"
        got="$(sha256sum <"${WHEELS}/${f}" | cut -d' ' -f1)"
        if [[ -z "${want}" ]]; then
            warn "${WHEELS} 里没有 SHA256SUMS，${f} 没有校验"
            [[ "${f}" != "${PYS}" ]] || SUM_PYS="${got}"
        elif [[ "${got}" == "${want}" ]]; then ok "${f}"
        else bad+=("${f}"); fi
    done
    if [[ ${#missing[@]} -gt 0 ]]; then
        local partial=()
        for f in "${WHEELS}"/*.crdownload "${WHEELS}"/*.part "${WHEELS}"/*.download; do
            [[ -e "${f}" ]] && partial+=("$(basename "${f}")")
        done
        die "在 ${WHEELS} 里缺少这些发布文件：
$(printf '        %s\n' "${missing[@]}")${partial[*]:+
    这些文件还没下载完：
$(printf '        %s\n' "${partial[@]}")}
    从 RevoHuman ${VERSION} 发布页（Release）下载缺少的文件放进这个目录后重新执行$( (( want_pys )) && printf '；不用 Tracker 时可以加 --no-tracker')"
    fi
    [[ ${#bad[@]} -eq 0 ]] || die "这些文件与 SHA256SUMS 对不上（下载不完整或文件损坏），重新下载：
$(printf '        %s\n' "${bad[@]}")"
}

# 选建虚拟环境用的 Python：必须是 CPython 3.10（wheel 只有 cp310 的），而且带 venv 与 ensurepip
py_probe() {   # 打印「实现 版本 前缀 基础前缀」，不是能运行的 Python 返回非 0
    "$1" -c 'import platform, sys; print(platform.python_implementation(), "%d.%d.%d" % sys.version_info[:3], sys.prefix, sys.base_prefix)' 2>/dev/null
}

pick_python() {
    PY=""; PY_APT=()
    local c p line impl ver prefix base cands=()
    if [[ -n "${PY_ARG}" ]]; then cands=("${PY_ARG}"); else cands=(python3.10 python3); fi
    local tried=()
    for c in "${cands[@]}"; do
        p="$(command -v "${c}" 2>/dev/null || true)"
        if [[ -z "${p}" ]]; then tried+=("${c}（找不到）"); continue; fi
        line="$(py_probe "${p}")" || { tried+=("${c}（运行不了）"); continue; }
        read -r impl ver prefix base <<<"${line}"
        # 在别的虚拟环境里（例如激活着一个 venv 或 conda 环境）时改用它的基础解释器，免得新环境套在旧环境上
        if [[ "${prefix}" != "${base}" ]]; then
            local bp
            for bp in "${base}/bin/python${ver%.*}" "${base}/bin/python3"; do
                [[ -x "${bp}" ]] && { info "${p} 在虚拟环境 ${prefix} 里，改用它的基础解释器 ${bp}"; p="${bp}"; break; }
            done
        fi
        if [[ "${impl}" != CPython || "${ver%.*}" != 3.10 ]]; then
            tried+=("${c}（${p}：${impl} ${ver}）"); continue
        fi
        PY="${p}"; PY_VER="${ver}"
        break
    done
    if [[ -z "${PY}" ]]; then
        # Ubuntu 22.04 的系统 Python 就是 3.10，只是没装：第 1 步一起装上
        if [[ -z "${PY_ARG}" ]] && has_apt && [[ "$(osr ID)" == ubuntu && "$(osr VERSION_ID)" == 22.04 ]]; then
            PY="/usr/bin/python3.10"; PY_VER="3.10"; PY_APT=(python3.10 python3.10-venv)
            info "本机还没装 Python 3.10，第 1 步用 apt 安装 ${PY_APT[*]}"
            return 0
        fi
        die "需要 CPython 3.10 建虚拟环境（SDK 与 pysurvive 的 wheel 只有 cp310 的），没有找到：
$(printf '        %s\n' "${tried[@]}")
    Ubuntu 22.04 自带的 python3 就是 3.10；其他发行版装一个 CPython 3.10，例如
        Ubuntu 24.04：sudo add-apt-repository ppa:deadsnakes/ppa && sudo apt install python3.10 python3.10-venv
        conda：conda create -n py310 python=3.10，然后 --python ~/miniconda3/envs/py310/bin/python3.10
    再用 --python 指向它重新执行"
    fi
    # Debian / Ubuntu 的系统 Python 把 ensurepip 拆到了 python3.X-venv 包里
    if ! "${PY}" -c 'import venv, ensurepip' >/dev/null 2>&1; then
        local owner=""
        has_apt && owner="$(dpkg-query -S "$(readlink -f "${PY}")" 2>/dev/null | head -n 1 | cut -d: -f1 || true)"
        if [[ -n "${owner}" ]]; then
            PY_APT=(python3.10-venv)
            info "${PY} 缺 venv/ensurepip 模块，第 1 步用 apt 安装 python3.10-venv"
        else
            die "${PY} 缺 venv/ensurepip 模块，建不了虚拟环境。换一个完整的 CPython 3.10，用 --python 指定"
        fi
    fi
    ok "Python ${PY_VER}（${PY}）"
}

# ---------- 1. 系统依赖 ----------
# 共享库按 soname 查（ldconfig -p），包名用 Ubuntu 22.04 的（24.04 上这些名字也能装）：
#   OpenCV 解码要 libGL、GLib；MuJoCo 窗口走 GLX（libGL），无窗口出图走 EGL；glfw 开窗口要 X11 的几个库
SONAMES=(
    "libGL.so.1|libgl1"
    "libEGL.so.1|libegl1"
    "libglib-2.0.so.0|libglib2.0-0"
    "libgthread-2.0.so.0|libglib2.0-0"
    "libX11.so.6|libx11-6"
    "libXrandr.so.2|libxrandr2"
    "libXinerama.so.1|libxinerama1"
    "libXcursor.so.1|libxcursor1"
    "libXi.so.6|libxi6"
)

ldcache() {
    local lc
    for lc in "$(command -v ldconfig 2>/dev/null || true)" /sbin/ldconfig /usr/sbin/ldconfig; do
        [[ -n "${lc}" && -x "${lc}" ]] && { "${lc}" -p 2>/dev/null; return 0; }
    done
    return 1
}

check_system() {
    APT_PKGS=("${PY_APT[@]}"); MISSING_LIBS=()
    local cache tag pair so pkg
    if cache="$(ldcache)"; then
        tag="x86-64"; [[ ${ARCH} == x86_64 ]] || tag="AArch64"
        for pair in "${SONAMES[@]}"; do
            so="${pair%%|*}"; pkg="${pair#*|}"
            if awk -v s="${so}" -v t="${tag}" '$1 == s && index($0, t) { f = 1 } END { exit !f }' <<<"${cache}"; then continue; fi
            MISSING_LIBS+=("${so}")
            [[ " ${APT_PKGS[*]} " == *" ${pkg} "* ]] || APT_PKGS+=("${pkg}")
        done
        # 只装了 libegl1 而没有任何 EGL 实现时（例如 --no-install-recommends 装的），无窗口出图起不来
        if [[ " ${MISSING_LIBS[*]} " != *" libEGL.so.1 "* ]] && ! compgen -G "${SYSROOT}/usr/share/glvnd/egl_vendor.d/*.json" >/dev/null; then
            MISSING_LIBS+=("EGL 实现（/usr/share/glvnd/egl_vendor.d/ 是空的）")
            APT_PKGS+=(libegl-mesa0)
        fi
    else
        warn "本机没有 ldconfig，查不了 OpenGL 等共享库，跳过检查"
    fi
    # 基站蓝牙（bleak）经 D-Bus 用 BlueZ
    if has_apt; then
        dpkg_installed bluez || APT_PKGS+=(bluez)
    elif ! command -v bluetoothctl >/dev/null; then
        MISSING_LIBS+=("BlueZ（bluetoothctl）")
    fi
}

step_system() {
    say 1/6 "系统依赖"
    check_system
    if [[ ${#APT_PKGS[@]} == 0 && ${#MISSING_LIBS[@]} == 0 ]]; then
        ok "需要的软件包都已安装"
    elif ! has_apt; then
        [[ ${#PY_APT[@]} == 0 ]] || die "需要 python3.10-venv，但本机不是 Debian / Ubuntu，没有 apt"
        warn "本机不是 Debian / Ubuntu，下面这些要用发行版的包管理器自己装上：$(printf '\n        %s' "${MISSING_LIBS[@]}")"
    elif [[ ${NO_SYSTEM} == 1 ]]; then
        [[ ${#PY_APT[@]} == 0 ]] || die "建虚拟环境需要先装 ${PY_APT[*]}（--no-system：不自动安装）。
    执行 sudo apt-get install -y ${PY_APT[*]}，或用 --python 指定一个带 venv 模块的 CPython 3.10"
        warn "缺少这些软件包（--no-system：不安装），相机图像解码、三维可视化或基站蓝牙可能用不了：
        sudo apt-get install -y ${APT_PKGS[*]}"
    elif confirm "安装缺少的软件包（sudo apt-get install -y ${APT_PKGS[*]}）？"; then
        try_root apt-get update \
            || warn "sudo apt-get update 没有成功（常见原因是某个第三方软件源失效），用已有的软件包列表继续安装"
        as_root env DEBIAN_FRONTEND=noninteractive apt-get install -y "${APT_PKGS[@]}"
        [[ ${DRY_RUN} == 1 ]] || ok "已安装 ${APT_PKGS[*]}"
    else
        [[ ${#PY_APT[@]} == 0 ]] || die "建虚拟环境需要先装 ${PY_APT[*]}：sudo apt-get install -y ${PY_APT[*]}，之后重新执行"
        warn "没有安装 ${APT_PKGS[*]}，相机图像解码、三维可视化或基站蓝牙可能用不了。手动安装：
        sudo apt-get install -y ${APT_PKGS[*]}"
    fi
    if [[ ${NO_TRACKER} == 1 ]]; then
        info "Tracker 设备规则：--no-tracker，跳过"
    else
        step_udev
    fi
    step_dialout
}

# 规则文件里有没有一行（不算注释）把 28de:2300 的 USB 设备节点设成所有用户可读写。
# 只管 hidraw 的、只加 uaccess 的（只对本机图形登录的用户生效）、只写了别的产品号的都不算
udev_rule_ok() {
    awk '
        /^[[:space:]]*#/ { next }
        !/ATTRS?[{]idVendor[}]=="28de"/ { next }
        /ATTRS?[{]idProduct[}]==/ && !/ATTRS?[{]idProduct[}]=="2300"/ { next }
        /KERNEL=="hidraw/ || (/SUBSYSTEM=="/ && !/SUBSYSTEM=="usb"/) { next }
        /SUBSYSTEMS?=="usb"/ && /MODE:?="0?666"/ { found = 1; exit }
        END { exit !found }' "$1" 2>/dev/null
}

step_udev() {
    local d f b found="" near="" seen=" "
    # 目录按 udev 的优先级排：同名文件只有排在前面的那个生效
    # （/etc 下的同名空文件或指向 /dev/null 的链接，就是把 /lib 下的规则屏蔽掉）
    for d in /etc/udev/rules.d /run/udev/rules.d /lib/udev/rules.d /usr/lib/udev/rules.d; do
        for f in "${SYSROOT}${d}"/*.rules; do
            [[ -e "${f}" || -L "${f}" ]] || continue
            b="$(basename "${f}")"
            [[ "${seen}" != *" ${b} "* ]] || continue
            seen+="${b} "
            [[ -f "${f}" ]] || continue
            if udev_rule_ok "${f}"; then found="${f#"${SYSROOT}"}"; break 2; fi
            [[ -n "${near}" ]] || ! grep -qsE '^[^#]*28de' "${f}" || near="${f#"${SYSROOT}"}"
        done
    done
    [[ -n "${found}" || -z "${near}" ]] \
        || info "${near} 里提到了 28de，但没有把 Tracker 的 USB 设备设成所有用户可读写（SUBSYSTEM==\"usb\" … MODE=\"0666\"），不算数"
    if [[ -n "${found}" ]]; then
        ok "Tracker 设备规则已安装（${found}）"
        # 规则装了但已插着的 Tracker 仍然没有权限：多半是装规则之后没有重新插拔
        local sys node
        for sys in "${SYSROOT}"/sys/bus/usb/devices/*; do
            [[ "$(cat "${sys}/idVendor" 2>/dev/null):$(cat "${sys}/idProduct" 2>/dev/null)" == 28de:2300 ]] || continue
            node="$(printf '/dev/bus/usb/%03d/%03d' "$(cat "${sys}/busnum")" "$(cat "${sys}/devnum")")"
            # 容器里 /sys 能看到宿主的 USB 设备、/dev 下却没有对应节点：那不是权限问题，不报
            [[ -e "${SYSROOT}${node}" ]] || continue
            [[ -r "${SYSROOT}${node}" && -w "${SYSROOT}${node}" ]] || warn "Tracker ${node} 当前用户无权读写：重新插拔一次手套单元"
        done
        return 0
    fi
    if [[ ${NO_SYSTEM} == 1 ]]; then
        warn "没有安装 Tracker 设备规则（--no-system：不安装），用不了 Tracker。安装方法见手册《01-环境安装与系统要求》第 6 节"
        return 0
    fi
    if ! confirm "安装 Tracker 设备规则（写入 /etc/udev/rules.d/${UDEV_RULE_NAME}，需要 sudo）？"; then
        warn "没有安装 Tracker 设备规则，用不了 Tracker。之后可以重新执行本脚本，或按手册《01-环境安装与系统要求》第 6 节手动安装"
        return 0
    fi
    as_root tee "${SYSROOT}/etc/udev/rules.d/${UDEV_RULE_NAME}" >/dev/null <<'RULES'
# Vive Tracker（USB 有线，28de:2300）
SUBSYSTEM=="usb", ATTRS{idVendor}=="28de", ATTRS{idProduct}=="2300", MODE="0666"
KERNEL=="hidraw*", SUBSYSTEM=="hidraw", ATTRS{idVendor}=="28de", ATTRS{idProduct}=="2300", MODE="0666"
RULES
    # 只打印命令时不看本机有没有 udevadm：第 1 步装 bluez 会顺带装上 udev
    if [[ ${DRY_RUN} == 1 ]] || command -v udevadm >/dev/null; then
        try_root udevadm control --reload-rules || warn "udevadm 重新加载规则失败：重启主机后规则生效"
        try_root udevadm trigger --subsystem-match=usb --subsystem-match=hidraw || true
    else
        warn "本机没有 udevadm，规则要等重启之后才生效"
    fi
    UDEV_INSTALLED=1
}

# 手套是串口设备（/dev/ttyACM*），属 dialout 组（Arch 等发行版是 uucp 组）
step_dialout() {
    local me grp=""
    me="$(id -un)"
    if getent group dialout >/dev/null 2>&1; then grp=dialout
    elif getent group uucp >/dev/null 2>&1; then grp=uucp
    else info "本机没有 dialout 组，跳过（打不开手套串口时查一下 /dev/ttyACM* 的属组）"; return 0; fi
    if [[ " $(id -nG) " == *" ${grp} "* ]]; then
        ok "当前用户在 ${grp} 组里，能打开手套串口"
    elif [[ " $(id -nG "${me}") " == *" ${grp} "* ]]; then
        info "用户 ${me} 已经在 ${grp} 组里，但当前登录会话还没生效"
        NEED_RELOGIN=1
    elif [[ ${NO_SYSTEM} == 1 ]]; then
        warn "当前用户不在 ${grp} 组里，打不开手套串口（--no-system：不加组）。执行 sudo usermod -aG ${grp} ${me}，注销并重新登录"
    elif confirm "把用户 ${me} 加入 ${grp} 组，之后不用 sudo 就能打开手套串口（sudo usermod -aG ${grp} ${me}）？"; then
        as_root usermod -aG "${grp}" "${me}"
        NEED_RELOGIN=1
    else
        warn "当前用户不在 ${grp} 组里，打不开手套串口。手动执行 sudo usermod -aG ${grp} ${me}，注销并重新登录"
    fi
}

# ---------- 2. 虚拟环境 ----------
venv_state() {   # 已有的虚拟环境能不能沿用：能用返回 0，否则把原因写进 VENV_WHY
    VENV_WHY=""
    local line impl ver
    [[ -x "${VPY}" ]] || { VENV_WHY="里面没有 bin/python"; return 1; }
    line="$(py_probe "${VPY}")" || { VENV_WHY="里面的 Python 运行不了（建它的解释器被删了或升级了？）"; return 1; }
    read -r impl ver _ <<<"${line}"
    [[ "${impl}" == CPython && "${ver%.*}" == 3.10 ]] || { VENV_WHY="里面的 Python 是 ${impl} ${ver}，不是 CPython 3.10"; return 1; }
    "${VPY}" -m pip --version >/dev/null 2>&1 || { VENV_WHY="里面的 pip 用不了"; return 1; }
}

step_venv() {
    say 2/6 "虚拟环境"
    VPY="${VENV}/bin/python"
    if [[ -e "${VENV}" || -L "${VENV}" ]]; then
        if [[ -f "${VENV}/pyvenv.cfg" ]]; then
            if [[ ${FORCE} == 1 ]]; then
                [[ "${VIRTUAL_ENV:-}" != "${VENV}" ]] || warn "当前终端激活着 ${VENV}，装完之后重新 source ${VENV}/bin/activate"
                info "按 --force 删掉 ${VENV} 重新建"
                run rm -rf -- "${VENV}"
            elif venv_state; then
                ok "${VENV} 已存在（Python $(py_probe "${VPY}" | cut -d' ' -f2)），沿用"
                return 0
            else
                die "${VENV} 是个虚拟环境，但${VENV_WHY}。加 --force 删掉它重新建"
            fi
        elif [[ -d "${VENV}" && ! -L "${VENV}" && -z "$(ls -A "${VENV}" 2>/dev/null)" ]]; then
            :   # 空目录：直接在里面建
        else
            die "${VENV} 已经存在，但不是虚拟环境（里面没有 pyvenv.cfg），不会动它：用 --venv 换一个位置"
        fi
    fi
    [[ ${DRY_RUN} == 1 ]] || mkdir -p "$(dirname "${VENV}")" || die "建不了 $(dirname "${VENV}")"
    run "${PY}" -m venv --prompt "${PROMPT_NAME}" "${VENV}" || die "建虚拟环境失败（见上面的输出）"
    # 系统自带的 pip 太旧时，国内镜像的索引页会刷一屏警告；升级失败不影响后面安装
    info "升级虚拟环境里的 pip"
    pip_run install --upgrade pip || warn "升级 pip 没有成功，用自带的旧版 pip 继续"
    [[ ${DRY_RUN} == 1 ]] || ok "已建好 ${VENV}（激活后提示符前显示 (${PROMPT_NAME})）"
}

# ---------- 3～5. Python 包 ----------
# pip 的输出缩进后照常打印；失败时按输出里的特征给出处理建议
pip_run() {
    local rc=0
    info "\$ $(shq "${VPY}" -m pip "$@")"
    [[ ${DRY_RUN} == 0 ]] || return 0
    "${VPY}" -m pip --disable-pip-version-check "$@" 2>&1 | tee "${TMP}/pip.log" | sed -u 's/^/      /' || rc=${PIPESTATUS[0]}
    return "${rc}"
}

pip_die() {
    local hint=""
    if grep -qiE 'Temporary failure in name resolution|Could not resolve|Failed to establish a new connection|Network is unreachable|Read timed out|ReadTimeoutError|Connection (refused|reset|timed out)|certificate verify failed|No matching distribution found|ProxyError' "${TMP}/pip.log" 2>/dev/null; then
        hint="
    多半是网络问题：国内网络加 --mirror cn 改用阿里云镜像；需要代理时先 export https_proxy=… http_proxy=… 再执行"
    elif grep -qi 'No space left on device' "${TMP}/pip.log" 2>/dev/null; then
        hint="
    磁盘满了：清理后重新执行"
    fi
    die "$1${hint}
    改好后重新执行本脚本，已经装好的部分会跳过"
}

# 已装的版本：importlib.metadata 只读元数据，不导入模块（不会因为缺系统库而失败）
dist_versions() {
    "${VPY}" - "$@" <<'PY' 2>/dev/null
import sys
from importlib import metadata
for name in sys.argv[1:]:
    try:
        print(name, metadata.version(name))
    except metadata.PackageNotFoundError:
        print(name, "-")
PY
}

# dists_ok 版本 包名…：这些包都装着（第一个包的版本还要等于给定版本，给空串不比）
dists_ok() {
    local want="$1" out; shift
    out="$(dist_versions "$@")" || return 1
    [[ "$(wc -l <<<"${out}")" == "$#" ]] || return 1
    awk -v v="${want}" -v first="$1" '$2 == "-" || (v != "" && $1 == first && $2 != v) { bad = 1 } END { exit bad }' <<<"${out}"
}

# ver_lt a b：版本号 a 小于 b
ver_lt() { [[ "$1" != "$2" && "$(printf '%s\n' "$1" "$2" | sort -V | head -n 1)" == "$1" ]]; }

# mujoco 装着且版本在 MUJOCO_REQ 的范围内
mujoco_ok() {
    local v
    v="$(dist_versions mujoco | awk '{ print $2 }')" || return 1
    [[ -n "${v}" && "${v}" != - ]] && ! ver_lt "${v}" "${MUJOCO_MIN}" && ver_lt "${v}" "${MUJOCO_BELOW}"
}

stamp_ok() { [[ -f "$1" && "$(cat "$1" 2>/dev/null)" == "$2" ]]; }
write_stamp() { [[ ${DRY_RUN} == 1 ]] || { mkdir -p "$(dirname "$1")" && printf '%s\n' "$2" >"$1"; }; }

step_pysurvive() {
    say 3/6 "Tracker 驱动 pysurvive"
    [[ ${NO_TRACKER} == 0 ]] || { info "--no-tracker：跳过"; return 0; }
    local stamp="${VENV}/.revohuman/pysurvive.sha256"
    if [[ ${DRY_RUN} == 0 ]] && stamp_ok "${stamp}" "${SUM_PYS}" && [[ "$(dist_versions pysurvive)" == "pysurvive ${PYS_VER}" ]]; then
        ok "pysurvive ${PYS_VER} 已安装，跳过"; return 0
    fi
    # --no-deps 不能省：pysurvive 声明了一个图形界面依赖，SDK 用不到，而它在 Linux 上通常装不上
    pip_run install --no-deps --force-reinstall "${WHEELS}/${PYS}" || pip_die "安装 ${PYS} 失败（见上面 pip 的输出）"
    write_stamp "${stamp}" "${SUM_PYS}"
    [[ ${DRY_RUN} == 1 ]] || ok "pysurvive ${PYS_VER}"
}

step_sdk() {
    say 4/6 "RevoHuman SDK"
    local stamp="${VENV}/.revohuman/sdk.sha256" names=(revohuman pyserial PyYAML mcap lz4 opencv-python bleak mujoco glfw)
    if [[ ${SRC_MODE} == 1 ]]; then
        # 源码安装每次都重装 SDK 本身（源码可能改过）；依赖已经装好的 pip 会跳过
        pip_run install "${SCRIPT_DIR}[${EXTRAS}]" "${MUJOCO_REQ}" || pip_die "从 ${SCRIPT_DIR} 安装 SDK 失败（见上面 pip 的输出）"
        pip_run install --no-deps --force-reinstall "${SCRIPT_DIR}" || pip_die "从 ${SCRIPT_DIR} 重装 SDK 失败（见上面 pip 的输出）"
        [[ ${DRY_RUN} == 1 ]] || ok "revohuman ${VERSION}（源码安装，含 ${EXTRAS}）"
        return 0
    fi
    if [[ ${DRY_RUN} == 0 ]] && stamp_ok "${stamp}" "${SUM_WHEEL}" && dists_ok "${VERSION}" "${names[@]}" && mujoco_ok; then
        ok "revohuman ${VERSION}（含 ${EXTRAS}）已安装，跳过"; return 0
    fi
    pip_run install "${WHEELS}/${WHEEL}[${EXTRAS}]" "${MUJOCO_REQ}" || pip_die "安装 ${WHEEL} 失败（见上面 pip 的输出）"
    # 同一个版本号的 wheel 换过文件时（例如重新下载了修正版），pip 认为已经装好、不会替换：强制换成这一份
    pip_run install --no-deps --force-reinstall "${WHEELS}/${WHEEL}" || pip_die "重装 ${WHEEL} 失败（见上面 pip 的输出）"
    write_stamp "${stamp}" "${SUM_WHEEL}"
    [[ ${DRY_RUN} == 1 ]] || ok "revohuman ${VERSION}（含 ${EXTRAS}）"
}

step_viewer() {
    say 5/6 "可视化依赖"
    local req="${SCRIPT_DIR}/simulation/requirements.txt" want names
    # 与 simulation/run_viewer.sh 用同一个标记：它看到标记对得上就不再重装
    local stamp="${VENV}/.revohuman_viewer.stamp"
    want="$(sha256sum <"${req}" | cut -d' ' -f1)"
    mapfile -t names < <(sed -nE 's/^[[:space:]]*([A-Za-z0-9_.-]+).*/\1/p' "${req}")
    if [[ ${DRY_RUN} == 0 ]] && stamp_ok "${stamp}" "${want}" && dists_ok "" "${names[@]}" && mujoco_ok; then
        ok "已安装（${names[*]}），跳过"; return 0
    fi
    pip_run install -r "${req}" "${MUJOCO_REQ}" || pip_die "安装 ${req} 里的依赖失败（见上面 pip 的输出）"
    write_stamp "${stamp}" "${want}"
    [[ ${DRY_RUN} == 1 ]] || ok "${names[*]}"
}

# ---------- 6. 自检 ----------
step_check() {
    say 6/6 "自检"
    [[ ${DRY_RUN} == 0 ]] || { info "（--dry-run：跳过）"; return 0; }
    local mods=(revohuman mcap lz4 bleak cv2 mujoco glfw yaml serial numpy) m out failed=() v
    [[ ${NO_TRACKER} == 1 ]] || mods=(revohuman pysurvive "${mods[@]:1}")
    v="$("${VPY}" -c 'import revohuman; print(revohuman.__version__)' 2>/dev/null || true)"
    [[ "${v}" == "${VERSION}" ]] || die "虚拟环境里 revohuman 的版本是「${v:-导入失败}」，应为 ${VERSION}。加 --force 重新建虚拟环境后再试"
    for m in "${mods[@]}"; do
        out="$("${VPY}" -c "import ${m}" 2>&1)" || failed+=("${m}：$(tail -n 1 <<<"${out}")")
    done
    if [[ ${#failed[@]} -gt 0 ]]; then
        local hint="加 --force 重新建虚拟环境后再试"
        [[ "${failed[*]}" != *"cannot open shared object file"* ]] \
            || hint="缺的是系统库：去掉 --no-system 重新执行，让脚本装上第 1 步列出的软件包（非 Debian / Ubuntu 用发行版的包管理器装）"
        die "这些模块导入失败：
$(printf '        %s\n' "${failed[@]}")
    ${hint}"
    fi
    ok "导入 ${mods[*]}"
    [[ ${NO_TRACKER} == 1 ]] || ok "pysurvive $(dist_versions pysurvive | cut -d' ' -f2)"

    local cmds=("${VENV}/bin/revohuman-live" "${VENV}/bin/revohuman-lighthouses" "${VPY} -m revohuman") i
    for i in 0 1 2; do
        if [[ ${i} == 2 ]]; then out="$("${VPY}" -m revohuman --help 2>&1 >/dev/null)"
        else out="$("${cmds[i]}" --help 2>&1 >/dev/null)"; fi \
            || die "命令启动失败：${cmds[i]} --help
$(tail -n 5 <<<"${out}" | sed 's/^/        /')"
    done
    ok "命令行工具 revohuman-live、revohuman-lighthouses、python3 -m revohuman 能启动"

    # 无窗口跑一遍演示可视化：不接手套、不需要显示器，能出图就说明 MuJoCo 与 OpenGL（EGL）都能用
    out="$(REVOHUMAN_SIM_VENV="${VENV}" "${SCRIPT_DIR}/simulation/run_viewer.sh" --demo P1 --headless --seconds 2 \
               --snapshot "${TMP}/viewer-check.png" 2>&1 </dev/null)" && [[ -s "${TMP}/viewer-check.png" ]] \
        && ok "三维可视化能出图（无窗口演示）" \
        || warn "三维可视化无窗口出图失败（不影响 SDK 本身）。最后几行输出：
$(tail -n 5 <<<"${out}" | sed 's/^/        /')"
}

summary() {
    if [[ ${DRY_RUN} == 1 ]]; then
        say 完成 "--dry-run 结束，没有改动系统，也没有建虚拟环境"
        print_warnings
        return 0
    fi
    say 完成 "虚拟环境 ${VENV} 已就绪"
    [[ -z "${LOG}" ]] || info "日志：${LOG}"
    echo
    echo "下一步："
    [[ ${NEED_RELOGIN} != 1 ]] || info "注销并重新登录一次，dialout 组才生效（之前打不开手套串口）"
    [[ ${UDEV_INSTALLED} != 1 ]] || info "已经插着的手套单元重新插拔一次，让 Tracker 设备规则生效"
    info "source $(shq "${VENV}/bin/activate")      # 每个新终端先激活；deactivate 退出"
    info "python3 -m revohuman doctor                 # 只读体检：USB 拓扑、串口、注册表、Tracker、相机"
    info "python3 -m revohuman doctor --probe         # 另外短开串口读固件信息（正在采集时不要加）"
    info "$(shq "${SCRIPT_DIR}/simulation/run_sim_host.sh")    # 一条命令拉起广播与三维可视化"
    print_warnings
}

preflight
find_files
pick_python
if [[ ${MIRROR} == cn ]]; then
    export PIP_INDEX_URL="${CN_INDEX}"
    info "PyPI：阿里云镜像（${CN_INDEX}）"
elif [[ ${MIRROR} == official ]]; then
    export PIP_INDEX_URL="https://pypi.org/simple/"
    info "PyPI：官方源"
elif [[ -n "${PIP_INDEX_URL:-}" ]]; then
    info "PyPI：${PIP_INDEX_URL}（来自 PIP_INDEX_URL）"
fi
step_system
if [[ ${#PY_APT[@]} -gt 0 && ${DRY_RUN} == 0 ]]; then
    pick_python
    [[ ${#PY_APT[@]} == 0 ]] || die "装完 python3.10-venv 之后 ${PY} 仍然缺 venv/ensurepip 模块：用 --python 指定一个完整的 CPython 3.10"
fi
step_venv
step_pysurvive
step_sdk
step_viewer
step_check
summary
