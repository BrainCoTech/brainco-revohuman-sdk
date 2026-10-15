#!/usr/bin/env bash
# 一键构建 RevoHuman 的 Docker 镜像：检查并准备主机（Docker、docker 组、Tracker 设备规则），
# 核对发布文件，构建镜像，再在镜像里跑一遍检查。构建好之后用同目录的 run.sh 进入容器。
#
#   ./install.sh                  用官方软件源构建镜像 revohuman:0.1.1
#   ./install.sh --mirror cn      国内网络：Ubuntu、ROS 2、PyPI 改用阿里云镜像
#
# 需要 sudo 的步骤（安装 Docker、把用户加入 docker 组、安装 Tracker 设备规则）执行前逐项询问。
# 镜像已经用同一批发布文件、为当前用户构建过时不重复构建，只重跑检查，所以可以放心重复执行。
# 完整说明见使用手册《08-容器运行》。

# 用 sh 执行时转给 bash（下面用到 bash 的语法）；source 进当前 shell 会在出错时连 shell 一起退出，不允许
if [ -n "${ZSH_VERSION:-}" ] || { [ -z "${BASH_VERSION:-}" ] && [ ! -f "$0" ]; }; then
    echo "install.sh: 请直接执行 ./install.sh（或 bash install.sh）" >&2
    return 1 2>/dev/null || exit 1
fi
if [ -z "${BASH_VERSION:-}" ]; then exec bash "$0" "$@"; fi
if [[ "${BASH_SOURCE[0]}" != "$0" ]]; then
    echo "install.sh: 请直接执行 ./install.sh，不要用 source" >&2
    return 1 2>/dev/null || exit 1
fi

set -eo pipefail

VERSION="0.1.1"
SCRIPT_DIR="$(dirname "$(readlink -f "${BASH_SOURCE[0]}")")"
TAG="revohuman:${VERSION}"
RELEASE_DIR=""
MIRROR=official
BASE_IMAGE=""
REBUILD=0
NO_CACHE=0
SKIP_HOST=0
YES=0
DRY_RUN=0
MIN_DOCKER_MAJOR=23
MIN_FREE_GB=5
LOW_FREE_GB=10
UDEV_RULE_NAME="81-revohuman-tracker.rules"
LOG=""
NEED_RELOGIN=0
UDEV_INSTALLED=0
USE_SG=""
PYS=""
BUILDER_ARGS=()
# 只给测试用：把读写的主机系统文件（/etc、/lib、/usr/lib、/run、/proc、/sys 下的）换到另一个根目录下
SYSROOT="${REVOHUMAN_INSTALL_SYSROOT:-}"

usage() {
    cat <<EOF
用法：install.sh [选项]

构建 RevoHuman 的 Docker 镜像并检查它能用。发布文件（两个 wheel、源码包、SHA256SUMS）
默认在源码包的上一级目录，也就是解开源码包的那个目录。

选项：
  --mirror cn|official  软件源。cn：Ubuntu、ROS 2、PyPI 都用阿里云镜像（国内网络用）；默认 official
  --tag 名字            镜像名（默认 ${TAG}，与 run.sh 的默认值相同）
  --base-image 镜像     基础镜像（默认 ubuntu:22.04）。Docker Hub 访问不了时换成能拉到的 Ubuntu 22.04 镜像
  --release-dir 目录    发布文件所在的目录（默认：本脚本往上两级）
  --rebuild             镜像已经存在也重新构建
  --no-cache            不用 Docker 的构建缓存，从头构建（同时意味着 --rebuild）
  --skip-host-setup     不安装 Docker、不改 docker 组、不装 Tracker 设备规则，只检查与构建
  -y, --yes             需要 sudo 的步骤不逐项询问，直接执行
  --dry-run             只做检查并打印要执行的命令，不改动主机、不构建
  -h, --help            显示本帮助

当前 shell 里的 http_proxy、https_proxy、no_proxy 等代理变量会传给构建；代理指向本机时构建使用主机网络。
EOF
}

die() { printf '\ninstall.sh: %s\n' "$*" >&2; exit 1; }

while [[ $# -gt 0 ]]; do
    case "$1" in
        --*=*) set -- "${1%%=*}" "${1#*=}" "${@:2}"; continue ;;
        --mirror) MIRROR="${2:?--mirror 需要参数（cn 或 official）}"; shift 2 ;;
        --tag) TAG="${2:?--tag 需要参数}"; shift 2 ;;
        --base-image) BASE_IMAGE="${2:?--base-image 需要参数}"; shift 2 ;;
        --release-dir) RELEASE_DIR="${2:?--release-dir 需要参数}"; shift 2 ;;
        --rebuild) REBUILD=1; shift ;;
        --no-cache) NO_CACHE=1; REBUILD=1; shift ;;
        --skip-host-setup) SKIP_HOST=1; shift ;;
        -y|--yes) YES=1; shift ;;
        --dry-run) DRY_RUN=1; shift ;;
        -h|--help) usage; exit 0 ;;
        *) die "不认识的参数 $1（用 --help 查看用法）" ;;
    esac
done
case "${MIRROR}" in official|cn) ;; *) die "--mirror 只能是 cn 或 official，收到的是「${MIRROR}」" ;; esac
# 镜像名按 Docker 的规则检查（[主机[:端口]/]名字[:标签]）：名字的每一段是小写字母、数字，段内只能用单个 . 或 _、
# 两个 _、或连续的 - 隔开，不能以分隔符开头结尾；标签不超过 128 个字符
valid_tag() {
    local repo="$1" tag="" path dom comp='[a-z0-9]+(([._]|__|-+)[a-z0-9]+)*'
    if [[ "${1##*/}" == *:* ]]; then repo="${1%:*}"; tag="${1##*:}"; fi
    path="${repo}"
    if [[ "${repo}" == */* && ( "${repo%%/*}" == *[.:]* || "${repo%%/*}" == localhost ) ]]; then
        dom="${repo%%/*}"; path="${repo#*/}"
        [[ "${dom}" =~ ^(([A-Za-z0-9]([A-Za-z0-9-]*[A-Za-z0-9])?)(\.[A-Za-z0-9]([A-Za-z0-9-]*[A-Za-z0-9])?)*|\[[0-9A-Fa-f:]+\])(:[0-9]+)?$ ]] || return 1
    fi
    [[ ${#repo} -le 255 && "${path}" =~ ^${comp}(/${comp})*$ && "${tag}" =~ ^([A-Za-z0-9_][A-Za-z0-9_.-]{0,127})?$ ]]
}
valid_tag "${TAG}" \
    || die "镜像名「${TAG}」不合 Docker 的规则：名字部分只能用小写字母、数字，中间可以用 . _ - / 隔开（不能连用两个 . 或以它们结尾），例如 revohuman-alice:${VERSION}"

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
dry_stop() {
    info "（--dry-run：后面的检查要等上面的命令真正执行之后才能做，到此为止）"
    print_warnings
    exit 0
}

# 按 POSIX sh 的规则给参数加引号：sg 用 /bin/sh 解释命令串，打印给用户照抄的命令也用它
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

# docker 组还没在当前登录会话里生效时，经 sg 以该组身份执行 docker 与 run.sh
with_docker() {
    if [[ -n "${USE_SG}" ]]; then sg "${USE_SG}" -c "$(shq "$@")"; else "$@"; fi
}
dk() { with_docker docker "$@"; }

norm_arch() { case "$1" in x86_64|amd64) echo x86_64 ;; aarch64|arm64) echo aarch64 ;; *) echo "$1" ;; esac; }
df_arg() { sed -n "s/^ARG $1=//p" "${SCRIPT_DIR}/Dockerfile" | head -n 1; }
osr() { awk -F= -v k="$1" '$1 == k { gsub(/"/, "", $2); print $2 }' "${SYSROOT}/etc/os-release" 2>/dev/null || true; }

CTX=""
TEE_PID=""
CHECK_NAME=""
cleanup() {
    [[ -z "${CHECK_NAME}" ]] || dk rm -f "${CHECK_NAME}" >/dev/null 2>&1 </dev/null || true
    [[ -z "${CTX}" || ! -d "${CTX}" ]] || rm -rf -- "${CTX}"
    if [[ -n "${TEE_PID}" ]]; then exec 1>&- 2>&-; wait "${TEE_PID}" 2>/dev/null || true; fi
}
trap cleanup EXIT
trap 'exit 130' INT
trap 'exit 143' TERM

# ---------- 0. 运行条件 ----------
preflight() {
    [[ "$(id -u)" != 0 ]] || die "不要用 root 或 sudo 运行本脚本：镜像里的用户要与运行容器的普通用户一致。
    以普通用户执行 ./install.sh，需要 sudo 的步骤脚本会单独询问"
    local uid; uid="$(id -u)"
    [[ "${uid}" =~ ^[0-9]+$ ]] && (( uid > 100 && uid != 65534 )) \
        || die "当前用户号 ${uid} 在 Ubuntu 22.04 镜像里已被系统账户占用，镜像里建不出同号的用户。请换一个普通用户（用户号大于 100）运行"
    [[ "$(uname -s)" == Linux ]] || die "只支持 Linux 主机（手套、Tracker 要直接接给 Docker）"
    ARCH="$(norm_arch "$(uname -m)")"
    [[ "${ARCH}" == x86_64 || "${ARCH}" == aarch64 ]] || die "不支持的 CPU 架构 $(uname -m)：只提供 x86_64 与 aarch64 的 wheel"
    BUNDLE="revohuman-${VERSION}-bundle.tar.gz"
    WHEEL="revohuman-${VERSION}-cp310-cp310-linux_${ARCH}.whl"

    # 同一用户同时只跑一个 install.sh（并发构建会互相覆盖镜像名、争抢临时目录）；--dry-run 不改动什么，不用排队
    local lock="${XDG_RUNTIME_DIR:-/tmp}/revohuman-install-$(id -u).lock"
    if [[ ${DRY_RUN} == 0 ]] && command -v flock >/dev/null && { : >>"${lock}"; } 2>/dev/null; then
        exec 9>>"${lock}"
        flock -n 9 || die "另一个 install.sh 正在运行（锁文件 ${lock}），等它结束后再执行"
    fi

    # 全部输出另存一份日志，出问题时把它发给技术支持
    local logdir="${XDG_STATE_HOME:-${HOME}/.local/state}/revohuman-docker"
    if mkdir -p "${logdir}" 2>/dev/null && [[ -w "${logdir}" ]]; then
        LOG="${logdir}/install-$(date +%Y%m%d-%H%M%S).log"
        exec > >(tee -ai --output-error=warn "${LOG}") 2>&1
        TEE_PID=$!
    fi
    echo "RevoHuman ${VERSION} Docker 镜像一键构建（$(date '+%F %T')）"
    [[ -z "${LOG}" ]] || echo "日志：${LOG}"
    [[ ${DRY_RUN} == 0 ]] || echo "--dry-run：只检查、打印命令，不改动主机、不构建"
    if grep -qsi microsoft "${SYSROOT}/proc/sys/kernel/osrelease"; then
        warn "检测到 WSL：手套与 Tracker 要先用 usbipd 接进 WSL，容器里才看得到"
    fi
}

# ---------- 1. 发布文件 ----------
sum_of() { awk -v f="$1" '$2 == f { print $1; exit }' <<<"${SUMS_TEXT}"; }

extras_text() {   # 目录里同类但版本不同的文件（例如旧版本的 wheel）不能拿来代替、没下载完的文件，单独列出免得误会
    local f others=() partial=()
    for f in "${RELEASE_DIR}"/revohuman-*-linux_"${ARCH}".whl "${RELEASE_DIR}"/revohuman-*-bundle.tar.gz \
             "${RELEASE_DIR}"/pysurvive-*-linux_"${ARCH}".whl; do
        [[ -f "${f}" ]] || continue
        f="$(basename "${f}")"
        [[ "${f}" == "${WHEEL}" || "${f}" == "${BUNDLE}" ]] && continue
        if [[ -n "${PYS}" ]]; then [[ "${f}" == "${PYS}" ]] && continue
        else [[ "${f}" == pysurvive-* ]] && continue; fi    # SHA256SUMS 不对时判断不了 pysurvive 是不是这个版本的
        others+=("${f}")
    done
    for f in "${RELEASE_DIR}"/*.crdownload "${RELEASE_DIR}"/*.part "${RELEASE_DIR}"/*.download; do
        [[ -e "${f}" ]] && partial+=("$(basename "${f}")")
    done
    [[ ${#others[@]} == 0 ]] || printf '\n    目录里的这些文件版本不同，不能代替：\n%s' "$(printf '        %s\n' "${others[@]}")"
    [[ ${#partial[@]} == 0 ]] || printf '\n    这些文件还没下载完：\n%s' "$(printf '        %s\n' "${partial[@]}")"
    return 0
}

step_files() {
    say 1/6 "核对发布文件"
    [[ -n "${RELEASE_DIR}" ]] || RELEASE_DIR="${SCRIPT_DIR}/../.."
    [[ -d "${RELEASE_DIR}" ]] || die "--release-dir 指定的 ${RELEASE_DIR} 不是目录"
    RELEASE_DIR="$(cd "${RELEASE_DIR}" && pwd -P)"
    info "发布文件目录：${RELEASE_DIR}"
    [[ -f "${SCRIPT_DIR}/Dockerfile" ]] || die "${SCRIPT_DIR} 里没有 Dockerfile：install.sh 要在解开的源码包的 docker/ 目录里运行"

    local f dups=()
    for f in "${RELEASE_DIR}"/SHA256SUMS?*; do [[ -f "${f}" ]] && dups+=("$(basename "${f}")"); done
    if [[ ! -f "${RELEASE_DIR}/SHA256SUMS" ]]; then
        die "${RELEASE_DIR} 里没有 SHA256SUMS。${dups[*]:+目录里有 ${dups[*]}，像是浏览器重复下载时改了名，确认是 ${VERSION} 的之后改回 SHA256SUMS。}
    发布文件要和解开的源码包放在同一个目录里，例如：
        ~/revohuman-release/SHA256SUMS
        ~/revohuman-release/${BUNDLE}
        ~/revohuman-release/${WHEEL}
        ~/revohuman-release/pysurvive-…-cp310-cp310-linux_${ARCH}.whl
        ~/revohuman-release/revohuman-${VERSION}/docker/install.sh   ← 本脚本
    这几个文件都在 RevoHuman ${VERSION} 发布页（Release）的附件里；放在别的目录时用 --release-dir 指定"
    fi
    # 兼容 Windows 换行与 sha256sum 的二进制模式标记（文件名前的 *）
    SUMS_TEXT="$(tr -d '\r' <"${RELEASE_DIR}/SHA256SUMS" | awk 'NF >= 2 { f = $2; sub(/^\*/, "", f); print $1, f }')"

    if [[ -z "$(sum_of "${BUNDLE}")" ]]; then
        # SHA256SUMS 是别的版本的（常见于旧版本的下载目录里解开了新版本的源码包）：把缺的文件一次列全
        local other missing=()
        other="$(grep -oE 'revohuman-[^ ]+-bundle\.tar\.gz' <<<"${SUMS_TEXT}" | head -n 1 | sed -E 's/^revohuman-(.*)-bundle\.tar\.gz$/\1/' || true)"
        for f in "${WHEEL}" "${BUNDLE}"; do [[ -e "${RELEASE_DIR}/${f}" ]] || missing+=("${f}"); done
        compgen -G "${RELEASE_DIR}/pysurvive-*-cp310-cp310-linux_${ARCH}.whl" >/dev/null || missing+=("pysurvive-…-cp310-cp310-linux_${ARCH}.whl")
        die "${RELEASE_DIR}/SHA256SUMS 不是 ${VERSION} 的${other:+（是 ${other} 的）}。${dups[*]:+目录里还有 ${dups[*]}，可能就是新下载的那份被浏览器改了名。}${missing[*]:+
    目录里还缺少 ${VERSION} 的这些文件：
$(printf '        %s\n' "${missing[@]}")}$(extras_text)
    从 RevoHuman ${VERSION} 发布页（Release）下载 SHA256SUMS${missing[*]:+ 与缺少的文件}放进这个目录（替换旧的 SHA256SUMS）后重新执行"
    fi
    PYS="$(awk -v a="${ARCH}" '$2 ~ ("^pysurvive-.*-cp310-cp310-linux_" a "\\.whl$") { print $2 }' <<<"${SUMS_TEXT}")"
    [[ -n "${PYS}" && "${PYS}" != *$'\n'* ]] || die "SHA256SUMS 里应当恰好有一个 linux_${ARCH} 的 pysurvive wheel，实际：${PYS:-没有}"
    [[ -n "$(sum_of "${WHEEL}")" ]] || die "SHA256SUMS 里没有 ${WHEEL}：这个版本没有提供 ${ARCH} 的 wheel"

    local missing=() bad=() want got
    for f in "${PYS}" "${WHEEL}" "${BUNDLE}"; do
        if [[ ! -e "${RELEASE_DIR}/${f}" ]]; then missing+=("${f}"); continue; fi
        if [[ ! -f "${RELEASE_DIR}/${f}" || ! -r "${RELEASE_DIR}/${f}" ]]; then bad+=("${f}（读不了）"); continue; fi
        want="$(sum_of "${f}")"
        got="$(sha256sum <"${RELEASE_DIR}/${f}" | cut -d' ' -f1)"
        if [[ "${got}" == "${want}" ]]; then ok "${f}"; else bad+=("${f}"); fi
    done
    if [[ ${#missing[@]} -gt 0 ]]; then
        die "在 ${RELEASE_DIR} 里缺少这些发布文件：
$(printf '        %s\n' "${missing[@]}")$(extras_text)
    从 RevoHuman ${VERSION} 发布页（Release）下载缺少的文件放进这个目录后重新执行"
    fi
    [[ ${#bad[@]} -eq 0 ]] || die "这些文件与 SHA256SUMS 对不上（下载不完整或文件损坏），重新下载：
$(printf '        %s\n' "${bad[@]}")"
    SUM_PYS="$(sum_of "${PYS}")"; SUM_WHEEL="$(sum_of "${WHEEL}")"; SUM_BUNDLE="$(sum_of "${BUNDLE}")"
}

# ---------- 2. Docker ----------
install_docker() {
    local id like pkgs cand upd=""
    id="$(osr ID)"; like="$(osr ID_LIKE)"
    if ! command -v apt-get >/dev/null || [[ " ${id} ${like} " != *" ubuntu "* && " ${id} ${like} " != *" debian "* ]]; then
        die "找不到 docker 命令。请按 Docker 官方文档安装 Docker Engine（${MIN_DOCKER_MAJOR}.0 或更新）后重新执行本脚本"
    fi
    # 已经配置了 Docker 官方软件源的，装官方包；否则装发行版自带的 docker.io（两者不能混装）
    if grep -qs 'download\.docker\.com' "${SYSROOT}/etc/apt/sources.list" "${SYSROOT}"/etc/apt/sources.list.d/*; then
        pkgs=(docker-ce docker-ce-cli containerd.io docker-buildx-plugin)
    else
        pkgs=(docker.io)
    fi
    [[ ${SKIP_HOST} == 0 ]] || die "找不到 docker 命令（--skip-host-setup：不自动安装）。安装：sudo apt-get install ${pkgs[*]}"
    confirm "本机没有 Docker，安装它（sudo apt-get install ${pkgs[*]}）？" \
        || die "没有 Docker 无法继续。手动安装：sudo apt-get update && sudo apt-get install -y ${pkgs[*]}"
    if ! try_root apt-get update; then
        upd="（前面的 sudo apt-get update 没有成功，先按它的报错修好软件源再试）"
        warn "sudo apt-get update 没有成功（常见原因是某个第三方软件源失效），用已有的软件包列表继续安装"
    fi
    if [[ ${pkgs[0]} == docker.io && ${DRY_RUN} == 0 ]]; then
        cand="$(apt-cache policy docker.io 2>/dev/null | awk '/Candidate:/ { print $2 }' || true)"
        cand="${cand#*:}"
        if [[ -z "${cand}" || "${cand}" == "(none)" ]]; then
            die "软件源里没有 docker.io${upd}。请按 Docker 官方文档安装 Docker Engine 后重新执行"
        elif [[ "${cand%%.*}" =~ ^[0-9]+$ ]] && (( ${cand%%.*} < MIN_DOCKER_MAJOR )); then
            die "发行版自带的 docker.io 是 ${cand}，低于需要的 ${MIN_DOCKER_MAJOR}.0。请按 Docker 官方文档安装 docker-ce 后重新执行"
        fi
    fi
    try_root apt-get install -y "${pkgs[@]}" || die "安装 ${pkgs[*]} 没有成功${upd}。没有 sudo 权限时请让管理员执行：sudo apt-get install -y ${pkgs[*]}"
    if [[ ${pkgs[0]} == docker.io ]]; then
        try_root apt-get install -y docker-buildx || warn "docker-buildx 装不上，构建将改用旧版构建器"
    fi
    if command -v systemctl >/dev/null; then try_root systemctl enable --now docker || true; fi
    [[ ${DRY_RUN} == 0 ]] || dry_stop
    command -v docker >/dev/null || die "装完之后仍然找不到 docker 命令"
}

# 连上 Docker：服务没起来就启动它，没权限就加 docker 组（本次经 sg 继续，免得要求用户先注销）
connect_docker() {
    local attempt out sock grp me
    me="$(id -un)"
    sock="${DOCKER_SOCK#unix://}"
    for attempt in 1 2; do
        if docker info >/dev/null 2>&1; then return 0; fi
        out="$(docker info 2>&1 >/dev/null || true)"
        if [[ "${out,,}" == *"permission denied"* ]] || [[ -S "${sock}" && ! -w "${sock}" ]]; then
            grp="$(stat -c %G "${sock}" 2>/dev/null || true)"
            [[ -n "${grp}" && "${grp}" != root && "${grp}" != UNKNOWN ]] \
                || die "当前用户无权访问 ${sock}，它的属组（${grp:-未知}）也不是可以加入的组：检查 Docker 的安装方式"
            if [[ " $(id -nG "${me}") " == *" ${grp} "* ]]; then
                info "用户 ${me} 已经在 ${grp} 组里，但当前登录会话还没生效（加组后要注销并重新登录）；本次先经 sg 以 ${grp} 组身份执行 docker"
            else
                [[ ${SKIP_HOST} == 0 ]] || die "当前用户不在 ${grp} 组里，用不了 Docker（--skip-host-setup：不自动加组）。
    执行 sudo usermod -aG ${grp} ${me}，注销并重新登录后再运行本脚本"
                confirm "把用户 ${me} 加入 ${grp} 组，之后不用 sudo 就能用 docker（sudo usermod -aG ${grp} ${me}）？" \
                    || die "不加组用不了 Docker。手动执行 sudo usermod -aG ${grp} ${me}，注销并重新登录后再运行本脚本"
                as_root usermod -aG "${grp}" "${me}"
                [[ ${DRY_RUN} == 0 ]] || dry_stop
            fi
            command -v sg >/dev/null || die "找不到 sg 命令：注销并重新登录一次，让 ${grp} 组生效后再运行本脚本"
            USE_SG="${grp}"; NEED_RELOGIN=1
            out="$(dk info 2>&1 >/dev/null)" || die "经 sg 仍然连不上 Docker，注销并重新登录后再试。docker info 的输出：
$(head -n 5 <<<"${out}")"
            return 0
        fi
        # 连不上又不是权限问题：多半是服务没在运行
        if [[ ${attempt} == 1 && ${SKIP_HOST} == 0 && ( "${sock}" == /var/run/docker.sock || "${sock}" == /run/docker.sock ) ]] \
            && command -v systemctl >/dev/null; then
            confirm "Docker 服务没有运行，启动它并设为开机自启（sudo systemctl enable --now docker）？" \
                || die "Docker 服务没有运行。手动执行 sudo systemctl enable --now docker 后再运行本脚本"
            as_root systemctl enable --now docker
            [[ ${DRY_RUN} == 0 ]] || dry_stop
            # 等服务起来；连上了、或者变成权限问题（交给下一轮处理）就不再等
            local i
            for i in $(seq 1 20); do
                out="$(docker info 2>&1 >/dev/null)" && return 0
                [[ "${out,,}" != *"permission denied"* ]] || break
                sleep 1
            done
            continue
        fi
        break
    done
    die "连不上 Docker 服务。docker info 的输出：
$(head -n 5 <<<"${out}")"
}

step_docker() {
    say 2/6 "检查 Docker"
    command -v docker >/dev/null || install_docker
    # docker 连的必须是本机的服务（DOCKER_HOST 或 docker context 指向别的机器时，手套、Tracker 接不进去）
    if [[ -n "${DOCKER_HOST}" ]]; then
        DOCKER_SOCK="${DOCKER_HOST}"
        [[ "${DOCKER_SOCK}" == unix://* ]] || die "DOCKER_HOST=${DOCKER_HOST} 指向别的机器：手套、Tracker 只能接给本机的 Docker。unset DOCKER_HOST 后再运行"
    else
        DOCKER_SOCK="$(docker context inspect -f '{{.Endpoints.docker.Host}}' 2>/dev/null || true)"
        [[ -n "${DOCKER_SOCK}" ]] || DOCKER_SOCK="unix:///var/run/docker.sock"
        [[ "${DOCKER_SOCK}" == unix://* ]] || die "当前的 docker context（$(docker context show 2>/dev/null || echo 未知)）指向 ${DOCKER_SOCK}，不是本机的 Docker：
    手套、Tracker 只能接给本机的 Docker。执行 docker context use default 切回本机后再运行"
    fi
    local v
    v="$(docker --version 2>&1 || true)"
    [[ "${v,,}" != *podman* ]] || die "docker 命令实际是 Podman（${v}）。容器的用户映射与设备访问方式不同，run.sh 不支持；请安装 Docker Engine"
    [[ "$(command -v docker)" != /snap/* ]] \
        || warn "用的是 snap 版 Docker，它的沙箱可能挡住设备与目录的访问；遇到问题时改用 apt 安装的 docker.io"
    connect_docker

    local line os arch secopts major
    line="$(dk info -f '{{.ServerVersion}}|{{.OperatingSystem}}|{{.Architecture}}|{{.DockerRootDir}}|{{range .SecurityOptions}}{{.}} {{end}}' 2>/dev/null)" \
        || die "读不了 docker info"
    IFS='|' read -r SERVER_VER os arch DOCKER_ROOT secopts <<<"${line}"
    major="${SERVER_VER%%.*}"
    if [[ "${major}" =~ ^[0-9]+$ ]]; then
        (( major >= MIN_DOCKER_MAJOR )) || die "Docker 版本是 ${SERVER_VER}，需要 ${MIN_DOCKER_MAJOR}.0 或更新。
    Ubuntu：sudo apt-get update && sudo apt-get install --only-upgrade docker.io；其他情况按 Docker 官方文档升级"
    else
        warn "认不出 Docker 的版本号「${SERVER_VER}」，继续"
    fi
    [[ "${os}" != *"Docker Desktop"* ]] || die "当前连的是 Docker Desktop：它把容器跑在虚拟机里，手套、Tracker 与图形界面都接不进容器。
    请改用 Docker Engine（Ubuntu：sudo apt-get install docker.io），并执行 docker context use default 切换过去"
    [[ "$(norm_arch "${arch}")" == "${ARCH}" ]] || die "Docker 服务的架构是 ${arch}，本机是 ${ARCH}：docker 连到的不是本机的 Docker 服务（检查 docker context）"
    [[ "${secopts}" != *rootless* ]] || die "当前是 rootless 模式的 Docker：容器里的用户号会被映射成别的号，
    挂载目录的属主与手套串口的权限都对不上。请改用普通（rootful）Docker"
    [[ "${secopts}" != *name=userns* ]] || die "Docker 开启了 userns-remap：容器里的用户号会被映射成别的号，
    挂载目录的属主与手套串口的权限都对不上。请在 /etc/docker/daemon.json 里关掉 userns-remap 后重启 Docker"
    ok "Docker ${SERVER_VER}${USE_SG:+（本次经 sg ${USE_SG} 执行）}"
    # run.sh 在 Docker 25 及以上把主机的 /dev 挂进容器，插拔手套即时生效；更早的版本只能逐个挂当时在的设备节点
    if [[ "${major}" =~ ^[0-9]+$ ]] && (( major < 25 )); then
        warn "Docker ${SERVER_VER} 低于 25：容器起来之后重新插拔手套，要执行 ./run.sh --restart 才能用上（Docker 25 及以上不用）"
    fi

    if dk buildx version >/dev/null 2>&1; then
        BUILDKIT=1
    else
        local pkg=docker-buildx
        [[ "$(dpkg-query -W -f='${Status}' docker-ce-cli 2>/dev/null || true)" != *"install ok"* ]] || pkg=docker-buildx-plugin
        if [[ ${SKIP_HOST} == 0 ]] && command -v apt-get >/dev/null \
            && confirm "没有 buildx 插件，安装它（sudo apt-get install ${pkg}）？" && try_root apt-get install -y "${pkg}" \
            && { [[ ${DRY_RUN} == 1 ]] || dk buildx version >/dev/null 2>&1; }; then
            BUILDKIT=1
        else
            BUILDKIT=0
            warn "没有 buildx 插件，改用旧版构建器构建（Docker 已标为弃用，但仍能完成构建）"
        fi
    fi
    export DOCKER_BUILDKIT="${BUILDKIT}"
    if [[ ${BUILDKIT} == 1 ]]; then
        # 当前 builder 若是 docker-container 等驱动（docker buildx create --use 建的），构建不在本机 Docker 里进行：
        # 用不了本地已有的基础镜像，--network host 也要额外授权。改用本机 Docker 自带的那个 builder
        local drv cur name
        cur="$(dk buildx inspect 2>/dev/null || true)"
        drv="$(awk '/^Driver:/ { print $2; exit }' <<<"${cur}")"
        if [[ -n "${drv}" && "${drv}" != docker ]]; then
            name="$(dk context show 2>/dev/null || echo default)"
            cur="$(dk buildx inspect "${name}" 2>/dev/null || true)"
            if [[ "$(awk '/^Driver:/ { print $2; exit }' <<<"${cur}")" == docker ]]; then
                BUILDER_ARGS=(--builder "${name}")
                info "当前的 buildx builder 用的是 ${drv} 驱动，本次构建改用本机 Docker 自带的 builder「${name}」"
            else
                warn "当前的 buildx builder 用的是 ${drv} 驱动，构建用不了本机已有的基础镜像；拉取失败时执行 docker buildx use default 后重试"
            fi
        fi
        ok "BuildKit 可用"
    fi

    if command -v getenforce >/dev/null && [[ "$(getenforce 2>/dev/null)" == Enforcing ]]; then
        warn "SELinux 处于 Enforcing：容器访问挂载目录与设备可能被拒绝"
    fi
}

# ---------- 3. Tracker 设备规则 ----------
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
    say 3/6 "Tracker 设备规则"
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
        ok "已安装（${found}）"
        # 规则装了但已插着的 Tracker 仍然没有权限：多半是装规则之后没有重新插拔
        local sys node
        for sys in "${SYSROOT}"/sys/bus/usb/devices/*; do
            [[ "$(cat "${sys}/idVendor" 2>/dev/null):$(cat "${sys}/idProduct" 2>/dev/null)" == 28de:2300 ]] || continue
            node="$(printf '/dev/bus/usb/%03d/%03d' "$(cat "${sys}/busnum")" "$(cat "${sys}/devnum")")"
            [[ -r "${node}" && -w "${node}" ]] || warn "Tracker ${node} 当前用户无权读写：重新插拔一次手套单元"
        done
        return 0
    fi
    if [[ ${SKIP_HOST} == 1 ]]; then
        warn "没有安装 Tracker 设备规则（--skip-host-setup：不安装），容器里会用不了 Tracker。安装方法见手册《08-容器运行》第 2 节"
        return 0
    fi
    if ! confirm "安装 Tracker 设备规则（写入 /etc/udev/rules.d/${UDEV_RULE_NAME}，需要 sudo）？"; then
        warn "没有安装 Tracker 设备规则，容器里会用不了 Tracker。之后可以重新执行本脚本，或按手册《08-容器运行》第 2 节手动安装"
        return 0
    fi
    as_root tee "${SYSROOT}/etc/udev/rules.d/${UDEV_RULE_NAME}" >/dev/null <<'RULES'
# Vive Tracker（USB 有线，28de:2300）
SUBSYSTEM=="usb", ATTRS{idVendor}=="28de", ATTRS{idProduct}=="2300", MODE="0666"
KERNEL=="hidraw*", SUBSYSTEM=="hidraw", ATTRS{idVendor}=="28de", ATTRS{idProduct}=="2300", MODE="0666"
RULES
    if command -v udevadm >/dev/null; then
        try_root udevadm control --reload-rules || warn "udevadm 重新加载规则失败：重启主机后规则生效"
        try_root udevadm trigger --subsystem-match=usb --subsystem-match=hidraw || true
    else
        warn "本机没有 udevadm，规则要等重启之后才生效"
    fi
    UDEV_INSTALLED=1
}

# ---------- 4. 构建前的准备 ----------
reach_url() {   # 能否在限定时间内访问 URL：0 能、1 不能、2 本机没有 curl 与 wget。与构建一样走当前 shell 的代理变量
    if command -v curl >/dev/null; then
        curl -fsSL -o /dev/null -r 0-1023 --connect-timeout 5 --max-time 15 "$1" 2>/dev/null || return 1
    elif command -v wget >/dev/null; then
        wget -q -O /dev/null --timeout=10 --tries=1 "$1" 2>/dev/null || return 1
    else
        return 2
    fi
}

step_plan() {
    say 4/6 "构建前检查"
    BASE="${BASE_IMAGE:-$(df_arg BASE_IMAGE)}"
    # 镜像标签记下构建它的输入：Dockerfile、三个发布文件与用户号（org.revohuman.inputs），
    # 基础镜像另记名字与 ID（org.revohuman.base、org.revohuman.base-id）。软件源不算：换个源装出来的东西相同
    INPUTS="$({ sha256sum <"${SCRIPT_DIR}/Dockerfile"
                printf '%s\n' "${SUM_PYS} ${PYS}" "${SUM_WHEEL} ${WHEEL}" "${SUM_BUNDLE} ${BUNDLE}" \
                              "uid=$(id -u) gid=$(id -g)"; } | sha256sum | cut -c1-16)"
    local binfo base_arch=""
    BASE_ID=""
    if binfo="$(dk image inspect -f '{{.Id}}|{{.Architecture}}' "${BASE}" 2>/dev/null)"; then
        IFS='|' read -r BASE_ID base_arch <<<"${binfo}"
    fi
    NEED_BUILD=1
    local labels iu ig ii ib iid me
    if labels="$(dk image inspect -f '{{index .Config.Labels "org.revohuman.uid"}}|{{index .Config.Labels "org.revohuman.gid"}}|{{index .Config.Labels "org.revohuman.inputs"}}|{{index .Config.Labels "org.revohuman.base"}}|{{index .Config.Labels "org.revohuman.base-id"}}' "${TAG}" 2>/dev/null)"; then
        labels="${labels//<no value>/}"
        IFS='|' read -r iu ig ii ib iid <<<"${labels}"
        if [[ -n "${iu}" && ( "${iu}" != "$(id -u)" || "${ig}" != "$(id -g)" ) && ${REBUILD} == 0 ]]; then
            # 建议的镜像名要能直接用：用户名转成小写字母、数字与单个 -，去掉首尾的 -，转完是空的就用用户号
            me="$(id -un | tr 'A-Z' 'a-z' | tr -c 'a-z0-9\n' '-' | sed -E 's/-+/-/g; s/^-//; s/-$//')"
            [[ -n "${me}" ]] || me="user$(id -u)"
            die "镜像 ${TAG} 已经存在，是为用户号 ${iu}:${ig} 构建的，当前用户是 $(id -u):$(id -g)（可能是本机另一个用户的镜像）。
    给自己构建一份：./install.sh --tag revohuman-${me}:${VERSION}，之后用 ./run.sh --image revohuman-${me}:${VERSION}；
    确认要覆盖它时加 --rebuild"
        fi
        if [[ ${REBUILD} == 1 ]]; then
            info "镜像 ${TAG} 已经存在，按 --rebuild 重新构建并替换它"
        elif [[ "${ii}" != "${INPUTS}" ]]; then
            info "镜像 ${TAG} 已经存在，但不是用这批发布文件、为当前用户构建的；重新构建并替换它"
        elif [[ -n "${iid}" && "${iid}" == "${BASE_ID}" ]] || [[ -n "${ib}" && "${ib}" == "${BASE}" ]]; then
            # 基础镜像 ID 相同（同一个镜像换了名字，例如 --base-image 写全了 ubuntu:22.04，或镜像站的同一个镜像），
            # 或者名字相同，都算同一个基础镜像。名字相同而 ID 变了，是构建之后重新拉取过：只提醒，不自动重建
            # （重建要十几分钟，装进镜像的软件包反正都在构建时从软件源下载）；本地的基础镜像删掉了也不用重建
            NEED_BUILD=0
            ok "镜像 ${TAG} 已经用这批发布文件、为当前用户构建过，跳过构建（要重新构建加 --rebuild）"
            [[ -z "${BASE_ID}" || -z "${iid}" || "${iid}" == "${BASE_ID}" ]] \
                || warn "基础镜像 ${BASE} 在镜像 ${TAG} 构建之后更新过（重新拉取过）；要基于新的基础镜像构建，加 --rebuild"
            return 0
        else
            info "镜像 ${TAG} 是基于另一个基础镜像（${ib:-未知}）构建的；基于 ${BASE} 重新构建并替换它"
        fi
    fi

    # 磁盘：镜像数据可能在 Docker 数据目录，也可能在 containerd 的目录（containerd 镜像存储）
    local dir avail min_mb=""
    for dir in "${DOCKER_ROOT}" /var/lib/containerd; do
        [[ -n "${dir}" && -d "${dir}" ]] || continue
        avail="$(df --output=avail -B1M "${dir}" 2>/dev/null | tail -n 1 | tr -d ' ' || true)"
        [[ "${avail}" =~ ^[0-9]+$ ]] || continue
        [[ -n "${min_mb}" && "${min_mb}" -le "${avail}" ]] || min_mb="${avail}"
    done
    if [[ -z "${min_mb}" ]]; then
        warn "查不到 Docker 数据目录（${DOCKER_ROOT}）的剩余空间，跳过检查。构建需要约 ${MIN_FREE_GB} GB"
    elif (( min_mb < MIN_FREE_GB * 1024 )); then
        die "Docker 数据目录所在磁盘只剩 $((min_mb / 1024)) GB，构建至少需要 ${MIN_FREE_GB} GB。
    docker system df 查看占用，docker builder prune 清理构建缓存"
    else
        (( min_mb >= LOW_FREE_GB * 1024 )) || warn "Docker 数据目录所在磁盘只剩 $((min_mb / 1024)) GB：镜像约 2.6 GB，录制数据也要占空间"
        ok "磁盘剩余 $((min_mb / 1024)) GB"
    fi

    # 基础镜像：本地有就直接用（架构要对），没有先单独拉取，失败时报得清楚
    local local_base=1
    if [[ -z "${BASE_ID}" ]]; then
        local_base=0
        if [[ ${DRY_RUN} == 1 ]]; then
            info "\$ docker pull ${BASE}（--dry-run：不拉取）"
        else
            info "拉取基础镜像 ${BASE}"
            dk pull "${BASE}" || die "拉取 ${BASE} 失败。Docker 服务拉镜像不走当前 shell 的代理变量：
    给 Docker 服务配置代理或镜像加速（/etc/docker/daemon.json 的 registry-mirrors），或用 --base-image 指定一个能拉到的 Ubuntu 22.04 镜像"
            binfo="$(dk image inspect -f '{{.Id}}|{{.Architecture}}' "${BASE}" 2>/dev/null || true)"
            IFS='|' read -r BASE_ID base_arch <<<"${binfo}"
        fi
    fi
    if [[ -n "${BASE_ID}" ]]; then
        [[ "$(norm_arch "${base_arch}")" == "${ARCH}" ]] || die "基础镜像 ${BASE} 是 ${base_arch} 架构，本机是 ${ARCH}。
    docker rmi ${BASE} 后重新执行（会重新拉取本机架构的），或用 --base-image 指定"
        # Dockerfile 只适用于 Ubuntu 22.04（ROS 2 Humble 只有 22.04 的软件包）。--base-image 换成别的版本时
        # 在这里说清楚，而不是构建到一半在软件源那一步失败。--dry-run 不起容器，跳过
        local osrel bid bver
        if [[ ${DRY_RUN} == 1 ]]; then
            :
        elif osrel="$(dk run --rm --network none --entrypoint cat "${BASE}" /etc/os-release 2>/dev/null </dev/null)"; then
            bid="$(sed -n 's/^ID=//p' <<<"${osrel}" | tr -d '"')"
            bver="$(sed -n 's/^VERSION_ID=//p' <<<"${osrel}" | tr -d '"')"
            [[ "${bid}" == ubuntu && "${bver}" == 22.04 ]] || die "基础镜像 ${BASE} 是 ${bid:-未知系统}${bver:+ ${bver}}，不是 Ubuntu 22.04：镜像里装的 ROS 2 Humble 只有 Ubuntu 22.04 的软件包。
    --base-image 要指定 Ubuntu 22.04 的镜像（例如镜像站上的 ubuntu:22.04）"
        else
            warn "读不到基础镜像 ${BASE} 里的 /etc/os-release，确认不了它是 Ubuntu 22.04，继续构建"
        fi
        ok "基础镜像 ${BASE}$([[ ${local_base} == 0 ]] || printf '（本地已有）')"
    fi

    # 代理：shell 里设了就传给构建；指向本机（127.0.0.1 等）时构建容器要用主机网络才连得上
    PROXY_ARGS=(); NET_ARGS=()
    local v
    for v in http_proxy HTTP_PROXY https_proxy HTTPS_PROXY ftp_proxy FTP_PROXY all_proxy ALL_PROXY no_proxy NO_PROXY; do
        [[ -n "${!v}" ]] || continue
        PROXY_ARGS+=(--build-arg "${v}")
        [[ "${v,,}" == no_proxy ]] && continue
        [[ "${!v}" =~ (^|://|@)(localhost|127\.[0-9.]+|\[?::1\]?)(:|/|$) ]] && NET_ARGS=(--network host)
    done
    [[ ${#PROXY_ARGS[@]} == 0 ]] || info "代理：传给构建 ${PROXY_ARGS[*]//--build-arg /}${NET_ARGS[*]:+；代理在本机，构建使用主机网络}"

    # 软件源：构建前先试一遍，免得十几分钟后才在某个下载步骤失败
    local ub ros gpg pip failed=() name url
    if [[ ${MIRROR} == cn ]]; then
        ub="http://mirrors.aliyun.com/ubuntu"; ros="https://mirrors.aliyun.com/ros2/ubuntu"; pip="https://mirrors.aliyun.com/pypi/simple/"
        gpg="https://mirrors.aliyun.com/rosdistro/ros.key"   # 与官方 ros.key 逐字节相同
        [[ ${ARCH} == x86_64 ]] || ub="http://mirrors.aliyun.com/ubuntu-ports"
    else
        ub="http://archive.ubuntu.com/ubuntu"; ros="$(df_arg ROS_APT_MIRROR)"; pip="https://pypi.org/simple/"
        gpg="$(df_arg ROS_GPG_URL)"
        [[ ${ARCH} == x86_64 ]] || ub="http://ports.ubuntu.com/ubuntu-ports"
    fi
    MIRROR_ARGS=()
    [[ ${MIRROR} == official ]] || MIRROR_ARGS=(--build-arg "UBUNTU_MIRROR=http://mirrors.aliyun.com"
                                                --build-arg "ROS_APT_MIRROR=${ros}" --build-arg "ROS_GPG_URL=${gpg}"
                                                --build-arg "PIP_INDEX_URL=${pip}")
    info "软件源：${MIRROR}${MIRROR_ARGS[*]:+（阿里云镜像）}"
    for name in "Ubuntu 软件源|${ub%/}/dists/jammy/Release" "ROS 2 软件源|${ros%/}/dists/jammy/Release" \
                "ROS 2 软件源密钥|${gpg}" "PyPI|${pip%/}/pip/"; do
        url="${name#*|}"
        if reach_url "${url}"; then ok "${name%%|*}：${url}"
        elif [[ $? == 2 ]]; then info "本机没有 curl 与 wget，跳过网络检查"; failed=(); break
        else failed+=("${name%%|*}：${url}"); fi
    done
    if [[ ${#failed[@]} -gt 0 ]]; then
        warn "构建要用到的这些地址现在访问不了，构建很可能在下载那一步失败："
        printf '        %s\n' "${failed[@]}" >&2
        [[ ${MIRROR} == cn ]] || info "国内网络加 --mirror cn 改用阿里云镜像"
        info "需要代理时先 export https_proxy=… http_proxy=… 再执行本脚本"
        if [[ -t 0 && ${YES} == 0 && ${DRY_RUN} == 0 ]]; then
            local reply
            read -r -p "    仍然继续构建？[y/N] " reply || reply=""
            [[ "${reply}" == [Yy]* || "${reply}" == 是* ]] || die "已取消"
        fi
    fi
}

# ---------- 5. 构建 ----------
make_ctx() {   # 临时目录：构建上下文与检查镜像时挂进容器的目录。不用隐藏目录名（snap 版 Docker 读不了）
    local base
    for base in "${RELEASE_DIR}" "${HOME}" "${TMPDIR:-/tmp}"; do
        [[ "${base}" != *[:,]* ]] || continue     # 路径里有 : 或 , 时 docker -v 解析不了
        CTX="$(mktemp -d "${base}/revohuman-build-ctx.XXXXXX" 2>/dev/null)" && break
        CTX=""
    done
    [[ -n "${CTX}" ]] || die "建不了临时目录（${RELEASE_DIR}、${HOME}、${TMPDIR:-/tmp} 都不可写，或路径里有 : 、, ）"
}

fill_ctx() {   # 构建上下文只放这次用到的发布文件（目录里混着别的版本也不影响），同一磁盘上用硬链接
    local f
    for f in "${PYS}" "${WHEEL}" "${BUNDLE}"; do
        ln "${RELEASE_DIR}/${f}" "${CTX}/${f}" 2>/dev/null || cp "${RELEASE_DIR}/${f}" "${CTX}/${f}"
    done
    printf '%s  %s\n' "${SUM_PYS}" "${PYS}" "${SUM_WHEEL}" "${WHEEL}" "${SUM_BUNDLE}" "${BUNDLE}" >"${CTX}/SHA256SUMS"
    cp "${SCRIPT_DIR}/Dockerfile" "${CTX}/Dockerfile"
}

explain_failure() {   # 按构建日志里的特征给出处理建议
    local log="$1" hit=0
    if grep -qiE 'no space left on device' "${log}"; then
        info "磁盘满了：docker system df 查看占用，docker builder prune 清理构建缓存"; hit=1
    fi
    if grep -qiE 'is not valid yet|is in the future' "${log}"; then
        info "主机时钟不对，软件源的签名被判为尚未生效：校准系统时间后重新执行"; hit=1
    fi
    if grep -qiE 'Temporary failure resolving|Could not resolve|Failed to fetch|Connection timed out|Read timed out|ReadTimeoutError|Network is unreachable|Connection refused|Connection reset|certificate verify failed|Failed to establish a new connection|curl: \([0-9]+\)' "${log}"; then
        local tip=""
        [[ ${MIRROR} == cn ]] || tip="国内网络加 --mirror cn；"
        info "网络问题：下载软件包时失败。${tip}需要代理时先 export https_proxy=… http_proxy=… 再执行"; hit=1
    fi
    if grep -qiE 'pull access denied|failed to resolve source metadata|registry-1\.docker\.io|toomanyrequests' "${log}"; then
        info "拉不到基础镜像：给 Docker 服务配置镜像加速或代理，或用 --base-image 指定能拉到的 Ubuntu 22.04 镜像"; hit=1
    fi
    if grep -qiE 'exec format error' "${log}"; then
        info "基础镜像的架构与本机不同：docker rmi ${BASE} 后重新执行"; hit=1
    fi
    [[ ${hit} == 1 ]] || info "看上面输出的最后几行找原因"
    info "重新执行本脚本即可接着构建，已完成的步骤有缓存，不会重做"
}

step_build() {
    say 5/6 "构建镜像 ${TAG}"
    make_ctx
    [[ ${NEED_BUILD} == 1 ]] || { ok "跳过（镜像已是最新）"; return 0; }
    fill_ctx
    local args=(build "${BUILDER_ARGS[@]}" -f "${CTX}/Dockerfile" -t "${TAG}"
                --build-arg "USER_UID=$(id -u)" --build-arg "USER_GID=$(id -g)"
                --build-arg "BASE_IMAGE=${BASE}" --label "org.revohuman.inputs=${INPUTS}"
                --label "org.revohuman.base=${BASE}" --label "org.revohuman.base-id=${BASE_ID}")
    [[ ${BUILDKIT} == 0 ]] || args+=(--progress=plain --load)
    [[ ${NO_CACHE} == 0 ]] || args+=(--no-cache)
    args+=("${MIRROR_ARGS[@]}" "${PROXY_ARGS[@]}" "${NET_ARGS[@]}" "${CTX}")
    info "\$ docker $(shq "${args[@]}")"
    [[ ${DRY_RUN} == 0 ]] || { info "（--dry-run：不构建）"; return 0; }
    info "首次构建约需 10～20 分钟，主要花在下载上"
    local rc=0 started=${SECONDS}
    dk "${args[@]}" 2>&1 | tee "${CTX}/build.log" || rc=${PIPESTATUS[0]}
    if [[ ${rc} != 0 ]]; then
        printf '\n' >&2
        explain_failure "${CTX}/build.log" >&2
        die "构建失败（docker build 退出码 ${rc}）${LOG:+，完整日志：${LOG}}"
    fi
    BUILT=1
    ok "构建完成，用时 $(( (SECONDS - started) / 60 )) 分 $(( (SECONDS - started) % 60 )) 秒"
}

# ---------- 6. 检查镜像 ----------
step_verify() {
    say 6/6 "检查镜像"
    [[ ${DRY_RUN} == 0 ]] || { info "（--dry-run：跳过）"; return 0; }
    local home out check
    home="$(dk image inspect -f '{{index .Config.Labels "org.revohuman.home"}}' "${TAG}")"
    [[ -n "${home}" ]] || die "镜像 ${TAG} 缺少 org.revohuman.home 标签，不是用源码包里的 Dockerfile 构建的"
    mkdir -p "${CTX}/data"
    # 不联网运行：构建好的镜像不应再依赖下载；顺带确认容器里写出的文件属主是当前用户
    check='set -e
v="$(python3 -c "import revohuman; print(revohuman.__version__)")"
[ "$v" = "$EXPECT_VERSION" ] || { echo "镜像里的 SDK 版本是 $v，应为 $EXPECT_VERSION"; exit 1; }
echo "revohuman $v"
python3 -c "import pysurvive, mcap, bleak, mujoco"
python3 -m revohuman --help >/dev/null
command -v revohuman-live revohuman-lighthouses >/dev/null
ros2 pkg list | grep -q "^revohuman"
"$HOME/revohuman-release/revohuman-$EXPECT_VERSION/simulation/run_viewer.sh" --demo P1 --headless --seconds 2 \
    --snapshot /tmp/revohuman-check.png >/dev/null
[ -s /tmp/revohuman-check.png ]
touch "$HOME/data/uid-check"'
    out="$(dk run --rm --network none -e "EXPECT_VERSION=${VERSION}" -v "${CTX}/data:${home}/data" "${TAG}" bash -c "${check}" 2>&1)" \
        || die "镜像检查没有通过：
$(tail -n 20 <<<"${out}")
    镜像可能不完整，加 --rebuild 重新构建"
    ok "SDK、pysurvive、ROS 2 包可用（$(grep -m1 '^revohuman ' <<<"${out}" || echo "revohuman ${VERSION}")），无窗口可视化能出图，不联网也能运行"
    local owner
    owner="$(stat -c %u "${CTX}/data/uid-check" 2>/dev/null || true)"
    [[ "${owner}" == "$(id -u)" ]] || die "容器里写出的文件在主机上的属主是 ${owner:-未知}，不是当前用户 $(id -u)：Docker 的用户号映射不对（rootless / userns-remap？）"
    ok "容器里写出的文件属主是当前用户"

    # 用 run.sh 真的创建一个检查用的容器、在里面执行一条命令，再删掉它（数据与状态目录放在临时目录里）：
    # 主机上的问题（Docker 不接受某个挂载选项、设备目录挂不进去等）在这里暴露，而不是等到第一次 ./run.sh
    CHECK_NAME="revohuman-install-check-$(id -u)"
    dk rm -f "${CHECK_NAME}" >/dev/null 2>&1 </dev/null || true
    out="$(with_docker bash "${SCRIPT_DIR}/run.sh" --image "${TAG}" --name "${CHECK_NAME}" \
               --data "${CTX}/run/data" --state "${CTX}/run/state" true 2>&1 </dev/null)" \
        || die "镜像本身的检查都通过了，但 run.sh 用它创建容器失败，多半是 Docker 或主机设置的问题。run.sh 的输出：
$(tail -n 10 <<<"${out}")"
    if dk rm -f "${CHECK_NAME}" >/dev/null 2>&1 </dev/null; then
        ok "run.sh 能用这个镜像创建容器、在容器里执行命令（检查用的容器已删除）"
    else
        warn "检查用的容器 ${CHECK_NAME} 没能删除，手动执行 docker rm -f ${CHECK_NAME}"
    fi
    CHECK_NAME=""

    # 用旧镜像创建的容器不会自动换成新镜像（run.sh 只比较镜像名），提醒重建
    local new_id names n img cimg
    new_id="$(dk image inspect -f '{{.Id}}' "${TAG}")"
    mapfile -t names < <(dk ps -a --filter label=org.revohuman.run-config --format '{{.Names}}' 2>/dev/null || true)
    for n in "${names[@]}"; do
        [[ -n "${n}" ]] || continue
        IFS='|' read -r img cimg <<<"$(dk container inspect -f '{{.Image}}|{{.Config.Image}}' "${n}" 2>/dev/null </dev/null || true)"
        [[ "${cimg}" == "${TAG}" && "${img}" != "${new_id}" ]] || continue
        warn "容器 ${n} 还在用构建之前的旧镜像：容器里没有正在运行的程序时执行 ./run.sh --restart$([[ ${n} == revohuman ]] || printf ' --name %s' "${n}") 换成新镜像"
    done
}

summary() {
    local id size
    if [[ ${DRY_RUN} == 1 ]]; then
        say 完成 "--dry-run 结束，没有改动主机，也没有构建"
        print_warnings
        return 0
    fi
    say 完成 "镜像 ${TAG} 已就绪"
    id="$(dk image inspect -f '{{.Id}}' "${TAG}" | sed 's/^sha256://' | cut -c1-12)"
    # 占用的磁盘空间。不用 image inspect 的 .Size：containerd 镜像存储下它是压缩后的大小，只有实际的四分之一左右
    size="$(dk image ls --format '{{.Size}}' "${TAG}" 2>/dev/null | head -n 1 || true)"
    info "镜像 ID ${id}${size:+，占用磁盘 ${size}}"
    [[ -z "${LOG}" ]] || info "日志：${LOG}"
    echo
    echo "下一步："
    local run="./run.sh"
    [[ "${TAG}" == "revohuman:${VERSION}" ]] || run+=" --image $(shq "${TAG}")"
    if [[ ${NEED_RELOGIN} == 1 ]]; then
        info "注销并重新登录一次，docker 组才对新开的终端生效；在那之前用 sg ${USE_SG} -c '${run}' 代替 ${run}"
    fi
    [[ ${UDEV_INSTALLED} != 1 ]] || info "已经插着的手套单元重新插拔一次，让 Tracker 设备规则生效"
    info "cd $(shq "${SCRIPT_DIR}") && ${run}"
    info "进入容器后按手册《02-硬件连接》第 3 节体检；完整说明见《08-容器运行》"
    print_warnings
}

preflight
step_files
step_docker
step_udev
step_plan
step_build
step_verify
summary
