#!/usr/bin/env bash
# One-step build of the RevoHuman Docker image: checks and prepares the host (Docker, the docker group,
# Tracker device rules), verifies the release files, builds the image, then runs checks inside the image.
# After building, use run.sh in this directory to enter the container.
#
#   ./install.sh                  build image revohuman:0.1.4 from the official package sources
#   ./install.sh --mirror cn      networks in mainland China: use Aliyun mirrors for Ubuntu, ROS 2 and PyPI
#
# Steps that need sudo (installing Docker, adding the user to the docker group, installing the Tracker
# device rules) ask for confirmation one by one before running.
# If the image was already built from the same release files for the current user, it is not rebuilt and
# only the checks are re-run, so it is safe to run repeatedly.
# Full details: user manual *08 Running in Containers*.

# When run with sh, hand over to bash (bash syntax is used below); sourcing into the current shell is
# not allowed, since an error would then exit that shell as well
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

VERSION="0.1.4"
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
# For tests only: relocate the host system files read and written (under /etc, /lib, /usr/lib, /run, /proc,
# /sys) to another root directory
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
# Check the image name by Docker's rules ([host[:port]/]name[:tag]): each name component consists of
# lowercase letters and digits, separated within the component only by a single . or _, two _, or a run
# of -, and must not start or end with a separator; the tag is at most 128 characters
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

# ---------- Output and prompts ----------
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

# Quote arguments by POSIX sh rules: sg interprets the command string with /bin/sh; also used for
# commands printed for the user to copy
shq() {
    local a out=""
    for a in "$@"; do
        if [[ "${a}" =~ ^[A-Za-z0-9_./:=@%+,-]+$ ]]; then out+="${a} "
        else out+="'${a//\'/\'\\\'\'}' "; fi
    done
    printf '%s' "${out% }"
}

# confirm QUESTION: returns 0 on yes. --yes agrees directly; a non-interactive run without --yes declines
# (the caller then prints the command to run manually)
confirm() {
    if [[ ${DRY_RUN} == 1 ]]; then info "$1（--dry-run：只打印命令）"; return 0; fi
    if [[ ${YES} == 1 ]]; then info "$1（--yes：直接执行）"; return 0; fi
    if [[ ! -t 0 ]]; then info "$1（非交互运行，也没有给 --yes：跳过）"; return 1; fi
    local reply
    read -r -p "    $1 [Y/n] " reply || return 1
    [[ -z "${reply}" || "${reply}" == [Yy]* || "${reply}" == 是* ]]
}

# Run a command with sudo, printing it first; as_root exits on failure, try_root only returns the result.
# Prints to stderr: the caller may discard the command's own output (e.g. as_root tee FILE >/dev/null)
try_root() {
    info "\$ sudo $(shq "$@")" >&2
    [[ ${DRY_RUN} == 0 ]] || return 0
    command -v sudo >/dev/null || { warn "这一步需要 sudo，本机没有 sudo 命令"; return 1; }
    sudo "$@"
}
as_root() { try_root "$@" || die "命令没有成功：sudo $(shq "$@")（没有 sudo 权限时请让管理员执行）"; }

# When the docker group is not yet in effect in the current login session, run docker and run.sh
# as that group via sg
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

# ---------- 0. Prerequisites ----------
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

    # Only one install.sh per user at a time (concurrent builds would overwrite each other's image name and
    # contend for temporary directories); --dry-run changes nothing and does not need to wait
    local lock="${XDG_RUNTIME_DIR:-/tmp}/revohuman-install-$(id -u).lock"
    if [[ ${DRY_RUN} == 0 ]] && command -v flock >/dev/null && { : >>"${lock}"; } 2>/dev/null; then
        exec 9>>"${lock}"
        flock -n 9 || die "另一个 install.sh 正在运行（锁文件 ${lock}），等它结束后再执行"
    fi

    # Save a copy of all output to a log; send it to technical support if something goes wrong
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

# ---------- 1. Release files ----------
sum_of() { awk -v f="$1" '$2 == f { print $1; exit }' <<<"${SUMS_TEXT}"; }

extras_text() {   # files of the same kind but another version in the directory (e.g. an older
                  # wheel), which cannot substitute, and unfinished downloads: listed separately
                  # to avoid confusion
    local f others=() partial=()
    for f in "${RELEASE_DIR}"/revohuman-*-linux_"${ARCH}".whl "${RELEASE_DIR}"/revohuman-*-bundle.tar.gz \
             "${RELEASE_DIR}"/pysurvive-*-linux_"${ARCH}".whl; do
        [[ -f "${f}" ]] || continue
        f="$(basename "${f}")"
        [[ "${f}" == "${WHEEL}" || "${f}" == "${BUNDLE}" ]] && continue
        if [[ -n "${PYS}" ]]; then [[ "${f}" == "${PYS}" ]] && continue
        else [[ "${f}" == pysurvive-* ]] && continue; fi    # if SHA256SUMS is wrong, we cannot tell
                                                            # whether pysurvive belongs to this version
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
    # Accept Windows line endings and the sha256sum binary-mode marker (the * before the file name)
    SUMS_TEXT="$(tr -d '\r' <"${RELEASE_DIR}/SHA256SUMS" | awk 'NF >= 2 { f = $2; sub(/^\*/, "", f); print $1, f }')"

    if [[ -z "$(sum_of "${BUNDLE}")" ]]; then
        # SHA256SUMS belongs to another version (common when a later version's source bundle was unpacked
        # in an older version's download directory): list all missing files at once
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
    # If Docker's official package source is configured, install the official packages; otherwise install
    # the distribution's docker.io (the two cannot be mixed)
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

# Connect to Docker: start the service if it is not running; if permission is missing, add the docker group
# (continue via sg this time, so the user does not have to log out first)
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
        # Cannot connect and it is not a permission problem: most likely the service is not running
        if [[ ${attempt} == 1 && ${SKIP_HOST} == 0 && ( "${sock}" == /var/run/docker.sock || "${sock}" == /run/docker.sock ) ]] \
            && command -v systemctl >/dev/null; then
            confirm "Docker 服务没有运行，启动它并设为开机自启（sudo systemctl enable --now docker）？" \
                || die "Docker 服务没有运行。手动执行 sudo systemctl enable --now docker 后再运行本脚本"
            as_root systemctl enable --now docker
            [[ ${DRY_RUN} == 0 ]] || dry_stop
            # Wait for the service to start; stop waiting once connected or once it becomes a permission
            # problem (handled in the next round)
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
    # docker must connect to the local service (when DOCKER_HOST or the docker context points to another
    # machine, gloves and Trackers cannot be passed in)
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
    # With Docker 25 and later, run.sh mounts the host's /dev into the container, so replugging gloves takes
    # effect immediately; earlier versions can only mount, one by one, the device nodes present at the time
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
        # If the current builder uses a driver such as docker-container (created by docker buildx create
        # --use), the build does not run in the local Docker: local base images cannot be used, and
        # --network host needs extra authorization. Use the builder built into the local Docker instead
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

# ---------- 3. Tracker device rules ----------
# Whether the rules file has a line (not counting comments) that makes the 28de:2300 USB device node
# readable and writable by all users.
# Rules that only cover hidraw, only add uaccess (effective only for users logged in graphically on this
# machine), or only name other product IDs do not count
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
    # Directories in udev priority order: of files with the same name, only the one listed first takes effect
    # (an empty file of the same name under /etc, or a link to /dev/null, masks the rule under /lib)
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
        # Rule installed but an already plugged-in Tracker still lacks permission: most likely it was not
        # replugged after the rule was installed
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

# ---------- 4. Pre-build preparation ----------
reach_url() {   # whether the URL is reachable within a time limit: 0 yes, 1 no, 2 neither curl
                # nor wget on this machine. Uses the current shell's proxy variables, like the build
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
    # Image labels record the inputs the image was built from: the Dockerfile, the three release files and
    # the user ID (org.revohuman.inputs); the base image name and ID are recorded separately
    # (org.revohuman.base, org.revohuman.base-id). Package sources are not included: a different source
    # installs the same thing
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
            # The suggested image name must be usable as-is: convert the user name to lowercase letters,
            # digits and single -, strip leading and trailing -, and use the user ID if the result is empty
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
            # Same base image ID (the same image under another name, e.g. --base-image given as the full
            # name of ubuntu:22.04, or the same image on a mirror site), or the same name: either counts as
            # the same base image. Same name with a changed ID means it was pulled again after the build:
            # only warn, do not rebuild automatically (a rebuild takes over ten minutes, and the packages in
            # the image are downloaded from the package sources during the build anyway); a deleted local
            # base image does not require a rebuild either
            NEED_BUILD=0
            ok "镜像 ${TAG} 已经用这批发布文件、为当前用户构建过，跳过构建（要重新构建加 --rebuild）"
            [[ -z "${BASE_ID}" || -z "${iid}" || "${iid}" == "${BASE_ID}" ]] \
                || warn "基础镜像 ${BASE} 在镜像 ${TAG} 构建之后更新过（重新拉取过）；要基于新的基础镜像构建，加 --rebuild"
            return 0
        else
            info "镜像 ${TAG} 是基于另一个基础镜像（${ib:-未知}）构建的；基于 ${BASE} 重新构建并替换它"
        fi
    fi

    # Disk: image data may be in the Docker data directory or in containerd's directory
    # (containerd image store)
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

    # Base image: use the local one if present (the architecture must match); otherwise pull it separately
    # first, so that a failure is reported clearly
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
        # The Dockerfile only works with Ubuntu 22.04 (ROS 2 Humble only has packages for 22.04). When
        # --base-image is another version, say so clearly here instead of failing halfway through the build
        # at the package source step. --dry-run does not start containers: skip
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

    # Proxy: if set in the shell, pass it to the build; if it points to this machine (127.0.0.1 etc.),
    # the build container needs host networking to reach it
    PROXY_ARGS=(); NET_ARGS=()
    local v
    for v in http_proxy HTTP_PROXY https_proxy HTTPS_PROXY ftp_proxy FTP_PROXY all_proxy ALL_PROXY no_proxy NO_PROXY; do
        [[ -n "${!v}" ]] || continue
        PROXY_ARGS+=(--build-arg "${v}")
        [[ "${v,,}" == no_proxy ]] && continue
        [[ "${!v}" =~ (^|://|@)(localhost|127\.[0-9.]+|\[?::1\]?)(:|/|$) ]] && NET_ARGS=(--network host)
    done
    [[ ${#PROXY_ARGS[@]} == 0 ]] || info "代理：传给构建 ${PROXY_ARGS[*]//--build-arg /}${NET_ARGS[*]:+；代理在本机，构建使用主机网络}"

    # Package sources: try them before building, rather than failing at some download step over ten
    # minutes in
    local ub ros gpg pip failed=() name url
    if [[ ${MIRROR} == cn ]]; then
        ub="http://mirrors.aliyun.com/ubuntu"; ros="https://mirrors.aliyun.com/ros2/ubuntu"; pip="https://mirrors.aliyun.com/pypi/simple/"
        gpg="https://mirrors.aliyun.com/rosdistro/ros.key"   # byte-identical to the official ros.key
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

# ---------- 5. Build ----------
make_ctx() {   # temporary directory: the build context, and the directory mounted into the
               # container when checking the image. No hidden directory name (snap Docker cannot read it)
    local base
    for base in "${RELEASE_DIR}" "${HOME}" "${TMPDIR:-/tmp}"; do
        [[ "${base}" != *[:,]* ]] || continue     # docker -v cannot parse paths containing : or ,
        CTX="$(mktemp -d "${base}/revohuman-build-ctx.XXXXXX" 2>/dev/null)" && break
        CTX=""
    done
    [[ -n "${CTX}" ]] || die "建不了临时目录（${RELEASE_DIR}、${HOME}、${TMPDIR:-/tmp} 都不可写，或路径里有 : 、, ）"
}

fill_ctx() {   # the build context holds only the release files used this time (other
               # versions in the directory do not matter); hard links on the same disk
    local f
    for f in "${PYS}" "${WHEEL}" "${BUNDLE}"; do
        ln "${RELEASE_DIR}/${f}" "${CTX}/${f}" 2>/dev/null || cp "${RELEASE_DIR}/${f}" "${CTX}/${f}"
    done
    printf '%s  %s\n' "${SUM_PYS}" "${PYS}" "${SUM_WHEEL}" "${WHEEL}" "${SUM_BUNDLE}" "${BUNDLE}" >"${CTX}/SHA256SUMS"
    cp "${SCRIPT_DIR}/Dockerfile" "${CTX}/Dockerfile"
}

explain_failure() {   # give advice based on patterns in the build log
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

# ---------- 6. Check the image ----------
# Whether image $1 is an earlier version of the ${TAG} built this time: same image name, numeric versions,
# and lower (the same rule run.sh uses to decide whether it can switch images automatically)
older_than_tag() {
    local cv nv
    [[ "$1" == *:* && "${1%:*}" == "${TAG%:*}" ]] || return 1
    cv="${1##*:}"; nv="${TAG##*:}"
    [[ "${cv}" =~ ^[0-9]+(\.[0-9]+)*$ && "${nv}" =~ ^[0-9]+(\.[0-9]+)*$ && "${cv}" != "${nv}" ]] || return 1
    [[ "$(printf '%s\n' "${cv}" "${nv}" | sort -V | head -n 1)" == "${cv}" ]]
}

step_verify() {
    say 6/6 "检查镜像"
    [[ ${DRY_RUN} == 0 ]] || { info "（--dry-run：跳过）"; return 0; }
    local home out check
    home="$(dk image inspect -f '{{index .Config.Labels "org.revohuman.home"}}' "${TAG}")"
    [[ -n "${home}" ]] || die "镜像 ${TAG} 缺少 org.revohuman.home 标签，不是用源码包里的 Dockerfile 构建的"
    mkdir -p "${CTX}/data"
    # Run without network: the built image should no longer depend on downloads; also confirm that files
    # written in the container are owned by the current user
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

    # Actually create a check container with run.sh, run one command in it, then delete it (data and state
    # directories in the temporary directory): host problems (Docker rejecting a mount option, a device
    # directory that cannot be mounted, etc.) show up here instead of on the first ./run.sh
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

    # Existing containers: when the image of the same name was rebuilt, or the container uses an earlier
    # version under the same image name (e.g. revohuman:0.1.3 before an upgrade), run.sh switches to the
    # updated image on its next entry (if programs are still running in the container, it notifies first
    # and does not interrupt them); here we only inform. For other versions (downgrade, no version number)
    # run.sh does not switch automatically: remind the user to recreate the container
    local new_id names n img cimg cmd
    new_id="$(dk image inspect -f '{{.Id}}' "${TAG}")"
    mapfile -t names < <(dk ps -a --filter label=org.revohuman.run-config --format '{{.Names}}' 2>/dev/null </dev/null || true)
    for n in "${names[@]}"; do
        [[ -n "${n}" ]] || continue
        IFS='|' read -r img cimg <<<"$(dk container inspect -f '{{.Image}}|{{.Config.Image}}' "${n}" 2>/dev/null </dev/null || true)"
        [[ -n "${img}" && "${img}" != "${new_id}" ]] || continue
        cmd="./run.sh$([[ ${n} == revohuman ]] || printf ' --name %s' "${n}")$([[ ${TAG} == "revohuman:${VERSION}" ]] || printf ' --image %s' "${TAG}")"
        if [[ "${cimg}" == "${TAG}" ]]; then
            info "容器 ${n} 还在用构建之前的旧镜像：下次执行 ${cmd} 时自动换成新镜像，创建容器时给的 --data、--ble、-e 等设置照旧（容器里还有程序在运行时先提示，不打断它们）"
        elif older_than_tag "${cimg}"; then
            info "容器 ${n} 用的是更早版本的镜像 ${cimg}：下次执行 ${cmd} 时自动换成本次构建的 ${TAG}，创建容器时给的 --data、--ble、-e 等设置照旧（容器里还有程序在运行时先提示，不打断它们）"
        elif [[ "${cimg%:*}" == "${TAG%:*}" ]]; then
            cmd="./run.sh --restart${cmd#./run.sh}"
            warn "容器 ${n} 用的是另一个版本的镜像 ${cimg}，不是本次构建的 ${TAG}，run.sh 不会自动换：容器里没有正在运行的程序时执行 ${cmd} 换成新镜像（创建容器时加过 --data、--ble、-e 等选项的要一并写上）"
        fi
    done

    # Replaced old images (several GB each) are not deleted automatically: list the old images built by the
    # current user that no container uses any more, and tell the user how to delete them
    local iid ref size refs=() rows=()
    while IFS='|' read -r iid ref size; do
        [[ -n "${iid}" && "${new_id#sha256:}" != "${iid#sha256:}"* ]] || continue
        [[ -z "$(dk ps -aq --filter "ancestor=${iid}" 2>/dev/null </dev/null || echo '?')" ]] || continue
        [[ "${ref}" != *"<none>"* ]] || ref="${iid#sha256:}"
        refs+=("${ref}"); rows+=("${ref}（${size}）")
    done < <(dk image ls --filter "label=org.revohuman.uid=$(id -u)" --format '{{.ID}}|{{.Repository}}:{{.Tag}}|{{.Size}}' 2>/dev/null </dev/null || true)
    [[ ${#refs[@]} == 0 ]] \
        || info "这些旧镜像已经没有容器在用：$(printf '%s、' "${rows[@]}" | sed 's/、$//')。确认新版本没问题之后，可以执行 docker image rm ${refs[*]} 删掉它们，腾出磁盘空间"
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
    # Disk space used. Not image inspect's .Size: with the containerd image store that is the compressed
    # size, only about a quarter of the actual size
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
