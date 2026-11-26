#!/usr/bin/env bash
# Run RevoHuman in a container. The container stays up in the background, and every run of this script
# enters the same container, so revohuman-live, visualization, ros2 and other commands can run in
# separate terminals.
#
#   ./run.sh                        enter a shell in the container (starts it first if it is not running)
#   ./run.sh <command> [args…]      run one command in the container, e.g.
#                                   ./run.sh python3 -m revohuman doctor
#   ./run.sh --restart [command…]   recreate the container with this run's options, then enter it
#                                   (use after replugging gloves or changing options)
#   ./run.sh --stop                 stop and remove the container (data and calibration are in host
#                                   directories and are not affected)
#
# Programs in the container and on the host share the same device locks, shared memory and live broadcast
# socket, with the same effect as running directly on the host: a given glove or set of Trackers can be
# used by only one program at a time, whether that program is in the container or on the host.
#
# After upgrading to a later version (or rebuilding the image), when no program is running in the
# container, ./run.sh recreates the container from the updated image automatically, keeping the --data,
# --state, --ble, -e given when the container was created; if programs are still running in the
# container, it only notifies and does not interrupt them.
# Full details: user manual *08 Running in Containers*.

set -eo pipefail

IMAGE="revohuman:0.1.4"
DEFAULT_IMAGE="${IMAGE}"
NAME="revohuman"
DATA_DIR="${HOME}/revohuman-data"
STATE_DIR="${XDG_DATA_HOME:-${HOME}/.local/share}/revohuman-docker"
GUI=1
BLE=0
ACTION=run
DRY_RUN=0
EXTRA_ENV=()
SET_DATA=0
SET_STATE=0

usage() {
    cat <<'EOF'
用法：run.sh [选项] [--] [命令 [参数…]]

选项（只在创建容器时生效；容器已在运行时要加 --restart。不加 --restart 时，没给出的
--data、--state、--ble、-e 沿用现有容器的设置，容器停了或要换成新镜像时也按它们重建）：
  --image 镜像        使用的镜像（默认 revohuman:0.1.4）
  --name 名字         容器名（默认 revohuman）
  --data 目录         挂到容器 ~/data 的目录（默认 ~/revohuman-data），录制数据写在这里
  --state 目录        标定与运行状态的存放目录（默认 ~/.local/share/revohuman-docker）
  --ble               允许容器使用宿主蓝牙（读取基站信道）
  --no-gui            不接宿主的图形界面
  -e, --env 名=值     给容器加环境变量，可重复

操作：
  --restart           删除正在运行的容器，完全按这次给的选项重新创建后再进入
  --stop              停止并删除容器
  --dry-run           只打印要执行的 docker 命令，不执行
  -h, --help          显示本帮助
EOF
}

die() { echo "run.sh: $*" >&2; exit 1; }
warn() { echo "run.sh: 注意：$*" >&2; }

while [[ $# -gt 0 ]]; do
    case "$1" in
        --image) IMAGE="${2:?--image 需要参数}"; shift 2 ;;
        --name) NAME="${2:?--name 需要参数}"; shift 2 ;;
        --data) DATA_DIR="${2:?--data 需要参数}"; SET_DATA=1; shift 2 ;;
        --state) STATE_DIR="${2:?--state 需要参数}"; SET_STATE=1; shift 2 ;;
        --ble) BLE=1; shift ;;
        --no-gui) GUI=0; shift ;;
        -e|--env) EXTRA_ENV+=("${2:?$1 需要参数}"); shift 2 ;;
        --restart) ACTION=restart; shift ;;
        --stop) ACTION=stop; shift ;;
        --dry-run) DRY_RUN=1; shift ;;
        -h|--help) usage; exit 0 ;;
        --) shift; break ;;
        -*) die "不认识的选项 $1（用 --help 查看用法；要传给容器里命令的参数放在命令名之后）" ;;
        *) break ;;
    esac
done
[[ $# -gt 0 ]] || set -- bash
DATA_DIR="$(realpath -m "${DATA_DIR}")"
STATE_DIR="$(realpath -m "${STATE_DIR}")"
exec 3>&1

run() {
    if [[ ${DRY_RUN} == 1 ]]; then
        printf '%q ' "$@" >&3; echo >&3
    else
        "$@"
    fi
}

command -v docker >/dev/null || die "找不到 docker 命令，请先安装 Docker"
docker info >/dev/null 2>&1 || die "无法连接 Docker：确认 Docker 服务已启动，并且当前用户在 docker 组里（加组后要重新登录）"

running() { [[ "$(docker container inspect -f '{{.State.Running}}' "${NAME}" 2>/dev/null)" == true ]]; }
exists() { docker container inspect "${NAME}" >/dev/null 2>&1; }

if [[ ${ACTION} == stop ]]; then
    if exists; then run docker rm -f "${NAME}" >/dev/null; echo "已停止并删除容器 ${NAME}"; else echo "容器 ${NAME} 没有在运行"; fi
    exit 0
fi

# ---------- Check image and user ----------
[[ "$(id -u)" != 0 ]] || die "不要用 root 运行本脚本：把当前用户加进 docker 组后以普通用户运行"
docker image inspect "${IMAGE}" >/dev/null 2>&1 \
    || die "找不到镜像 ${IMAGE}：先执行同目录的 ./install.sh 构建$([[ ${IMAGE} == revohuman:0.1.4 ]] || printf '（加 --tag %s）' "${IMAGE}")，说明见手册《08-容器运行》"
label() { docker image inspect -f "{{index .Config.Labels \"$1\"}}" "${IMAGE}"; }
IMG_UID="$(label org.revohuman.uid)"; IMG_GID="$(label org.revohuman.gid)"; CHOME="$(label org.revohuman.home)"
[[ -n "${IMG_UID}" && -n "${CHOME}" ]] || die "镜像 ${IMAGE} 不是用源码包里的 docker/Dockerfile 构建的"
if [[ "${IMG_UID}" != "$(id -u)" || "${IMG_GID}" != "$(id -g)" ]]; then
    die "镜像里的用户是 ${IMG_UID}:${IMG_GID}，当前用户是 $(id -u):$(id -g)。两者必须一致（挂载目录的属主与 Tracker 锁都按用户号判断），
    执行同目录的 ./install.sh 为当前用户构建一份（镜像是本机别的用户在用时加 --tag 换个名字，再用 --image 指定）"
fi

# ---------- Assemble the container creation arguments ----------
create_args() {
    ARGS=(--init --network host
          -v /dev/shm:/dev/shm
          -v "${DATA_DIR}:${CHOME}/data"
          -v "${STATE_DIR}/config/libsurvive:${CHOME}/.config/libsurvive"
          -v "${STATE_DIR}/config/revohuman:${CHOME}/.config/revohuman"
          -v "${STATE_DIR}/state/revohuman:${CHOME}/.local/state/revohuman"
          -v "${STATE_DIR}/ros:${CHOME}/.ros")

    # Time zone matches the host (otherwise times and logs in the container are off by several hours)
    local tz=""
    [[ -r /etc/timezone ]] && tz="$(head -n 1 /etc/timezone)"
    [[ -n "${tz}" ]] || tz="$(readlink /etc/localtime 2>/dev/null | sed -n 's#.*/zoneinfo/##p')"
    [[ -z "${tz}" ]] || ARGS+=(-e "TZ=${tz}")

    # ROS 2 domain settings match the host
    local v
    for v in ROS_DOMAIN_ID ROS_LOCALHOST_ONLY RMW_IMPLEMENTATION; do
        [[ -z "${!v}" ]] || ARGS+=(-e "${v}=${!v}")
    done
    for v in "${EXTRA_ENV[@]}"; do ARGS+=(-e "${v}"); done

    # The live broadcast socket uses the same path as on the host (same rule as the SDK's default path):
    # visualization on the host can connect to revohuman-live in the container, and vice versa
    ARGS+=(-e "REVOHUMAN_LIVE_SOCKET=${LIVE_DIR}/live.sock" -v "${LIVE_DIR}:${LIVE_DIR}")

    # Glove serial ports. The container must open the same device node as the host, so that the in-use check
    # works inside and outside the container at once.
    # Docker 25 and later: mount the host's /dev read-only at /run/host/dev (non-recursive; submounts such as
    # /dev/pts are not included); the SDK looks for gloves there via REVOHUMAN_DEV_DIR. Gloves replugged after
    # the container starts show up as device nodes directly, without recreating the container; which devices
    # can be opened is decided only by the cgroup rules below (only glove serial ports, video devices and USB
    # devices are allowed).
    # Earlier Docker cannot mount non-recursively: mount each glove node present at container creation
    # individually, and --restart after replugging.
    local tty dev sys groups=()
    GLOVE_TTYS=()
    for sys in /sys/class/tty/ttyACM*; do
        [[ -e "${sys}" ]] || continue
        dev="$(readlink -f "${sys}/device")"
        while [[ "${dev}" == /sys/* && ! -e "${dev}/idVendor" ]]; do dev="$(dirname "${dev}")"; done
        [[ "$(cat "${dev}/idVendor" 2>/dev/null):$(cat "${dev}/idProduct" 2>/dev/null)" == 0483:5710 ]] || continue
        tty="/dev/$(basename "${sys}")"
        [[ -c "${tty}" ]] || continue
        GLOVE_TTYS+=("${tty}")
        groups+=("$(stat -c %g "${tty}")")
        [[ ${DEV_HOTPLUG} == 1 ]] || ARGS+=(-v "${tty}:${tty}")
    done
    if [[ ${DEV_HOTPLUG} == 1 ]]; then
        ARGS+=(--mount "type=bind,src=/dev,dst=${HOST_DEV},readonly,bind-recursive=disabled"
               -e "REVOHUMAN_DEV_DIR=${HOST_DEV}")
        # Gloves plugged in later must also be openable: add the serial port group (usually dialout)
        # whether or not a glove is plugged in right now
        local gid
        gid="$(getent group dialout | cut -d: -f3)"
        [[ -z "${gid}" ]] || groups+=("${gid}")
    fi
    ARGS+=(--device-cgroup-rule 'c 166:* rmw')

    # Wrist cameras (V4L2): same approach as glove serial ports. The SDK identifies the glove unit's camera
    # via sysfs and opens it under the same device directory
    VIDEO_NODES=()
    for dev in /dev/video*; do
        [[ -c "${dev}" ]] || continue
        VIDEO_NODES+=("${dev}")
        groups+=("$(stat -c %g "${dev}")")
        [[ ${DEV_HOTPLUG} == 1 ]] || ARGS+=(-v "${dev}:${dev}")
    done
    if [[ ${DEV_HOTPLUG} == 1 ]]; then
        local vgid
        vgid="$(getent group video | cut -d: -f3)"
        [[ -z "${vgid}" ]] || groups+=("${vgid}")
    fi
    ARGS+=(--device-cgroup-rule 'c 81:* rmw')

    # Trackers (accessed by libsurvive via libusb): mount the whole USB device directory, so they are still
    # found when their numbers change after re-enumeration
    ARGS+=(-v /dev/bus/usb:/dev/bus/usb --device-cgroup-rule 'c 189:* rmw')
    [[ -d /run/udev ]] && ARGS+=(-v /run/udev:/run/udev:ro)

    # GUI: X11 socket + GPU (without GPU permission, visualization silently falls back to software rendering
    # with a very low frame rate)
    if [[ ${GUI} == 1 ]]; then
        [[ -d /tmp/.X11-unix ]] && ARGS+=(-v /tmp/.X11-unix:/tmp/.X11-unix:ro)
        ARGS+=(-v "${STATE_DIR}/x11:/run/revohuman-x11:ro" -e XAUTHORITY=/run/revohuman-x11/Xauthority)
        if [[ -d /dev/dri ]]; then
            ARGS+=(--device /dev/dri)
            for dev in /dev/dri/*; do [[ -c "${dev}" ]] && groups+=("$(stat -c %g "${dev}")"); done
        fi
    fi

    # Bluetooth: use BlueZ via the host's D-Bus
    if [[ ${BLE} == 1 ]]; then
        [[ -S /run/dbus/system_bus_socket ]] || die "--ble：宿主上没有 /run/dbus/system_bus_socket"
        ARGS+=(-v /run/dbus/system_bus_socket:/run/dbus/system_bus_socket --security-opt apparmor=unconfined)
    fi

    local g
    for g in $(printf '%s\n' "${groups[@]}" | sort -un); do ARGS+=(--group-add "${g}"); done
}

LIVE_DIR="${XDG_RUNTIME_DIR:-/tmp/revohuman-$(id -u)}"
[[ "${LIVE_DIR}" == "${XDG_RUNTIME_DIR}" ]] && LIVE_DIR="${LIVE_DIR}/revohuman"
HOST_DEV=/run/host/dev
DOCKER_MAJOR="$(docker version -f '{{.Server.Version}}' 2>/dev/null | cut -d. -f1)"
if [[ "${DOCKER_MAJOR}" =~ ^[0-9]+$ ]] && (( DOCKER_MAJOR >= 25 )); then DEV_HOTPLUG=1; else DEV_HOTPLUG=0; fi

# ---------- Existing container: keep its settings; if from an older image, decide whether to switch ---------
# Without --restart, any of --data, --state, --ble, -e not given on the command line follows the existing
# container (read from its mounts and environment variables): when the container is stopped (e.g. after
# a reboot) or is recreated to switch to the current image, options given at creation are not silently
# dropped; while the container is running, only options given explicitly this time that differ from
# creation count as "different settings".
# With --restart, this run's command line is used exactly, e.g. recreating without --ble returns to a
# container without Bluetooth
DEFAULT_DATA="$(realpath -m "${HOME}/revohuman-data")"
DEFAULT_STATE="$(realpath -m "${XDG_DATA_HOME:-${HOME}/.local/share}/revohuman-docker")"
INHERITED=()
inherit_settings() {
    local home="${CUR_HOME:-${CHOME}}" dst src line key e img_env
    while IFS='|' read -r dst src; do
        case "${dst}" in
            "${home}/data")
                [[ ${SET_DATA} == 1 || -z "${src}" ]] || DATA_DIR="$(realpath -m "${src}")" ;;
            "${home}/.config/revohuman")
                [[ ${SET_STATE} == 1 || "${src}" != */config/revohuman ]] || STATE_DIR="$(realpath -m "${src%/config/revohuman}")" ;;
            /run/dbus/system_bus_socket)
                [[ ${BLE} == 1 ]] || { BLE=1; INHERITED+=(--ble); } ;;
        esac
    done < <(docker container inspect -f '{{range .Mounts}}{{.Destination}}|{{.Source}}{{println}}{{end}}' "${NAME}" 2>/dev/null || true)
    [[ ${SET_DATA} == 1 || "${DATA_DIR}" == "${DEFAULT_DATA}" ]] || INHERITED+=(--data "${DATA_DIR}")
    [[ ${SET_STATE} == 1 || "${STATE_DIR}" == "${DEFAULT_STATE}" ]] || INHERITED+=(--state "${STATE_DIR}")

    # Variables added with -e = the container's environment variables, minus those its image already has and
    # minus the ones run.sh sets from the host.
    # A variable given again this time takes this run's value; keep the container's original order, so the
    # settings digest does not change when the settings have not changed
    [[ -n "${CUR_ID}" ]] || return 0
    if ! img_env="$(docker image inspect -f '{{range .Config.Env}}{{println .}}{{end}}' "${CUR_ID}" 2>/dev/null)"; then
        warn "读不到容器 ${NAME} 原来的镜像，没法沿用创建容器时用 -e 加的环境变量：需要的话这次用 -e 重新给出"
        return 0
    fi
    local -A given=()
    local merged=()
    for e in "${EXTRA_ENV[@]}"; do given["${e%%=*}"]="${e}"; done
    while IFS= read -r line; do
        key="${line%%=*}"
        case "${key}" in
            ""|TZ|ROS_DOMAIN_ID|ROS_LOCALHOST_ONLY|RMW_IMPLEMENTATION|REVOHUMAN_LIVE_SOCKET|REVOHUMAN_DEV_DIR|XAUTHORITY) continue ;;
        esac
        if [[ -n "${given[${key}]+x}" ]]; then
            merged+=("${given[${key}]}"); unset "given[${key}]"; continue
        fi
        grep -qxF -- "${line}" <<<"${img_env}" && continue
        merged+=("${line}"); INHERITED+=(-e "${line}")
    done < <(docker container inspect -f '{{range .Config.Env}}{{println .}}{{end}}' "${NAME}" 2>/dev/null || true)
    for e in "${EXTRA_ENV[@]}"; do [[ -z "${given[${e%%=*}]+x}" ]] || merged+=("${e}"); done
    EXTRA_ENV=("${merged[@]}")
}

# The existing container's image is older than the one to use now: the image of the same name was
# rebuilt, or the version under the same image name is lower (e.g. revohuman:0.1.3 → 0.1.4).
# A downgrade or a switch to another image name does not switch automatically; only a notice
older_image() {
    local cv nv
    [[ "$1" == "${IMAGE}" ]] && return 0
    [[ "$1" == *:* && "${1%:*}" == "${IMAGE%:*}" ]] || return 1
    cv="${1##*:}"; nv="${IMAGE##*:}"
    [[ "${cv}" =~ ^[0-9]+(\.[0-9]+)*$ && "${nv}" =~ ^[0-9]+(\.[0-9]+)*$ && "${cv}" != "${nv}" ]] || return 1
    [[ "$(printf '%s\n' "${cv}" "${nv}" | sort -V | head -n 1)" == "${cv}" ]]
}

# Programs running in the container: apart from docker-init and the resident sleep infinity, the rest
# are in use by the user (including open shells).
# If they cannot be seen (docker top fails), return failure, which is treated as "programs running"
busy_procs() {
    local out
    out="$(docker top "${NAME}" -o pid,args 2>/dev/null)" || return 1
    awk 'NR > 1 { $1 = ""; sub(/^ +/, "")
                  if ($0 == "sleep infinity" || $1 == "/sbin/docker-init" || $1 == "/dev/init") next
                  print }' <<<"${out}"
}

# Full command to recreate the container with the current settings (options given this time + kept
# options). Always give this when telling the user to --restart:
# --restart alone recreates exactly per that command line and drops the kept --data, --ble, -e
restart_cmd() {
    local cmd=(./run.sh --restart) e
    [[ "${NAME}" == revohuman ]] || cmd+=(--name "${NAME}")
    [[ "${IMAGE}" == "${DEFAULT_IMAGE}" ]] || cmd+=(--image "${IMAGE}")
    [[ "${DATA_DIR}" == "${DEFAULT_DATA}" ]] || cmd+=(--data "${DATA_DIR}")
    [[ "${STATE_DIR}" == "${DEFAULT_STATE}" ]] || cmd+=(--state "${STATE_DIR}")
    [[ ${BLE} == 0 ]] || cmd+=(--ble)
    [[ ${GUI} == 1 ]] || cmd+=(--no-gui)
    for e in "${EXTRA_ENV[@]}"; do cmd+=(-e "${e}"); done
    printf '%q ' "${cmd[@]}" | sed 's/ $//'
}

UPGRADE=""      # now: recreate from the updated image;
                # later: programs are still running in the container, do not switch this time
BUSY=""
CUR_ID="" CUR_IMAGE="" CUR_HOME="" CUR_CONFIG=""
WANT_ID="$(docker image inspect -f '{{.Id}}' "${IMAGE}" 2>/dev/null || true)"
if [[ ${ACTION} == run ]] && exists; then
    IFS='|' read -r CUR_ID CUR_IMAGE CUR_HOME CUR_CONFIG <<<"$(docker container inspect -f \
        '{{.Image}}|{{.Config.Image}}|{{index .Config.Labels "org.revohuman.home"}}|{{index .Config.Labels "org.revohuman.run-config"}}' \
        "${NAME}" 2>/dev/null || true)"
    # Only containers created by run.sh (with the settings digest label); other containers of the same name
    # as before: recreated with this run's options if stopped, only a notice if running
    if [[ -n "${CUR_CONFIG}" ]]; then
        inherit_settings
        if [[ -n "${CUR_ID}" && -n "${WANT_ID}" && "${CUR_ID}" != "${WANT_ID}" ]] && older_image "${CUR_IMAGE}"; then
            if ! running; then
                UPGRADE=now
            elif BUSY="$(busy_procs)"; then
                if [[ -n "${BUSY}" ]]; then UPGRADE=later; else UPGRADE=now; fi
            else
                UPGRADE=later; BUSY="（看不到容器里的进程）"
            fi
        fi
    fi
fi
OLD_DESC="镜像 ${CUR_IMAGE}"
[[ "${CUR_IMAGE}" != "${IMAGE}" ]] || OLD_DESC="重新构建之前的镜像 ${IMAGE}"

create_args
CONFIG_HASH="$(printf '%s\n' "${IMAGE}" "${ARGS[@]}" | sha1sum | cut -c1-12)"

# When mounting device nodes one by one (before Docker 25), record each glove node's inode at container
# creation: after replugging, the host has a different node for the glove, while the container still
# has the old one mounted (opening it gives Permission denied); the path and the settings digest above
# are unchanged, so only the inode can tell.
# The leading v1 tells "created with no glove plugged in" apart from "old container without this
# label": the latter is not compared
dev_snapshot() {
    [[ ${DEV_HOTPLUG} == 0 ]] || return 0
    local tty
    printf 'v1 '
    for tty in "${GLOVE_TTYS[@]}" "${VIDEO_NODES[@]}"; do printf '%s@%s ' "${tty}" "$(stat -c %i "${tty}")"; done
}

# X11 authorization: convert the current DISPLAY's authorization into a hostname-independent form in the
# state directory (refreshed on every entry)
refresh_xauth() {
    [[ ${GUI} == 1 && -n "${DISPLAY}" ]] || return 0
    [[ ${DRY_RUN} == 1 ]] && return 0
    local xa="${STATE_DIR}/x11/Xauthority"
    mkdir -p "${STATE_DIR}/x11"
    rm -f "${xa}.new"; touch "${xa}.new"
    if command -v xauth >/dev/null && xauth nlist "${DISPLAY}" 2>/dev/null | sed -e 's/^..../ffff/' | xauth -f "${xa}.new" nmerge - 2>/dev/null; then
        :
    elif [[ -r "${XAUTHORITY:-${HOME}/.Xauthority}" ]]; then
        cp "${XAUTHORITY:-${HOME}/.Xauthority}" "${xa}.new"
    fi
    chmod 600 "${xa}.new"; mv -f "${xa}.new" "${xa}"
}

# ---------- Create / reuse the container ----------
REMOVED=0       # remember the removal; with --dry-run the container actually still exists
if [[ ${ACTION} == restart ]] && exists; then
    run docker rm -f "${NAME}" >/dev/null; REMOVED=1
fi

if [[ ${UPGRADE} == later ]]; then
    # The container was created from an older image, but programs are still running in it (recording,
    # visualization, open shells, etc.): do not interrupt them; enter the old container this time
    warn "容器 ${NAME} 是用${OLD_DESC} 创建的，里面运行的还是旧版本的程序。容器里正在运行：
$(sed 's/^/        /' <<<"${BUSY}")
    这次先进入这个旧容器，不打断它们。等它们都退出后再执行 ./run.sh，会自动换成 ${IMAGE}（原来的设置照旧）；
    要现在就换（上面这些程序会被结束）：$(restart_cmd)"
elif [[ ${REMOVED} == 0 && ${UPGRADE} != now ]] && running; then
    # Compare the image itself first: when the container was created from another image (e.g. a downgrade,
    # or a different image name), the programs running in it are that image's programs,
    # so say which image it is, not just "different settings"
    if [[ -n "${CUR_ID}" && -n "${WANT_ID}" && "${CUR_ID}" != "${WANT_ID}" ]]; then
        warn "容器 ${NAME} 是用镜像 ${CUR_IMAGE:-（未知）} 创建的，不是 ${IMAGE}，容器里运行的还是那个镜像里的程序。容器里没有正在运行的程序时重建容器（数据与标定在主机目录里，不受影响）：$(restart_cmd)"
    elif [[ "$(docker container inspect -f '{{index .Config.Labels "org.revohuman.run-config"}}' "${NAME}")" != "${CONFIG_HASH}" ]]; then
        warn "容器 ${NAME} 是按另一组设置创建的（例如$([[ ${DEV_HOTPLUG} == 1 ]] || printf '插拔过手套、')换了镜像或选项）。要让当前设置生效，重建容器：$(restart_cmd)"
    elif SNAP="$(docker container inspect -f '{{index .Config.Labels "org.revohuman.dev-snapshot"}}' "${NAME}")" \
            && [[ -n "${SNAP}" && "${SNAP}" != "$(dev_snapshot)" ]]; then
        warn "容器 ${NAME} 创建之后插拔过手套，容器里的手套串口已失效（打开时报 Permission denied）。重建容器：$(restart_cmd)；Docker 升级到 25 及以上后插拔不用再重建"
    fi
else
    if [[ ${REMOVED} == 0 ]] && exists; then run docker rm -f "${NAME}" >/dev/null; fi
    [[ ${UPGRADE} != now ]] \
        || echo "run.sh: 容器 ${NAME} 原来是用${OLD_DESC} 创建的，换成 ${IMAGE} 重新创建（录制数据与标定在主机目录里，不受影响）" >&2
    [[ ${#INHERITED[@]} == 0 ]] || echo "run.sh: 沿用原来容器的设置：$(printf '%q ' "${INHERITED[@]}" | sed 's/ $//')" >&2
    if [[ ${DRY_RUN} == 0 ]]; then
        mkdir -p "${DATA_DIR}" "${STATE_DIR}/config/libsurvive" "${STATE_DIR}/config/revohuman" \
                 "${STATE_DIR}/state/revohuman" "${STATE_DIR}/ros" "${STATE_DIR}/x11"
        mkdir -p -m 700 "${LIVE_DIR}"
    fi
    [[ ${#GLOVE_TTYS[@]} -gt 0 || ${DEV_HOTPLUG} == 1 ]] \
        || warn "没有发现手套串口。容器启动之后再插上的手套在容器里看不到，插好后重建容器：$(restart_cmd)"
    for sys in /sys/bus/usb/devices/*; do
        [[ "$(cat "${sys}/idVendor" 2>/dev/null):$(cat "${sys}/idProduct" 2>/dev/null)" == 28de:2300 ]] || continue
        node="$(printf '/dev/bus/usb/%03d/%03d' "$(cat "${sys}/busnum")" "$(cat "${sys}/devnum")")"
        [[ -r "${node}" && -w "${node}" ]] \
            || warn "Tracker ${node} 当前用户无权读写：按手册《01-环境安装与系统要求》第 6 节安装 Tracker 设备规则，并重新插拔手套单元"
    done
    run docker run -d --name "${NAME}" --label "org.revohuman.run-config=${CONFIG_HASH}" \
        --label "org.revohuman.dev-snapshot=$(dev_snapshot)" \
        "${ARGS[@]}" "${IMAGE}" sleep infinity >/dev/null
    if [[ ${DRY_RUN} == 0 ]]; then
        if [[ ${DEV_HOTPLUG} == 1 ]]; then
            echo "已启动容器 ${NAME}（数据目录：${DATA_DIR}；插拔手套不用重建容器）" >&2
        else
            echo "已启动容器 ${NAME}（手套串口：${GLOVE_TTYS[*]:-无}；数据目录：${DATA_DIR}）" >&2
            warn "当前 Docker 版本低于 25，插拔手套之后要用 --restart 重建容器（进入容器时会提示，并给出完整命令）"
        fi
        # The replaced old image (several GB) is not deleted automatically: if no other container uses it,
        # tell the user how to delete it
        if [[ ${UPGRADE} == now && -z "$(docker ps -aq --filter "ancestor=${CUR_ID}" 2>/dev/null || echo ?)" ]]; then
            OLD_REF="${CUR_ID#sha256:}"; OLD_REF="${OLD_REF:0:12}"
            [[ "${CUR_IMAGE}" == "${IMAGE}" || "$(docker image inspect -f '{{.Id}}' "${CUR_IMAGE}" 2>/dev/null)" != "${CUR_ID}" ]] \
                || OLD_REF="${CUR_IMAGE}"
            echo "run.sh: 旧镜像 ${OLD_REF} 已经没有容器在用。确认新版本没问题之后，可以执行 docker image rm ${OLD_REF} 删掉它，腾出磁盘空间" >&2
        fi
    fi
fi

# ---------- Enter the container and run the command ----------
refresh_xauth
EXEC=(docker exec -i -w "${CHOME}/data")
if [[ -t 0 && -t 1 ]]; then
    EXEC+=(-t)
    [[ -z "${TERM}" ]] || EXEC+=(-e "TERM=${TERM}")
fi
[[ ${GUI} == 1 && -n "${DISPLAY}" ]] && EXEC+=(-e "DISPLAY=${DISPLAY}")
for v in NO_COLOR REVOHUMAN_LOG_COLOR; do
    [[ -z "${!v+x}" ]] || EXEC+=(-e "${v}=${!v}")
done
if [[ ${DRY_RUN} == 1 ]]; then
    run "${EXEC[@]}" "${NAME}" revohuman-entrypoint "$@"
else
    exec "${EXEC[@]}" "${NAME}" revohuman-entrypoint "$@"
fi
