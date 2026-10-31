#!/usr/bin/env bash
# 在容器里运行 RevoHuman。容器在后台常驻，每次执行本脚本都进入同一个容器，
# 所以可以在多个终端里分别跑 revohuman-live、可视化、ros2 等命令。
#
#   ./run.sh                     进入容器的 shell（容器没在运行时先启动它）
#   ./run.sh <命令> [参数…]       在容器里执行一条命令，例如 ./run.sh python3 -m revohuman doctor
#   ./run.sh --restart [命令…]    按这次的选项重建容器后再进入（重新插拔手套、改了选项之后用）
#   ./run.sh --stop               停止并删除容器（数据与标定在宿主目录里，不受影响）
#
# 容器与宿主上的程序共用同一套设备锁、共享内存与实时广播 socket，效果与在宿主上直接运行相同：
# 同一只手套、同一组 Tracker 同一时间只能有一个程序使用，不论这个程序在容器里还是在宿主上。
#
# 升级到新版本（或重新构建镜像）之后，容器里没有正在运行的程序时，./run.sh 自动换成新镜像重建容器，
# 原来创建容器时给的 --data、--state、--ble、-e 照旧；容器里还有程序在运行时只提示，不打断它们。
# 完整说明见使用手册《08-容器运行》。

set -eo pipefail

IMAGE="revohuman:0.1.2"
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
  --image 镜像        使用的镜像（默认 revohuman:0.1.2）
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

# ---------- 检查镜像与用户 ----------
[[ "$(id -u)" != 0 ]] || die "不要用 root 运行本脚本：把当前用户加进 docker 组后以普通用户运行"
docker image inspect "${IMAGE}" >/dev/null 2>&1 \
    || die "找不到镜像 ${IMAGE}：先执行同目录的 ./install.sh 构建$([[ ${IMAGE} == revohuman:0.1.2 ]] || printf '（加 --tag %s）' "${IMAGE}")，说明见手册《08-容器运行》"
label() { docker image inspect -f "{{index .Config.Labels \"$1\"}}" "${IMAGE}"; }
IMG_UID="$(label org.revohuman.uid)"; IMG_GID="$(label org.revohuman.gid)"; CHOME="$(label org.revohuman.home)"
[[ -n "${IMG_UID}" && -n "${CHOME}" ]] || die "镜像 ${IMAGE} 不是用源码包里的 docker/Dockerfile 构建的"
if [[ "${IMG_UID}" != "$(id -u)" || "${IMG_GID}" != "$(id -g)" ]]; then
    die "镜像里的用户是 ${IMG_UID}:${IMG_GID}，当前用户是 $(id -u):$(id -g)。两者必须一致（挂载目录的属主与 Tracker 锁都按用户号判断），
    执行同目录的 ./install.sh 为当前用户构建一份（镜像是本机别的用户在用时加 --tag 换个名字，再用 --image 指定）"
fi

# ---------- 组装创建容器的参数 ----------
create_args() {
    ARGS=(--init --network host
          -v /dev/shm:/dev/shm
          -v "${DATA_DIR}:${CHOME}/data"
          -v "${STATE_DIR}/config/libsurvive:${CHOME}/.config/libsurvive"
          -v "${STATE_DIR}/config/revohuman:${CHOME}/.config/revohuman"
          -v "${STATE_DIR}/state/revohuman:${CHOME}/.local/state/revohuman"
          -v "${STATE_DIR}/ros:${CHOME}/.ros")

    # 时区与宿主一致（否则容器里的时间与日志差几个小时）
    local tz=""
    [[ -r /etc/timezone ]] && tz="$(head -n 1 /etc/timezone)"
    [[ -n "${tz}" ]] || tz="$(readlink /etc/localtime 2>/dev/null | sed -n 's#.*/zoneinfo/##p')"
    [[ -z "${tz}" ]] || ARGS+=(-e "TZ=${tz}")

    # ROS 2 的域设置与宿主一致
    local v
    for v in ROS_DOMAIN_ID ROS_LOCALHOST_ONLY RMW_IMPLEMENTATION; do
        [[ -z "${!v}" ]] || ARGS+=(-e "${v}=${!v}")
    done
    for v in "${EXTRA_ENV[@]}"; do ARGS+=(-e "${v}"); done

    # 实时广播 socket 与宿主共用同一路径（与 SDK 的默认路径规则相同）：宿主上的可视化能连容器里的 revohuman-live，反过来也一样
    ARGS+=(-e "REVOHUMAN_LIVE_SOCKET=${LIVE_DIR}/live.sock" -v "${LIVE_DIR}:${LIVE_DIR}")

    # 手套串口。容器里打开的必须是宿主的同一个设备节点，占用检查才对容器内外同时有效。
    # Docker 25 及以上：把宿主的 /dev 只读挂到 /run/host/dev（只挂这一层，/dev/pts 等子挂载不进来），
    # SDK 经 REVOHUMAN_DEV_DIR 到这里找手套。容器起来之后插拔手套，新节点直接可见，不用重建容器；
    # 能打开哪些设备只由下面的 cgroup 规则决定（只放行手套串口、视频设备与 USB 设备）。
    # 更早的 Docker 不支持只挂一层：把创建容器时插着的每个手套节点本身挂进去，插拔之后要 --restart。
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
        # 之后才插上的手套也要能打开：串口所属的组（一般是 dialout）不论此刻插没插手套都加上
        local gid
        gid="$(getent group dialout | cut -d: -f3)"
        [[ -z "${gid}" ]] || groups+=("${gid}")
    fi
    ARGS+=(--device-cgroup-rule 'c 166:* rmw')

    # 腕部相机（V4L2）：与手套串口同样的做法。SDK 按 sysfs 认出手套单元里的相机，到同一个设备目录下打开它
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

    # Tracker（libsurvive 通过 libusb 访问）：挂整个 USB 设备目录，重新枚举后编号变了也能找到
    ARGS+=(-v /dev/bus/usb:/dev/bus/usb --device-cgroup-rule 'c 189:* rmw')
    [[ -d /run/udev ]] && ARGS+=(-v /run/udev:/run/udev:ro)

    # 图形界面：X11 socket + 显卡（不给显卡权限时可视化会悄悄退回软件渲染，帧率很低）
    if [[ ${GUI} == 1 ]]; then
        [[ -d /tmp/.X11-unix ]] && ARGS+=(-v /tmp/.X11-unix:/tmp/.X11-unix:ro)
        ARGS+=(-v "${STATE_DIR}/x11:/run/revohuman-x11:ro" -e XAUTHORITY=/run/revohuman-x11/Xauthority)
        if [[ -d /dev/dri ]]; then
            ARGS+=(--device /dev/dri)
            for dev in /dev/dri/*; do [[ -c "${dev}" ]] && groups+=("$(stat -c %g "${dev}")"); done
        fi
    fi

    # 蓝牙：经宿主的 D-Bus 使用 BlueZ
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

# ---------- 现有容器：沿用它的设置；它是用旧镜像创建的时，判断能不能直接换成新镜像 ----------
# 不加 --restart 时，命令行上没给的 --data、--state、--ble、-e 以现有容器为准（从它的挂载与环境变量读出来）：
# 容器停了（例如重启过电脑）或要换成新镜像而重建时，不会悄悄丢掉创建时给的选项；容器在运行时，
# 也只有这次明确给出、又与创建时不同的选项才算「设置不同」。
# 加 --restart 时完全按这次的命令行，例如不带 --ble 重建，换回不接蓝牙的容器
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

    # 用 -e 加的环境变量＝容器的环境变量去掉它的镜像本来就有的、去掉 run.sh 按宿主设置的那几个。
    # 这次给了同名变量的用这次的值；按容器里原来的顺序排，设置没变时设置摘要也不变
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

# 现有容器的镜像比这次要用的旧：同名镜像重新构建过，或者同一镜像名下版本号更低（例如 revohuman:0.1.1 → 0.1.2）。
# 降级、换成另一个镜像名时不自动换，只提示
older_image() {
    local cv nv
    [[ "$1" == "${IMAGE}" ]] && return 0
    [[ "$1" == *:* && "${1%:*}" == "${IMAGE%:*}" ]] || return 1
    cv="${1##*:}"; nv="${IMAGE##*:}"
    [[ "${cv}" =~ ^[0-9]+(\.[0-9]+)*$ && "${nv}" =~ ^[0-9]+(\.[0-9]+)*$ && "${cv}" != "${nv}" ]] || return 1
    [[ "$(printf '%s\n' "${cv}" "${nv}" | sort -V | head -n 1)" == "${cv}" ]]
}

# 容器里正在运行的程序：除去 docker-init 与常驻的 sleep infinity，剩下的都是用户在用的（包括开着的 shell）。
# 看不到时（docker top 失败）返回失败，按「有程序在运行」处理
busy_procs() {
    local out
    out="$(docker top "${NAME}" -o pid,args 2>/dev/null)" || return 1
    awk 'NR > 1 { $1 = ""; sub(/^ +/, "")
                  if ($0 == "sleep infinity" || $1 == "/sbin/docker-init" || $1 == "/dev/init") next
                  print }' <<<"${out}"
}

# 按现在这组设置（这次给的选项＋沿用的选项）重建容器的完整命令。提示用户 --restart 时一律给它：
# 只写 --restart 会完全按那次的命令行重建，把沿用下来的 --data、--ble、-e 丢掉
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

UPGRADE=""      # now：换成新镜像重建；later：容器里还有程序在运行，这次不换
BUSY=""
CUR_ID="" CUR_IMAGE="" CUR_HOME="" CUR_CONFIG=""
WANT_ID="$(docker image inspect -f '{{.Id}}' "${IMAGE}" 2>/dev/null || true)"
if [[ ${ACTION} == run ]] && exists; then
    IFS='|' read -r CUR_ID CUR_IMAGE CUR_HOME CUR_CONFIG <<<"$(docker container inspect -f \
        '{{.Image}}|{{.Config.Image}}|{{index .Config.Labels "org.revohuman.home"}}|{{index .Config.Labels "org.revohuman.run-config"}}' \
        "${NAME}" 2>/dev/null || true)"
    # 只处理 run.sh 创建的容器（带设置摘要标签）；别的同名容器照旧：停着就按这次的选项重建，在运行就只提示
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

# 逐个挂设备节点时（Docker 25 以前），记下创建容器时每个手套节点的 inode：手套插拔后宿主上换成了新节点，
# 容器里挂着的还是旧的那个（打开报 Permission denied），路径不变、上面的设置摘要也不变，只能靠 inode 认出来。
# 开头的 v1 让「没插手套时创建」与「没有这个标签的旧容器」区分开：后者不比
dev_snapshot() {
    [[ ${DEV_HOTPLUG} == 0 ]] || return 0
    local tty
    printf 'v1 '
    for tty in "${GLOVE_TTYS[@]}" "${VIDEO_NODES[@]}"; do printf '%s@%s ' "${tty}" "$(stat -c %i "${tty}")"; done
}

# X11 授权：把当前 DISPLAY 的授权转成不绑定主机名的形式放进状态目录（每次进入都刷新）
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

# ---------- 创建 / 复用容器 ----------
REMOVED=0       # 记下删过，--dry-run 时容器其实还在
if [[ ${ACTION} == restart ]] && exists; then
    run docker rm -f "${NAME}" >/dev/null; REMOVED=1
fi

if [[ ${UPGRADE} == later ]]; then
    # 容器是用旧镜像创建的，但里面还有程序在运行（录制、可视化、开着的 shell 等）：不打断它们，这次先进入旧容器
    warn "容器 ${NAME} 是用${OLD_DESC} 创建的，里面运行的还是旧版本的程序。容器里正在运行：
$(sed 's/^/        /' <<<"${BUSY}")
    这次先进入这个旧容器，不打断它们。等它们都退出后再执行 ./run.sh，会自动换成 ${IMAGE}（原来的设置照旧）；
    要现在就换（上面这些程序会被结束）：$(restart_cmd)"
elif [[ ${REMOVED} == 0 && ${UPGRADE} != now ]] && running; then
    # 先比镜像本身：容器是用另一个镜像创建的（例如降级，或换了镜像名）时，容器里运行的是那个镜像里的程序，
    # 要说清楚是哪个镜像，不能只说「设置不同」
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
        # 换下来的旧镜像（每个几个 GB）不自动删：没有别的容器在用时告诉用户怎么删
        if [[ ${UPGRADE} == now && -z "$(docker ps -aq --filter "ancestor=${CUR_ID}" 2>/dev/null || echo ?)" ]]; then
            OLD_REF="${CUR_ID#sha256:}"; OLD_REF="${OLD_REF:0:12}"
            [[ "${CUR_IMAGE}" == "${IMAGE}" || "$(docker image inspect -f '{{.Id}}' "${CUR_IMAGE}" 2>/dev/null)" != "${CUR_ID}" ]] \
                || OLD_REF="${CUR_IMAGE}"
            echo "run.sh: 旧镜像 ${OLD_REF} 已经没有容器在用。确认新版本没问题之后，可以执行 docker image rm ${OLD_REF} 删掉它，腾出磁盘空间" >&2
        fi
    fi
fi

# ---------- 进入容器执行命令 ----------
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
