#!/usr/bin/env bash
# Run on the host: in the RevoHuman container, start the gloves' local broadcast and the 3D visualization
# with one command (run_sim_host.sh in the container), with the window shown on the host desktop. Closing
# the window, pressing Ctrl+C or closing the terminal also stops the broadcast and visualization that this
# script started in the container.
#
#   simulation/run_sim_docker.sh                          with real gloves
#   simulation/run_sim_docker.sh --hands left --lite      same arguments as run_sim_host.sh
#   simulation/run_sim_docker.sh --name revohuman2 …      another container name / image (--image),
#                                                        passed to docker/run.sh
#   simulation/run_sim_docker.sh --restart                first recreate the container with the options
#                                                        given this time, passed to docker/run.sh
#   simulation/run_sim_docker.sh --ble …                  container creation options (--data, --state,
#                                                        --ble, -e) are passed to docker/run.sh as well
#
# When the container is not running, docker/run.sh creates it first; any of --data, --state, --ble, -e
# not given follows the existing container's settings, and after an upgrade docker/run.sh switches to
# the updated image automatically (both the same as running docker/run.sh directly).
# When already inside the container, use run_sim_host.sh directly instead. Full details: user manual
# *08 Running in Containers*, section 8.
set -eo pipefail

SIM_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
RUN_SH="$(dirname "${SIM_DIR}")/docker/run.sh"
STOP_WAIT=15
IMAGE=""
NAME="revohuman"
RESTART=0
OPTS=()       # container creation options (--data, --state, --ble, -e);
              # the recreate command shown in hints must include them
RUN_ARGS=()
SIM_ARGS=()

usage() {
    cat <<'EOF'
用法：run_sim_docker.sh [--image 镜像] [--name 容器名] [--restart] [--data 目录] [--state 目录] [--ble]
                         [-e 名=值]… [run_sim_host.sh 的参数…]

在主机上执行：进入 RevoHuman 容器（没在运行时先由 docker/run.sh 创建），在容器里执行 run_sim_host.sh——
后台启动手套广播、前台打开可视化窗口（显示在主机桌面上）；关闭窗口、按 Ctrl+C 或关掉终端时一并停止它们。

选项：
  --image 镜像        创建容器时用的镜像（默认 revohuman:0.1.4），交给 docker/run.sh
  --name 容器名       进入哪个容器（默认 revohuman），交给 docker/run.sh
  --restart           先删除正在运行的容器、完全按这次给的选项重新创建再进入，交给 docker/run.sh；
                      升级之后容器里还有程序在运行、docker/run.sh 没有自动换成新镜像时，加它立即换
  --data 目录         挂到容器 ~/data 的目录，交给 docker/run.sh
  --state 目录        标定与运行状态的存放目录，交给 docker/run.sh
  --ble               允许容器使用主机蓝牙，交给 docker/run.sh
  -e, --env 名=值     给容器加环境变量，可重复，交给 docker/run.sh
  -h, --help          显示本帮助

--data、--state、--ble、-e 只在创建容器时生效，这里没给的沿用原来容器的设置（与 docker/run.sh 相同）；
加 --restart 时完全按这里给的选项重建容器，原来加过的要一并写上。

其余参数原样交给容器里的 run_sim_host.sh（--hands、--product、--no-tracker、--demo、--lite、
-- 之后的 revohuman-live 参数等，见 run_sim_host.sh --help）。容器里固定使用镜像自带的 SDK，不接受 --venv。
EOF
}

say()  { printf '[run_sim_docker] %s\n' "$*"; }
die()  { printf '[run_sim_docker] %s\n' "$*" >&2; exit 2; }

while [[ $# -gt 0 ]]; do
    case "$1" in
        --) SIM_ARGS+=("$@"); break ;;
        --image|--name|--data|--state|-e|--env)
            [[ $# -ge 2 ]] || die "$1 需要参数"
            case "$1" in
                --image) IMAGE="$2" ;;
                --name) NAME="$2" ;;
                *) OPTS+=("$1" "$2") ;;
            esac
            RUN_ARGS+=("$1" "$2"); shift 2 ;;
        --image=*|--name=*|--data=*|--state=*|--env=*) set -- "${1%%=*}" "${1#*=}" "${@:2}" ;;
        --restart) RESTART=1; RUN_ARGS+=("$1"); shift ;;
        --ble) OPTS+=("$1"); RUN_ARGS+=("$1"); shift ;;
        --venv|--venv=*) die "容器里固定使用镜像自带的 SDK，不接受 --venv" ;;
        -h|--help) usage; exit 0 ;;
        *) SIM_ARGS+=("$1"); shift ;;
    esac
done

# Already inside the container: run run_sim_host.sh directly
if [[ -f /.dockerenv || -n "${container:-}" ]]; then
    [[ ${#RUN_ARGS[@]} == 0 ]] || die "已经在容器里了，--image、--name、--restart、--data、--state、--ble、-e 不起作用；去掉它们，或直接执行 ${SIM_DIR}/run_sim_host.sh"
    exec "${SIM_DIR}/run_sim_host.sh" --venv none "${SIM_ARGS[@]}"
fi

command -v docker >/dev/null || die "找不到 docker 命令：先按手册《08-容器运行》安装 Docker 并构建镜像（docker/install.sh）"
[[ -x "${RUN_SH}" ]] || die "找不到 ${RUN_SH}（源码包不完整？）"
[[ -z "${IMAGE}" ]] || say "镜像：${IMAGE}"

# Script run in the container: find run_sim_host.sh in the image's source bundle and record its pid (in the
# state directory mounted from the host), used to stop it when the host side is interrupted. The 0.1.0
# source bundle has no run_sim_host.sh: most likely a container created before the upgrade still has
# programs running and docker/run.sh did not switch it to the updated image, so recreate the container
# first (--restart); only if it is still missing after recreating is the image itself old, and the image
# must then be rebuilt from this source bundle on the host
TOKEN="$$-${RANDOM}${RANDOM}"
INNER='tok=$1; hint=$2; shift 2
shopt -s nullglob
c=(~/revohuman-release/revohuman-*/simulation/run_sim_host.sh)
if [[ ${#c[@]} != 1 ]]; then
    v=(~/revohuman-release/revohuman-*/); v=("${v[@]%/}"); v=("${v[@]##*/}")
    echo "[run_sim_docker] 容器里的源码包（${v[*]:-无}）没有 run_sim_host.sh，容器里还是旧版本的程序：" >&2
    echo "[run_sim_docker] ${hint}" >&2
    exit 3
fi
exec "${c[0]}" --venv none --pidfile "$HOME/.local/state/revohuman/sim_docker.$tok.pid" "$@"'
AGAIN="${SIM_DIR}/run_sim_docker.sh --restart"
[[ ${NAME} == revohuman ]] || AGAIN+=" --name ${NAME}"
[[ -z "${IMAGE}" ]] || AGAIN+=" --image ${IMAGE}"
[[ ${#OPTS[@]} == 0 ]] || AGAIN+="$(printf ' %q' "${OPTS[@]}")"
if [[ ${RESTART} == 0 ]]; then
    HINT="docker/run.sh 这次没有把容器换成新镜像（原因见上面 docker/run.sh 的提示，多半是容器里还有其他程序在运行）：等它们都退出后重新执行本命令，会自动换；要现在就换，执行 ${AGAIN} 重建容器（那些程序会被结束；创建容器时加过 --data、-e 等选项的要一并写上，docker/run.sh 的提示里有完整命令）。重建之后仍然这样，先用主机上这份源码包的 docker/install.sh 重新构建镜像"
else
    HINT="容器刚重建过，是镜像${IMAGE:+ ${IMAGE} }本身旧：用主机上这份源码包的 docker/install.sh 重新构建镜像，再执行 ${AGAIN}"
fi

STOPPED=0
stop_inner() {   # stop the run_sim_host.sh started in the container this time
                 # (it then stops the visualization and broadcast it started)
    [[ ${STOPPED} == 0 ]] || return 0
    STOPPED=1
    docker exec "${NAME}" bash -c 'f="$HOME/.local/state/revohuman/sim_docker.$1.pid"
        p="$(cat "$f" 2>/dev/null)" || exit 0
        if [[ "$p" =~ ^[0-9]+$ ]] && tr "\0" " " <"/proc/$p/cmdline" 2>/dev/null | grep -q run_sim_host; then
            kill -TERM "$p" 2>/dev/null
            for i in $(seq 1 '"$((STOP_WAIT * 10))"'); do [[ -e /proc/$p ]] || break; sleep 0.1; done
        fi
        rm -f "$f"' _ "${TOKEN}" >/dev/null 2>&1 || true
}

CHILD=""
on_signal() {
    trap - INT TERM HUP
    say "正在停止容器 ${NAME} 里的广播与可视化…"
    stop_inner
    if [[ -n "${CHILD}" ]]; then
        local i
        for (( i = 0; i < 50; i++ )); do kill -0 "${CHILD}" 2>/dev/null || break; sleep 0.1; done
        kill -TERM "${CHILD}" 2>/dev/null || true
    fi
    exit "$1"
}
trap 'stop_inner' EXIT
trap 'on_signal 130' INT
trap 'on_signal 143' TERM
trap 'on_signal 129' HUP

# Run docker/run.sh in the background and wait for it, so signals are handled immediately (docker exec
# does not forward signals into the container);
# <&0 keeps terminal input, so run.sh allocates a terminal for the container, and Ctrl+C in the window
# also goes straight into the container
"${RUN_SH}" "${RUN_ARGS[@]}" -- bash -c "${INNER}" run_sim_host "${TOKEN}" "${HINT}" "${SIM_ARGS[@]}" <&0 &
CHILD=$!
rc=0
wait "${CHILD}" || rc=$?
CHILD=""
exit "${rc}"
