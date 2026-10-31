#!/usr/bin/env bash
# 在主机上执行：在 RevoHuman 容器里一条命令拉起手套的本机广播与三维可视化（容器里的 run_sim_host.sh），
# 窗口显示在主机桌面上。关闭窗口、按 Ctrl+C 或关掉终端时一并停止容器里本脚本启动的广播与可视化。
#
#   simulation/run_sim_docker.sh                          接真手套
#   simulation/run_sim_docker.sh --hands left --lite      参数与 run_sim_host.sh 相同
#   simulation/run_sim_docker.sh --name revohuman2 …      换容器名 / 镜像（--image），交给 docker/run.sh
#   simulation/run_sim_docker.sh --restart                先按这次给的选项重建容器，交给 docker/run.sh
#   simulation/run_sim_docker.sh --ble …                  容器的创建选项（--data、--state、--ble、-e）同样交给 docker/run.sh
#
# 容器没在运行时由 docker/run.sh 先创建它；没给的 --data、--state、--ble、-e 沿用原来容器的设置，
# 升级之后 docker/run.sh 会自动换成新镜像（都与直接执行 docker/run.sh 相同）。
# 已经在容器里时直接改用 run_sim_host.sh。完整说明见使用手册《08-容器运行》第 8 节。
set -eo pipefail

SIM_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
RUN_SH="$(dirname "${SIM_DIR}")/docker/run.sh"
STOP_WAIT=15
IMAGE=""
NAME="revohuman"
RESTART=0
OPTS=()       # 容器的创建选项（--data、--state、--ble、-e），提示里重建容器的命令要带上
RUN_ARGS=()
SIM_ARGS=()

usage() {
    cat <<'EOF'
用法：run_sim_docker.sh [--image 镜像] [--name 容器名] [--restart] [--data 目录] [--state 目录] [--ble]
                         [-e 名=值]… [run_sim_host.sh 的参数…]

在主机上执行：进入 RevoHuman 容器（没在运行时先由 docker/run.sh 创建），在容器里执行 run_sim_host.sh——
后台启动手套广播、前台打开可视化窗口（显示在主机桌面上）；关闭窗口、按 Ctrl+C 或关掉终端时一并停止它们。

选项：
  --image 镜像        创建容器时用的镜像（默认 revohuman:0.1.2），交给 docker/run.sh
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

# 已经在容器里：直接执行 run_sim_host.sh
if [[ -f /.dockerenv || -n "${container:-}" ]]; then
    [[ ${#RUN_ARGS[@]} == 0 ]] || die "已经在容器里了，--image、--name、--restart、--data、--state、--ble、-e 不起作用；去掉它们，或直接执行 ${SIM_DIR}/run_sim_host.sh"
    exec "${SIM_DIR}/run_sim_host.sh" --venv none "${SIM_ARGS[@]}"
fi

command -v docker >/dev/null || die "找不到 docker 命令：先按手册《08-容器运行》安装 Docker 并构建镜像（docker/install.sh）"
[[ -x "${RUN_SH}" ]] || die "找不到 ${RUN_SH}（源码包不完整？）"
[[ -z "${IMAGE}" ]] || say "镜像：${IMAGE}"

# 容器里执行的脚本：找到镜像里源码包的 run_sim_host.sh，记下它的 pid（放在挂载到主机的状态目录里），
# 主机这边被打断时据此停止它。0.1.0 的源码包没有 run_sim_host.sh：多半是升级之前创建的容器里还有程序在运行、
# docker/run.sh 没有自动换成新镜像，先重建容器（--restart）；重建之后仍然没有，才是镜像本身旧，要用主机上这份源码包重新构建镜像
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
stop_inner() {   # 停止容器里本次启动的 run_sim_host.sh（它会接着停下自己启动的可视化与广播）
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

# docker/run.sh 放到后台再等它：这样收到信号能立刻处理（docker exec 不把信号转进容器）；
# <&0 保留终端输入，run.sh 才会给容器分配终端，窗口里的 Ctrl+C 也才能直接送进容器
"${RUN_SH}" "${RUN_ARGS[@]}" -- bash -c "${INNER}" run_sim_host "${TOKEN}" "${HINT}" "${SIM_ARGS[@]}" <&0 &
CHILD=$!
rc=0
wait "${CHILD}" || rc=$?
CHILD=""
exit "${rc}"
