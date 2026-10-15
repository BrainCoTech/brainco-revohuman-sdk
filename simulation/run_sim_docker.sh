#!/usr/bin/env bash
# 在主机上执行：在 RevoHuman 容器里一条命令拉起手套的本机广播与三维可视化（容器里的 run_sim_host.sh），
# 窗口显示在主机桌面上。关闭窗口、按 Ctrl+C 或关掉终端时一并停止容器里本脚本启动的广播与可视化。
#
#   simulation/run_sim_docker.sh                          接真手套
#   simulation/run_sim_docker.sh --hands left --lite      参数与 run_sim_host.sh 相同
#   simulation/run_sim_docker.sh --name revohuman2 …      换容器名 / 镜像（--image），交给 docker/run.sh
#
# 容器没在运行时由 docker/run.sh 先创建它；容器的其他选项（数据目录、蓝牙、--restart 等）先用 docker/run.sh 设好。
# 已经在容器里时直接改用 run_sim_host.sh。完整说明见使用手册《08-容器运行》第 8 节。
set -eo pipefail

SIM_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
RUN_SH="$(dirname "${SIM_DIR}")/docker/run.sh"
STOP_WAIT=15
IMAGE=""
NAME="revohuman"
RUN_ARGS=()
SIM_ARGS=()

usage() {
    cat <<'EOF'
用法：run_sim_docker.sh [--image 镜像] [--name 容器名] [run_sim_host.sh 的参数…]

在主机上执行：进入 RevoHuman 容器（没在运行时先由 docker/run.sh 创建），在容器里执行 run_sim_host.sh——
后台启动手套广播、前台打开可视化窗口（显示在主机桌面上）；关闭窗口、按 Ctrl+C 或关掉终端时一并停止它们。

选项：
  --image 镜像        创建容器时用的镜像（默认 revohuman:0.1.1），交给 docker/run.sh
  --name 容器名       进入哪个容器（默认 revohuman），交给 docker/run.sh
  -h, --help          显示本帮助

其余参数原样交给容器里的 run_sim_host.sh（--hands、--product、--no-tracker、--demo、--lite、
-- 之后的 revohuman-live 参数等，见 run_sim_host.sh --help）。容器里固定使用镜像自带的 SDK，不接受 --venv。
EOF
}

say()  { printf '[run_sim_docker] %s\n' "$*"; }
die()  { printf '[run_sim_docker] %s\n' "$*" >&2; exit 2; }

while [[ $# -gt 0 ]]; do
    case "$1" in
        --) SIM_ARGS+=("$@"); break ;;
        --image|--name)
            [[ $# -ge 2 ]] || die "$1 需要参数"
            if [[ "$1" == --image ]]; then IMAGE="$2"; else NAME="$2"; fi
            RUN_ARGS+=("$1" "$2"); shift 2 ;;
        --image=*|--name=*) set -- "${1%%=*}" "${1#*=}" "${@:2}" ;;
        --venv|--venv=*) die "容器里固定使用镜像自带的 SDK，不接受 --venv" ;;
        -h|--help) usage; exit 0 ;;
        *) SIM_ARGS+=("$1"); shift ;;
    esac
done

# 已经在容器里：直接执行 run_sim_host.sh
if [[ -f /.dockerenv || -n "${container:-}" ]]; then
    [[ ${#RUN_ARGS[@]} == 0 ]] || die "已经在容器里了，--image、--name 不起作用；去掉它们，或直接执行 ${SIM_DIR}/run_sim_host.sh"
    exec "${SIM_DIR}/run_sim_host.sh" --venv none "${SIM_ARGS[@]}"
fi

command -v docker >/dev/null || die "找不到 docker 命令：先按手册《08-容器运行》安装 Docker 并构建镜像（docker/install.sh）"
[[ -x "${RUN_SH}" ]] || die "找不到 ${RUN_SH}（源码包不完整？）"
[[ -z "${IMAGE}" ]] || say "镜像：${IMAGE}"

# 容器里执行的脚本：找到镜像里源码包的 run_sim_host.sh，记下它的 pid（放在挂载到主机的状态目录里），
# 主机这边被打断时据此停止它。旧镜像里的源码包没有 run_sim_host.sh，要用主机上这份源码包重建镜像
TOKEN="$$-${RANDOM}${RANDOM}"
INNER='tok=$1; shift
shopt -s nullglob
c=(~/revohuman-release/revohuman-*/simulation/run_sim_host.sh)
if [[ ${#c[@]} != 1 ]]; then
    echo "[run_sim_docker] 镜像里的源码包没有 run_sim_host.sh：用主机上这份源码包的 docker/install.sh 重建镜像后执行 docker/run.sh --restart" >&2
    exit 3
fi
exec "${c[0]}" --venv none --pidfile "$HOME/.local/state/revohuman/sim_docker.$tok.pid" "$@"'

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
"${RUN_SH}" "${RUN_ARGS[@]}" -- bash -c "${INNER}" run_sim_host "${TOKEN}" "${SIM_ARGS[@]}" <&0 &
CHILD=$!
rc=0
wait "${CHILD}" || rc=$?
CHILD=""
exit "${rc}"
