#!/usr/bin/env bash
# 一条命令拉起手套的本机广播（revohuman-live）与三维可视化：广播在后台，可视化窗口在前台；
# 关闭窗口或按 Ctrl+C 时一并停止本脚本启动的广播。
#
#   simulation/run_sim_host.sh                     接真手套
#   simulation/run_sim_host.sh --hands left        只开左手
#   simulation/run_sim_host.sh --lite -- -v        可视化参数写在前面，-- 之后的交给 revohuman-live
#   simulation/run_sim_host.sh --demo P1           不起广播，只看演示（不需要 SDK）
#
# 本机已经有广播在运行（另一个终端、或容器里的 revohuman-live）时不再启动广播，只打开窗口。
# 在容器里使用时改在主机上执行 run_sim_docker.sh。完整说明见使用手册《05-仿真与可视化》第 4.1 节。
#
# 环境变量：
#   REVOHUMAN_VENV=<dir>            默认找的 SDK 虚拟环境（默认 ~/.venvs/revohuman_sdk，install_venv.sh 建的）
#   REVOHUMAN_SIM_WAIT=<秒>          最多等广播就绪多少秒，超时后先打开窗口（默认 90）
#   REVOHUMAN_LIVE_SOCKET=<path>    广播 socket（两边自动用同一个值，一般不用设）
#   其余（REVOHUMAN_SIM_VENV 等）见 run_viewer.sh
set -eo pipefail

SIM_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROOT_DIR="$(dirname "${SIM_DIR}")"
RUN_VIEWER="${SIM_DIR}/run_viewer.sh"
STATE_DIR="${XDG_STATE_HOME:-${HOME}/.local/state}/revohuman"
LOG="${STATE_DIR}/sim_live.log"
WAIT="${REVOHUMAN_SIM_WAIT:-90}"
STOP_WAIT=10
VENV_OPT=""
HANDS=""
PRODUCT=""
NO_TRACKER=0
SOCKET=""
DEMO=""
PIDFILE=""
VIEWER_ARGS=()
LIVE_ARGS=()

usage() {
    cat <<'EOF'
用法：run_sim_host.sh [选项] [可视化参数…] [-- revohuman-live 参数…]

后台启动手套的本机广播（revohuman-live），等它就绪后在前台打开三维可视化窗口；
关闭窗口或按 Ctrl+C 时一并停止本脚本启动的广播。本机已经有广播在运行时只打开窗口。

选项：
  --hands both|left|right   开哪几只手（默认 both）
  --product auto|P1|PTV1    型号（默认 auto，自动识别）
  --no-tracker              广播端不开 Tracker，可视化也不画 Tracker 位姿，只看手指关节
  --socket 路径             广播 socket（广播与可视化用同一个；默认按固定规则找，一般不用设）
  --demo [P1|PTV1]          不启动广播，只看演示数据（不写型号时按 --product，没给就是 PTV1）；不需要 SDK
  --log 文件                广播端日志（默认 ~/.local/state/revohuman/sim_live.log，每次追加）
  --venv 目录|none          SDK 所在的虚拟环境；none：直接用当前终端的 python3
  -h, --help                显示本帮助

其余参数原样交给可视化（全部参数见 run_viewer.sh --help），例如 --lite、--view cart、
--headless --seconds 10 --snapshot out.png；-- 之后的参数交给 revohuman-live（见 revohuman-live --help），
例如 -- --tracker-gate error。

SDK 按以下顺序找：--venv → $REVOHUMAN_VENV 或 ~/.venvs/revohuman_sdk（存在才用）→ 当前终端的 python3。
都找不到时提示先运行源码包根目录的 install_venv.sh。
EOF
}

say()  { printf '[run_sim_host] %s\n' "$*"; }
warn() { printf '[run_sim_host] 注意：%s\n' "$*" >&2; }
die()  { printf '[run_sim_host] %s\n' "$*" >&2; exit 2; }

while [[ $# -gt 0 ]]; do
    case "$1" in
        --) shift; LIVE_ARGS=("$@"); break ;;
        --hands|--product|--socket|--log|--venv|--pidfile)
            [[ $# -ge 2 ]] || die "$1 需要参数"
            case "$1" in
                --hands) HANDS="$2" ;;
                --product) PRODUCT="$2" ;;
                --socket) SOCKET="$2" ;;
                --log) LOG="$2" ;;
                --venv) VENV_OPT="$2" ;;
                --pidfile) PIDFILE="$2" ;;   # 给 run_sim_docker.sh 用：记下本脚本的 pid，主机那边断开时据此停止
            esac
            shift 2 ;;
        --hands=*|--product=*|--socket=*|--log=*|--venv=*|--pidfile=*) set -- "${1%%=*}" "${1#*=}" "${@:2}" ;;
        --no-tracker) NO_TRACKER=1; shift ;;
        --demo) DEMO=default; shift
                if [[ "${1:-}" == P1 || "${1:-}" == PTV1 ]]; then DEMO="$1"; shift; fi ;;
        --demo=*) DEMO="${1#*=}"; shift ;;
        -h|--help) usage; exit 0 ;;
        *) VIEWER_ARGS+=("$1"); shift ;;
    esac
done
case "${HANDS}" in ""|both|left|right) ;; *) die "--hands 只能是 both、left 或 right，收到的是「${HANDS}」" ;; esac
case "${PRODUCT}" in ""|auto|P1|PTV1) ;; *) die "--product 只能是 auto、P1 或 PTV1，收到的是「${PRODUCT}」" ;; esac
case "${DEMO}" in ""|default|P1|PTV1) ;; *) die "--demo 只能是 P1 或 PTV1，收到的是「${DEMO}」" ;; esac
[[ "${WAIT}" =~ ^[0-9]+$ ]] || die "REVOHUMAN_SIM_WAIT 要是整数秒，收到的是「${WAIT}」"
[[ -x "${RUN_VIEWER}" ]] || die "找不到 ${RUN_VIEWER}（源码包不完整？）"

# ---------- 进程管理 ----------
VIEWER_PID=""
LIVE_PID=""
WATCH_PID=""
alive() {   # 进程还在（已退出、等着被回收的不算）
    local st
    [[ -n "$1" && -r "/proc/$1/stat" ]] || return 1
    st="$(sed -E 's/^.*\) ([A-Za-z]).*$/\1/' "/proc/$1/stat" 2>/dev/null)" || return 1
    [[ -n "${st}" && "${st}" != Z && "${st}" != X ]]
}

stop_pid() {   # stop_pid pid 名称：先 TERM，等 STOP_WAIT 秒，还在就 KILL
    local pid="$1" i
    alive "${pid}" || return 0
    kill -TERM "${pid}" 2>/dev/null || return 0
    for (( i = 0; i < STOP_WAIT * 10; i++ )); do
        alive "${pid}" || return 0
        sleep 0.1
    done
    warn "$2（pid ${pid}）${STOP_WAIT} 秒内没有退出，强制结束"
    kill -KILL "${pid}" 2>/dev/null || true
}

cleanup() {
    local rc=$?
    trap - EXIT INT TERM HUP
    [[ -z "${WATCH_PID}" ]] || kill "${WATCH_PID}" 2>/dev/null || true
    [[ -z "${VIEWER_PID}" ]] || stop_pid "${VIEWER_PID}" 可视化
    if [[ -n "${LIVE_PID}" ]] && alive "${LIVE_PID}"; then
        say "停止广播（pid ${LIVE_PID}）"
        stop_pid "${LIVE_PID}" 广播
    fi
    if [[ -n "${PIDFILE}" && "$(cat "${PIDFILE}" 2>/dev/null)" == "$$" ]]; then rm -f "${PIDFILE}"; fi
    exit "${rc}"
}
trap cleanup EXIT
trap 'exit 130' INT
trap 'exit 143' TERM
trap 'exit 129' HUP
if [[ -n "${PIDFILE}" ]]; then
    mkdir -p "$(dirname "${PIDFILE}")" && echo "$$" >"${PIDFILE}" || warn "写不了 ${PIDFILE}"
fi

# 可视化放在后台运行、本脚本等它退出：这样收到 TERM / HUP（例如关掉终端）时能立刻停下可视化与广播
run_viewer() {
    "${RUN_VIEWER}" "$@" &
    VIEWER_PID=$!
    local rc=0
    wait "${VIEWER_PID}" || rc=$?
    VIEWER_PID=""
    return "${rc}"
}

# ---------- 只看演示 ----------
if [[ -n "${DEMO}" ]]; then
    if [[ "${DEMO}" == default ]]; then
        DEMO=PTV1
        [[ -z "${PRODUCT}" || "${PRODUCT}" == auto ]] || DEMO="${PRODUCT}"
    fi
    [[ ${#LIVE_ARGS[@]} == 0 ]] || warn "--demo 不启动广播，-- 之后的参数（${LIVE_ARGS[*]}）不起作用"
    args=(--demo "${DEMO}")
    [[ -z "${HANDS}" ]] || args+=(--demo-hands "${HANDS}")
    [[ ${NO_TRACKER} == 0 ]] || args+=(--tracker off)
    # 演示不用 SDK；有 install_venv.sh 建的虚拟环境就顺便用它跑可视化（依赖已装好），否则用 simulation/.venv
    sdk_venv="${REVOHUMAN_VENV:-${HOME}/.venvs/revohuman_sdk}"
    [[ -z "${VENV_OPT}" || "${VENV_OPT}" == none ]] || sdk_venv="${VENV_OPT}"
    if [[ -z "${REVOHUMAN_SIM_VENV:-}" && "${VENV_OPT}" != none && -d "${sdk_venv}/.revohuman" && -x "${sdk_venv}/bin/python" ]]; then
        export REVOHUMAN_SIM_VENV="${sdk_venv}"
    fi
    rc=0; run_viewer "${args[@]}" "${VIEWER_ARGS[@]}" || rc=$?
    exit "${rc}"
fi

# ---------- 已经有广播在运行？ ----------
PROBE_PY="$(command -v python3 || true)"

# 连上广播读 hello：打印「pid 型号 手 socket」；没有广播（或不是 RevoHuman 的广播）返回 1。
# socket 的默认位置与 SDK 的 revohuman.live.default_socket_path() 同一规则
probe() {
    [[ -n "${PROBE_PY}" ]] || return 1
    "${PROBE_PY}" - "${SOCKET}" <<'PY' 2>/dev/null
import json, os, socket, struct, sys
path = sys.argv[1] or os.environ.get("REVOHUMAN_LIVE_SOCKET")
if not path:
    run = os.environ.get("XDG_RUNTIME_DIR")
    if run and os.path.isdir(run) and os.access(run, os.W_OK | os.X_OK):
        path = os.path.join(run, "revohuman", "live.sock")
    else:
        path = os.path.join("/tmp", f"revohuman-{os.getuid()}", "live.sock")
path = os.path.abspath(path)
s = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
s.settimeout(3.0)
try:
    s.connect(path)
    buf = b""
    while len(buf) < 4:
        chunk = s.recv(4 - len(buf))
        if not chunk:
            sys.exit(1)
        buf += chunk
    n = struct.unpack(">I", buf)[0]
    if n > (16 << 20):
        sys.exit(1)
    body = b""
    while len(body) < n:
        chunk = s.recv(min(65536, n - len(body)))
        if not chunk:
            sys.exit(1)
        body += chunk
    hello = json.loads(body)
except (OSError, ValueError):
    sys.exit(1)
if not isinstance(hello, dict) or hello.get("type") != "hello":
    sys.exit(1)
print(hello.get("pid", "?"), (hello.get("product") or {}).get("code", "?"),
      ",".join(hello.get("hands") or []) or "-", path)
PY
}

viewer_args() {
    VARGS=()
    [[ -z "${SOCKET}" ]] || VARGS+=(--socket "${SOCKET}")
    [[ ${NO_TRACKER} == 0 ]] || VARGS+=(--tracker off)
    VARGS+=("${VIEWER_ARGS[@]}")
}

if info="$(probe)"; then
    read -r b_pid b_code b_hands b_path <<<"${info}"
    say "检测到已有后端在运行（pid ${b_pid}，型号 ${b_code}，手 ${b_hands}，socket ${b_path}），只启动可视化"
    [[ ${#LIVE_ARGS[@]} == 0 && -z "${HANDS}${PRODUCT}" ]] \
        || warn "--hands、--product 与 -- 之后的参数只在启动广播时起作用，这次不起作用；要换设置先停掉那个广播"
    say "关闭窗口不会停止那个广播"
    viewer_args
    rc=0; run_viewer "${VARGS[@]}" || rc=$?
    exit "${rc}"
fi

# ---------- 找 SDK ----------
INSTALLER="${ROOT_DIR}/install_venv.sh"
not_found() {
    die "$1
    先运行源码包根目录的 install_venv.sh 把 SDK 装进虚拟环境：
        ${INSTALLER}
    已经装在别的虚拟环境里时用 --venv 指定它"
}
sdk_ok() { "$1" -c 'import revohuman' >/dev/null 2>&1; }

SDK_PY=""
SIM_VENV=""
if [[ "${VENV_OPT}" == none ]]; then
    SDK_PY="$(command -v python3 || true)"
    [[ -n "${SDK_PY}" ]] && sdk_ok "${SDK_PY}" || not_found "--venv none：当前终端的 python3${SDK_PY:+（${SDK_PY}）}导入不了 revohuman"
elif [[ -n "${VENV_OPT}" ]]; then
    [[ -x "${VENV_OPT}/bin/python" ]] || not_found "--venv ${VENV_OPT} 不是虚拟环境（里面没有 bin/python）"
    sdk_ok "${VENV_OPT}/bin/python" || not_found "--venv ${VENV_OPT} 里导入不了 revohuman"
    SDK_PY="${VENV_OPT}/bin/python"; SIM_VENV="${VENV_OPT}"
else
    v="${REVOHUMAN_VENV:-${HOME}/.venvs/revohuman_sdk}"
    if [[ -x "${v}/bin/python" ]]; then
        sdk_ok "${v}/bin/python" || not_found "虚拟环境 ${v} 里导入不了 revohuman（没装完？重新执行 install_venv.sh 即可补齐）"
        SDK_PY="${v}/bin/python"; SIM_VENV="${v}"
    elif SDK_PY="$(command -v python3)" && sdk_ok "${SDK_PY}"; then
        say "没有找到虚拟环境 ${v}，用当前终端的 python3（${SDK_PY}）"
    else
        not_found "没有找到 RevoHuman SDK：虚拟环境 ${v} 不存在，当前终端的 python3 也导入不了 revohuman"
    fi
fi
# install_venv.sh 建的虚拟环境里已经装好了可视化的依赖，可视化也用它；其他环境不往里装东西，可视化用 simulation/.venv
if [[ -n "${SIM_VENV}" && -z "${REVOHUMAN_SIM_VENV:-}" && -d "${SIM_VENV}/.revohuman" ]]; then
    export REVOHUMAN_SIM_VENV="${SIM_VENV}"
fi

# ---------- 后台启动广播 ----------
[[ "${LOG}" == /* ]] || LOG="${PWD}/${LOG}"
mkdir -p "$(dirname "${LOG}")" || die "建不了日志目录 $(dirname "${LOG}")"
touch "${LOG}" 2>/dev/null || die "写不了日志 ${LOG}"
LIVE_CMD=("${SDK_PY}" -m revohuman live)
[[ -z "${HANDS}" ]] || LIVE_CMD+=(--hands "${HANDS}")
[[ -z "${PRODUCT}" ]] || LIVE_CMD+=(--product "${PRODUCT}")
[[ ${NO_TRACKER} == 0 ]] || LIVE_CMD+=(--no-tracker)
[[ -z "${SOCKET}" ]] || LIVE_CMD+=(--socket "${SOCKET}")
LIVE_CMD+=("${LIVE_ARGS[@]}")
printf '\n===== %s run_sim_host.sh 启动广播：%s =====\n' "$(date '+%F %T')" "${LIVE_CMD[*]}" >>"${LOG}"
LOG_OFF="$(stat -c %s "${LOG}")"
# 放进单独的会话：终端里的 Ctrl+C 只到可视化与本脚本，广播由本脚本按顺序停下
SETSID=(); command -v setsid >/dev/null && SETSID=(setsid)
PYTHONUNBUFFERED=1 "${SETSID[@]}" "${LIVE_CMD[@]}" >>"${LOG}" 2>&1 </dev/null &
LIVE_PID=$!
say "后台启动广播（pid ${LIVE_PID}），日志：${LOG}"
if [[ ${NO_TRACKER} == 1 ]]; then
    say "等广播就绪（只开手套，几秒）…"
else
    say "等广播就绪（要等基站数据与 Tracker 的首帧位姿，约半分钟）…"
fi

# 把日志里新增的行转给终端，用户能看到启动进度
shown="${LOG_OFF}"
relay() {
    local size
    size="$(stat -c %s "${LOG}" 2>/dev/null || echo "${shown}")"
    (( size > shown )) || return 0
    tail -c "+$((shown + 1))" "${LOG}" | head -c "$((size - shown))" | sed 's/^/    /'
    shown="${size}"
}
log_tail() { tail -c "+$((LOG_OFF + 1))" "${LOG}" | tail -n "${1:-20}" | sed 's/^/    /'; }

start="${SECONDS}"
ready=0
while :; do
    if alive "${LIVE_PID}" && info="$(probe)"; then
        read -r b_pid b_code b_hands b_path <<<"${info}"
        if [[ "${b_pid}" == "${LIVE_PID}" ]]; then ready=1; break; fi
    fi
    relay
    if ! alive "${LIVE_PID}"; then
        rc=0; wait "${LIVE_PID}" || rc=$?
        LIVE_PID=""
        relay
        printf '[run_sim_host] 广播没能启动（退出码 %s）。日志 %s 的最后几行：\n' "${rc}" "${LOG}" >&2
        log_tail 15 >&2
        exit 1
    fi
    if (( SECONDS - start >= WAIT )); then
        warn "等了 ${WAIT} 秒广播还没就绪，先打开窗口（广播就绪后窗口自动连上；进度见 ${LOG}）"
        break
    fi
    sleep 0.5
done
relay
[[ ${ready} == 0 ]] || say "广播已就绪（型号 ${b_code}，手 ${b_hands}，socket ${b_path}）"

# 看窗口期间广播意外退出时，在终端提示一次（窗口会一直等它）
(
    while alive "${LIVE_PID}"; do sleep 1 </dev/null >/dev/null 2>&1; done
    printf '\n[run_sim_host] 广播已退出，窗口不再更新。日志 %s 的最后几行：\n' "${LOG}" >&2
    log_tail 10 >&2
) &
WATCH_PID=$!

viewer_args
rc=0; run_viewer "${VARGS[@]}" || rc=$?
exit "${rc}"
