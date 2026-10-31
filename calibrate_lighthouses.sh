#!/usr/bin/env bash
# 一条命令做完基站几何标定：核对白名单 → 认 Tracker → 扫描基站（刷新信道预检）→ 提示怎么走 → 标定 → 启动前自查。
# 只从基站读取信息，不向基站写入；有信道冲突、白名单没填好时停下，提示怎么处理。
#
#   ./calibrate_lighthouses.sh                    在主机上，用 install_venv.sh 装的 SDK
#   ./calibrate_lighthouses.sh --hands right      只用右手的 Tracker
#   ./calibrate_lighthouses.sh --docker           在主机上执行，进 RevoHuman 容器里标定（容器带蓝牙创建）
#   ./calibrate_lighthouses.sh -- --coverage-cells 4   -- 之后的参数交给 python3 -m revohuman tracker calibrate
#
# 已经在容器里时直接执行本脚本（容器要用 docker/run.sh --ble 创建）。
# 完整说明见使用手册《03-基站使用与配对》第 5.4 节与《08-容器运行》第 7.3 节。
set -eo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
SELF="${ROOT_DIR}/$(basename "${BASH_SOURCE[0]}")"
STOP_WAIT=30           # 停止标定时最多等它回滚多少秒
# 等回车超过这么多秒就在标定前重扫一次（信道预检 5 分钟有效，留出余量）；环境变量只给测试用
PREFLIGHT_REUSE="${REVOHUMAN_CALIB_RESCAN_SEC:-240}"
VENV_OPT=""
HANDS=""
WHITELIST=""
STATE_DIR_OPT=""
CONFIG=""
SCAN_SECONDS=""
TIMEOUT=""
NO_CHECK=0
YES=0
PIDFILE=""
DOCKER=0
TRACKERS=()
CAL_EXTRA=()
# --docker：交给 docker/run.sh 的选项
IMAGE=""
NAME="revohuman"
RESTART=0
OPTS=()
RUN_ARGS=()
INNER_ARGS=()

usage() {
    cat <<'EOF'
用法：calibrate_lighthouses.sh [选项] [-- tracker calibrate 的参数…]

一条命令做完基站几何标定（《03-基站使用与配对》第 4.4–6 节）：
  1. 核对基站白名单；还没有时扫描附近的基站、按示例写出白名单，提示填好后重跑
  2. 认出要用的 Tracker（没给 --tracker 时按手套自动找）；一只都认不出时停下，提示怎么处理
  3. 扫描基站、刷新信道预检；有站没扫到、同信道冲突时停下，提示怎么处理
  4. 打印怎么走，回车后开始标定（失败、中断都会回滚到标定前）
  5. 打开一次 Tracker 做启动前自查（check --live），再打印当前状态
本脚本只从基站读取信息，不向基站写入。

选项：
  --hands both|left|right   用哪几只手的 Tracker（默认 both）
  --tracker 手=序列号       直接指定 Tracker，如 --tracker left=LHR-XXXXXXXX，可重复（默认按手套自动找）
  --whitelist 文件          基站白名单配置文件（默认按《03》第 3 节的顺序找）
  --state-dir 目录          Tracker 状态目录
  --config 文件             libsurvive 的 config.json
  --scan-seconds 秒         扫描基站的时长（默认 10）
  --timeout 秒              标定最长多少秒（默认 300）
  --no-check                标定完成后不做启动前自查
  -y, --yes                 不等回车，扫描完立即开始标定
  --venv 目录|none          SDK 所在的虚拟环境；none：直接用当前终端的 python3
  -h, --help                显示本帮助

在主机上执行、在 RevoHuman 容器里标定：
  --docker                  进入容器（没在运行时由 docker/run.sh 带 --ble 创建）后在容器里执行本脚本
  --image 镜像、--name 容器名、--restart、--data 目录、--state 目录、-e 名=值
                            与 --docker 一起用，原样交给 docker/run.sh；没给的 --data、--state、-e 沿用原来
                            容器的设置。容器不带蓝牙时加 --restart 重建（原来加过的选项要一并写上）

-- 之后的参数原样交给 python3 -m revohuman tracker calibrate（见它的 --help），例如 -- --coverage-cells 4。

SDK 按以下顺序找：--venv → $REVOHUMAN_VENV 或 ~/.venvs/revohuman_sdk（存在才用）→ 当前终端的 python3
（要导入得了 revohuman 与 Tracker 驱动 pysurvive）。还没装 SDK 时先运行源码包根目录的 install_venv.sh（默认装进 ~/.venvs/revohuman_sdk）。

退出码：0 完成；1 标定失败（已回滚）或自查没通过；2 用法有误、找不到手套或 Tracker，或环境、蓝牙、SDK 有问题；
3 标定被拒（标定前的条件不满足，没有开始）；4 要先处理白名单或基站（见打印的提示），再重跑；
130 / 143 / 129 被 Ctrl-C、kill、关闭终端打断（标定没完成的已回滚到标定前）。
EOF
}

say()  { printf '[calibrate_lighthouses] %s\n' "$*"; }
warn() { printf '[calibrate_lighthouses] 注意：%s\n' "$*" >&2; }
die()  { printf '[calibrate_lighthouses] %s\n' "$*" >&2; exit 2; }
need() { [[ $# -ge 2 ]] || die "$1 需要参数"; }

while [[ $# -gt 0 ]]; do
    case "$1" in
        --) shift; CAL_EXTRA=("$@"); INNER_ARGS+=(-- "$@"); break ;;
        --hands|--tracker|--whitelist|--state-dir|--config|--scan-seconds|--timeout)
            need "$@"
            case "$1" in
                --hands) HANDS="$2" ;;
                --tracker) TRACKERS+=(--tracker "$2") ;;
                --whitelist) WHITELIST="$2" ;;
                --state-dir) STATE_DIR_OPT="$2" ;;
                --config) CONFIG="$2" ;;
                --scan-seconds) SCAN_SECONDS="$2" ;;
                --timeout) TIMEOUT="$2" ;;
            esac
            INNER_ARGS+=("$1" "$2"); shift 2 ;;
        --venv|--pidfile)
            need "$@"
            case "$1" in
                --venv) VENV_OPT="$2" ;;
                --pidfile) PIDFILE="$2" ;;   # 给 --docker 用：记下容器里本脚本的 pid，主机那边断开时据此停止
            esac
            shift 2 ;;
        --image|--name|--data|--state|-e|--env)
            need "$@"
            case "$1" in
                --image) IMAGE="$2" ;;
                --name) NAME="$2" ;;
                *) OPTS+=("$1" "$2") ;;
            esac
            RUN_ARGS+=("$1" "$2"); shift 2 ;;
        --hands=*|--tracker=*|--whitelist=*|--state-dir=*|--config=*|--scan-seconds=*|\
        --timeout=*|--venv=*|--pidfile=*|--image=*|--name=*|--data=*|--state=*|--env=*)
            set -- "${1%%=*}" "${1#*=}" "${@:2}" ;;
        --restart) RESTART=1; RUN_ARGS+=("$1"); shift ;;
        --docker) DOCKER=1; shift ;;
        --no-check) NO_CHECK=1; INNER_ARGS+=("$1"); shift ;;
        -y|--yes) YES=1; INNER_ARGS+=("$1"); shift ;;
        -h|--help) usage; exit 0 ;;
        *) die "不认识的参数「$1」（tracker calibrate 的参数写在 -- 之后；全部选项见 --help）" ;;
    esac
done
case "${HANDS}" in ""|both|left|right) ;; *) die "--hands 只能是 both、left 或 right，收到的是「${HANDS}」" ;; esac
for v in SCAN_SECONDS TIMEOUT; do
    [[ -z "${!v}" || "${!v}" =~ ^[0-9]+([.][0-9]+)?$ ]] || die "--$(tr 'A-Z_' 'a-z-' <<<"${v}") 要是秒数，收到的是「${!v}」"
done
IN_CONTAINER=0
[[ ! -f /.dockerenv && -z "${container:-}" ]] || IN_CONTAINER=1
if [[ ${DOCKER} == 0 && ${#RUN_ARGS[@]} != 0 ]]; then
    die "--image、--name、--restart、--data、--state、-e 只和 --docker 一起用"
fi

# ======================================================================
# --docker：在主机上进入容器，在容器里执行本脚本
# ======================================================================
if [[ ${DOCKER} == 1 ]]; then
    [[ ${IN_CONTAINER} == 0 ]] || die "已经在容器里了，去掉 --docker 及 --image、--name、--restart、--data、--state、-e 直接执行本脚本"
    [[ -z "${VENV_OPT}" ]] || die "容器里固定使用镜像自带的 SDK，不接受 --venv"
    RUN_SH="${ROOT_DIR}/docker/run.sh"
    command -v docker >/dev/null || die "找不到 docker 命令：先按手册《08-容器运行》安装 Docker 并构建镜像（docker/install.sh）"
    [[ -x "${RUN_SH}" ]] || die "找不到 ${RUN_SH}（源码包不完整？）"
    RUN_ARGS+=(--ble)
    OPTS=(--ble "${OPTS[@]}")
    AGAIN="${SELF} --docker --restart"
    [[ ${NAME} == revohuman ]] || AGAIN+=" --name ${NAME}"
    [[ -z "${IMAGE}" ]] || AGAIN+=" --image ${IMAGE}"
    [[ ${#OPTS[@]} == 1 ]] || AGAIN+="$(printf ' %q' "${OPTS[@]:1}")"
    if [[ ${RESTART} == 0 ]]; then
        HINT="docker/run.sh 这次没有把容器换成新镜像（原因见上面 docker/run.sh 的提示，多半是容器里还有其他程序在运行）：等它们都退出后重新执行本命令，会自动换；要现在就换，执行 ${AGAIN} 重建容器（那些程序会被结束；创建容器时加过 --data、-e 等选项的要一并写上，docker/run.sh 的提示里有完整命令）。重建之后仍然这样，先用主机上这份源码包的 docker/install.sh 重新构建镜像"
    else
        HINT="容器刚重建过，是镜像${IMAGE:+ ${IMAGE} }本身旧：用主机上这份源码包的 docker/install.sh 重新构建镜像，再执行 ${AGAIN}"
    fi
    TOKEN="$$-${RANDOM}${RANDOM}"
    # 找到镜像里源码包的本脚本，记下它的 pid（放在挂载到主机的状态目录里），主机这边被打断时据此停止它
    INNER='tok=$1; hint=$2; shift 2
shopt -s nullglob
c=(~/revohuman-release/revohuman-*/calibrate_lighthouses.sh)
if [[ ${#c[@]} != 1 ]]; then
    v=(~/revohuman-release/revohuman-*/); v=("${v[@]%/}"); v=("${v[@]##*/}")
    echo "[calibrate_lighthouses] 容器里的源码包（${v[*]:-无}）没有 calibrate_lighthouses.sh，容器里还是旧版本的程序：" >&2
    echo "[calibrate_lighthouses] ${hint}" >&2
    exit 2
fi
exec "${c[0]}" --venv none --pidfile "$HOME/.local/state/revohuman/calibrate_docker.$tok.pid" "$@"'

    STOPPED=0
    stop_inner() {   # 停止容器里本次启动的脚本（它会接着停下标定，标定回滚到标定前）
        [[ ${STOPPED} == 0 ]] || return 0
        STOPPED=1
        docker exec "${NAME}" bash -c 'f="$HOME/.local/state/revohuman/calibrate_docker.$1.pid"
            p="$(cat "$f" 2>/dev/null)" || exit 0
            if [[ "$p" =~ ^[0-9]+$ ]] && tr "\0" " " <"/proc/$p/cmdline" 2>/dev/null | grep -q calibrate_lighthouses; then
                kill -TERM "$p" 2>/dev/null
                for i in $(seq 1 '"$((STOP_WAIT * 10))"'); do [[ -e /proc/$p ]] || break; sleep 0.1; done
            fi
            rm -f "$f"' _ "${TOKEN}" >/dev/null 2>&1 || true
    }
    CHILD=""
    on_signal() {
        trap - INT TERM HUP
        { say "正在停止容器 ${NAME} 里的标定（会回滚到标定前，最多约 ${STOP_WAIT} 秒）…"; } 2>/dev/null || true
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

    [[ -z "${IMAGE}" ]] || say "镜像：${IMAGE}"
    # docker/run.sh 放到后台再等它：收到信号能立刻处理（docker exec 不把信号转进容器）；
    # <&0 保留终端输入，run.sh 才会给容器分配终端，回车确认与 Ctrl+C 才能直接送进容器
    "${RUN_SH}" "${RUN_ARGS[@]}" -- bash -c "${INNER}" calibrate_lighthouses "${TOKEN}" "${HINT}" "${INNER_ARGS[@]}" <&0 &
    CHILD=$!
    rc=0
    wait "${CHILD}" || rc=$?
    CHILD=""
    if [[ ${rc} == 4 ]]; then
        state_root="${XDG_DATA_HOME:-${HOME}/.local/share}/revohuman-docker"
        for (( i = 0; i < ${#OPTS[@]}; i++ )); do [[ "${OPTS[i]}" != --state ]] || state_root="${OPTS[i+1]}"; done
        say "容器里的 ~/.config/revohuman/ 就是主机上的 $(realpath -m "${state_root}")/config/revohuman/，在主机上编辑白名单即可"
    fi
    exit "${rc}"
fi

# ======================================================================
# 在本机（主机或容器里）标定
# ======================================================================
CAL_PID=""
INTERRUPTED=""
cleanup() {
    local rc=$?
    trap - EXIT
    if [[ -n "${PIDFILE}" && "$(cat "${PIDFILE}" 2>/dev/null)" == "$$" ]]; then rm -f "${PIDFILE}"; fi
    exit "${rc}"
}
# 标定进程放在单独的进程组里（终端的 Ctrl+C 只到本脚本），由本脚本转成 SIGINT 交给它并等它回滚完
on_signal() {
    INTERRUPTED="$1"
    if [[ -n "${CAL_PID}" ]]; then
        local cal_rc=0
        kill -INT "${CAL_PID}" 2>/dev/null || true
        trap '' INT TERM HUP
        { say "正在停止标定，回滚到标定前…"; } 2>/dev/null || true
        wait "${CAL_PID}" 2>/dev/null || cal_rc=$?
        CAL_PID=""
        if [[ ${cal_rc} == 0 ]]; then   # 中断到达时标定刚好完成：几何已经更新，不是回滚
            { say "标定在停下之前已经完成，基站几何已更新；没做启动前自查（python3 -m revohuman tracker check --live）"; } 2>/dev/null || true
        fi
    fi
    exit "$1"
}
trap cleanup EXIT
trap 'on_signal 130' INT
trap 'on_signal 143' TERM
trap 'on_signal 129' HUP
if [[ -n "${PIDFILE}" ]]; then
    mkdir -p "$(dirname "${PIDFILE}")" && echo "$$" >"${PIDFILE}" || warn "写不了 ${PIDFILE}"
fi

# ---------- 找 SDK ----------
INSTALLER="${ROOT_DIR}/install_venv.sh"
not_found() {
    die "$1
    先运行源码包根目录的 install_venv.sh 把 SDK 装进虚拟环境：
        ${INSTALLER}
    已经装在别的虚拟环境里时用 --venv 指定它"
}
# 能跑标定的 python：导入得了 revohuman 与 Tracker 驱动 pysurvive。在 / 下试，免得当前目录正好是源码树时
# 把目录里的 revohuman 源码当成装好了的 SDK（那样扫描能跑，到标定才因缺 pysurvive 等依赖失败）
sdk_ok() { (cd / && "$1" -c 'import revohuman, pysurvive') >/dev/null 2>&1; }
PY=""
if [[ "${VENV_OPT}" == none ]]; then
    PY="$(command -v python3 || true)"
    [[ -n "${PY}" ]] && sdk_ok "${PY}" || not_found "--venv none：当前终端的 python3${PY:+（${PY}）}导入不了 revohuman 与 Tracker 驱动 pysurvive"
elif [[ -n "${VENV_OPT}" ]]; then
    [[ -x "${VENV_OPT}/bin/python" ]] || not_found "--venv ${VENV_OPT} 不是虚拟环境（里面没有 bin/python）"
    sdk_ok "${VENV_OPT}/bin/python" || not_found "--venv ${VENV_OPT} 里导入不了 revohuman 与 Tracker 驱动 pysurvive"
    PY="${VENV_OPT}/bin/python"
else
    v="${REVOHUMAN_VENV:-${HOME}/.venvs/revohuman_sdk}"
    if [[ -x "${v}/bin/python" ]]; then
        sdk_ok "${v}/bin/python" || not_found "虚拟环境 ${v} 里导入不了 revohuman 与 Tracker 驱动 pysurvive（没装完？重新执行 install_venv.sh 即可补齐）"
        PY="${v}/bin/python"
    elif PY="$(command -v python3)" && sdk_ok "${PY}"; then
        [[ ${IN_CONTAINER} == 1 ]] || say "没有找到虚拟环境 ${v}，用当前终端的 python3（${PY}）"
    else
        not_found "没有找到 RevoHuman SDK：虚拟环境 ${v} 不存在，当前终端的 python3${PY:+（${PY}）}也导入不了 revohuman 与 Tracker 驱动 pysurvive
    （在源码目录里运行时，目录里的 revohuman 源码不算装好的 SDK）"
    fi
fi

# 各步共用的参数
LH_ARGS=()
[[ -z "${WHITELIST}" ]] || LH_ARGS+=(--whitelist "${WHITELIST}")
[[ -z "${STATE_DIR_OPT}" ]] || LH_ARGS+=(--state-dir "${STATE_DIR_OPT}")
CFG_ARGS=("${LH_ARGS[@]}")
[[ -z "${CONFIG}" ]] || CFG_ARGS+=(--config "${CONFIG}")
ROLE_ARGS=("${TRACKERS[@]}")
[[ -z "${HANDS}" ]] || ROLE_ARGS+=(--hands "${HANDS}")
SCAN_ARGS=("${LH_ARGS[@]}")
[[ -z "${SCAN_SECONDS}" ]] || SCAN_ARGS+=(--seconds "${SCAN_SECONDS}")
lh() { "${PY}" -m revohuman tracker lighthouses "$@"; }

ble_hint() {   # 扫描报蓝牙不可用（退出码 2）时
    if [[ ${IN_CONTAINER} == 1 && ! -S /run/dbus/system_bus_socket ]]; then
        printf '[calibrate_lighthouses] 容器没有接主机的蓝牙。在主机上执行（容器里没有正在运行的程序时；创建容器时加过 --data、-e 等选项的要一并写上）：\n    %s\n' \
            "${ROOT_DIR}/calibrate_lighthouses.sh --docker --restart" >&2
        printf '[calibrate_lighthouses] 或 docker/run.sh --restart --ble 重建容器后再进来执行本脚本（《08-容器运行》第 7.1 节）\n' >&2
    elif [[ ${IN_CONTAINER} == 1 ]]; then
        printf '[calibrate_lighthouses] 在主机上确认蓝牙服务在运行、适配器已打开（《08-容器运行》第 7.1 节）；另一个程序正在用蓝牙时等它结束再重跑\n' >&2
    else
        printf '[calibrate_lighthouses] 确认蓝牙服务在运行、适配器已打开（《03-基站使用与配对》第 2 节）：\n' >&2
        printf '    systemctl is-active bluetooth\n    bluetoothctl show | grep Powered\n' >&2
        printf '[calibrate_lighthouses] 另一个程序正在用蓝牙时，等它结束再重跑\n' >&2
    fi
    exit 2
}

SCAN_AT=0
scan() {   # 扫描基站、刷新信道预检；返回 scan 的退出码（2 时直接退出）
    local rc=0
    say "扫描附近的基站（约 ${SCAN_SECONDS:-10} 秒）…"
    lh scan "${SCAN_ARGS[@]}" </dev/null || rc=$?
    [[ ${rc} != 2 ]] || ble_hint
    [[ ${rc} != 0 ]] || SCAN_AT=${SECONDS}
    return "${rc}"
}

# ---------- 1. 白名单 ----------
WL_KIND=""
WL_PATH=""
wl_state() {   # 白名单现在的状态（show --json 最后一行的 kind，没问题时为 ok）与本次生效的文件
    local out res
    out="$(lh show --json "${LH_ARGS[@]}" 2>/dev/null </dev/null)" || true
    res="$("${PY}" -c '
import json, sys
for line in reversed(sys.stdin.read().splitlines()):
    if line.startswith("{"):
        res = json.loads(line)
        print(res.get("kind") or "ok", res.get("path") or "", sep="\t")
        break
else:
    print("error\t")
' <<<"${out}")" || res=$'error\t'
    WL_KIND="${res%%$'\t'*}"
    WL_PATH="${res#*$'\t'}"
}
wl_state
case "${WL_KIND}" in
    whitelist_missing|whitelist_invalid)
        lh show "${LH_ARGS[@]}" </dev/null || true
        scan || true
        if [[ ${WL_KIND} == whitelist_missing ]]; then
            lh init "${LH_ARGS[@]}" </dev/null || true
            wl_state    # 从源码树运行时 init 写的是仓库根的 revohuman_tracker.yaml，以写出后生效的那份为准
        fi
        say "白名单${WL_PATH:+ ${WL_PATH} }还没填好：对照基站背面标签，把上面扫到的、要用的基站序列号（FA 开头的 12 位）填进去，"
        say "  填法见《03-基站使用与配对》第 4.3 节；基站多、分不清哪台是哪台时先按第 4.2 节让基站闪灯辨认。填好后重跑本脚本"
        exit 4 ;;
    ok|lighthouse_unresolved) ;;   # 还没解析出广播名的，扫描时会读出来
    *) lh show "${LH_ARGS[@]}" </dev/null || true
       die "读不了基站白名单（见上面的输出）" ;;
esac

# ---------- 2. 认 Tracker ----------
# 在扫描、按提示戴好手套之前就认：认不出要到标定那步才发现，人已经站好、走过一遍了。
# 只读 USB 描述符，不打开设备：按 MCU 序列号定手别，取同一 HUB 上的 Tracker（与标定时
# revohuman.identity.resolve_tracker_serial 同一规则）。
# 退出码：0 都认出；10 只认出一部分；11 一只都没认出；其他 = 预检本身出错，交给标定那步再认
ROLES_PY='# calibrate_lighthouses: roles
import sys
from revohuman import identity
from revohuman.frames import Handedness
hands = sys.argv[1]
want = [Handedness.LEFT, Handedness.RIGHT] if hands in ("", "both") else [Handedness(hands)]
units = identity.scan_units()
name = {Handedness.LEFT: "左手", Handedness.RIGHT: "右手"}
ok = 0
for hand in want:
    unit, _source, note = identity.locate_hand(hand, units=units)
    if unit is not None and unit.tracker_serial:
        ok += 1
        print(f"  {name[hand]}：Tracker {unit.tracker_serial}（按 MCU 序列号认出的手套）")
        continue
    if unit is not None:
        why = (f"{name[hand]}手套（单元 {unit.unit_id}）的 HUB 上没有 Tracker；"
               f"Tracker 接在无线接收器上时加 --tracker {hand.value}=<序列号> 指定")
    else:
        why = note or "手套不在位"
    print(f"  ✗ {name[hand]}：认不出 Tracker——{why}")
sys.exit(0 if ok == len(want) else 10 if ok else 11)
'
if [[ ${#TRACKERS[@]} == 0 ]]; then
    say "认 Tracker（只读 USB 信息，不打开设备）…"
    rc=0
    "${PY}" -c "${ROLES_PY}" "${HANDS}" </dev/null || rc=$?
    case "${rc}" in
        0) ;;
        10) say "只用认出来的那只 Tracker 标定；另一只按上面 ✗ 行的提示处理后重跑，或加 --hands 只用这一只" ;;
        11) say "没有认出可用的 Tracker，没有开始扫描和标定：按上面 ✗ 行的提示处理后重跑本脚本"
            exit 2 ;;
        *) warn "没能预先认 Tracker（退出码 ${rc}），到标定那步再认" ;;
    esac
fi

# ---------- 3. 扫描、刷新信道预检 ----------
if ! scan; then
    say "扫描没通过，没有开始标定。按上面 ✗ 行的提示处理后重跑本脚本："
    say "  有站没扫到：确认基站通电、离得不太远；同信道冲突：按《03-基站使用与配对》第 4.5 节改信道"
    say "  （revohuman-lighthouses plan 看方案、apply --yes 执行；本脚本不向基站写入）"
    exit 4
fi

# ---------- 4. 怎么走 ----------
cat <<'EOF'

接下来标定基站几何。开始前确认两只 Tracker 的指示灯都是绿色，戴好手套（Tracker 在手背上），
站到基站照得到的工作区里，面向基站：
  1. 先把双手左右分开，和肩同宽或更宽，手背朝向基站，停住约 1 秒；
  2. 换一个位置再停住约 1 秒：前后挪一步、双手抬高或放低；
  3. 照屏幕提示重复第 2 步，每换一处都停一下，不要一直晃动。
不要用身体、手臂挡住另一只手的 Tracker，也不要背对基站。每秒打印一行进度，→ 后面是下一步该做什么
（各条提示的含义见《03-基站使用与配对》第 5.2 节）。中途按 Ctrl+C 中止，会回滚到标定前。

EOF
if [[ ${YES} == 0 ]]; then
    [[ -t 0 ]] || die "没有终端可以确认：加 -y 不等回车直接开始"
    read -r -p "[calibrate_lighthouses] 准备好后按回车开始标定（Ctrl+C 取消）… " _ || die "没有读到回车，已取消"
    if (( SECONDS - SCAN_AT >= PREFLIGHT_REUSE )); then
        say "离上次扫描已超过 ${PREFLIGHT_REUSE} 秒，重扫一次刷新信道预检"
        scan || { say "重扫没通过，没有开始标定。按上面 ✗ 行的提示处理后重跑本脚本"; exit 4; }
    fi
fi

# ---------- 5. 标定 ----------
# 先把 SIGINT 恢复成默认再启动标定：本脚本是被后台启动的（nohup、别的脚本里加 &）时 SIGINT 被设成了忽略，
# 子进程照样继承，Python 就不会把 SIGINT 转成 KeyboardInterrupt，下面转交的 SIGINT 不起作用、不会回滚
CAL_CMD=("${PY}" -c 'import os, signal, sys; signal.signal(signal.SIGINT, signal.SIG_DFL); os.execv(sys.argv[1], sys.argv[1:])'
         "${PY}" -m revohuman tracker calibrate "${ROLE_ARGS[@]}" "${CFG_ARGS[@]}")
[[ -z "${TIMEOUT}" ]] || CAL_CMD+=(--timeout "${TIMEOUT}")
CAL_CMD+=("${CAL_EXTRA[@]}")
say "开始标定"
set -m
"${CAL_CMD[@]}" </dev/null &
CAL_PID=$!
set +m
rc=0
wait "${CAL_PID}" || rc=$?
CAL_PID=""
case "${rc}" in
    0) ;;
    2) say "没有开始标定：参数不对，或找不到要用的手套、Tracker（见上面的输出），改好后重跑本脚本"; exit 2 ;;
    3) say "标定被拒，没有开始标定：按上面的原因处理（见《03-基站使用与配对》第 5.2 节末尾的表格）后重跑本脚本"; exit 3 ;;
    *) say "标定没有完成（退出码 ${rc}），基站几何已回滚到标定前。按上面的失败原因处理后重跑本脚本"
       say "  （常见的是换个位置多走几处，或按《03》第 1.1 节「摆放环境」处理反光和强光）"
       exit 1 ;;
esac

# ---------- 6. 启动前自查 ----------
if [[ ${NO_CHECK} == 0 ]]; then
    say "启动前自查：再打开一次 Tracker，把基站与位姿实际验一遍（十几秒，最长约半分钟）…"
    rc=0
    "${PY}" -m revohuman tracker check --live "${ROLE_ARGS[@]}" "${CFG_ARGS[@]}" </dev/null || rc=$?
    if [[ ${rc} != 0 ]]; then
        say "标定已完成，但启动前自查没通过（退出码 ${rc}）：按上面 ✗ 行与「修复：」一节处理（《03-基站使用与配对》第 6 节）"
        exit 1
    fi
fi
echo
"${PY}" -m revohuman tracker status "${CFG_ARGS[@]}" </dev/null || true
echo
say "完成。零位不用另外记：录制等程序每次启动时会按双手当时的位姿自动记一次（《04-数据录制》第 1 节）"
say "挪过基站、改过信道之后要重新标定：重跑本脚本即可"
