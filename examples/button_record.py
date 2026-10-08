"""示例：用手套上的按键控制录制——按一下开始录一段，再按一下结束。

用法：python3 examples/button_record.py                 # 全机恰好插着一只时可裸连
     python3 examples/button_record.py right           # 按注册表手别连
     python3 examples/button_record.py right bench     # 注册表里同一手别声明在多组时，指定组名

Ctrl-C 退出（正在录的那段会先收完）。录制需 pip install 'revohuman[recording]'。

退出码：0 = 正常退出；1 = 连接或录制出错（stderr 上是带错误码的一行）；
       2 = 这只手套的固件不支持在采集期间上报按键。

按键默认不开启，本示例在连接时显式传 button=True。
按键状态由手套自己维护：每次有效按压在「开始」「结束」之间切换一次，上电为「结束」。
连接后第一个状态只作基线、不算按压；所以连接前按过键的话，第一下按压可能报「结束」，
再按一下即开始。
"""

import pathlib
import queue
import sys

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent.parent))  # 未安装时从仓库根跑

from revohuman import (ButtonMode, GloveManager, RevoHumanError, TopicRecorder, format_error,
                       set_log_level)

EXIT_UNSUPPORTED = 2


def main() -> None:
    set_log_level("INFO")
    hand = sys.argv[1] if len(sys.argv) > 1 else None
    set_name = sys.argv[2] if len(sys.argv) > 2 else None
    mgr = GloveManager()
    try:
        glove = (mgr.connect(handedness=hand, set_name=set_name, button=True) if hand
                 else mgr.connect(button=True))
    except RevoHumanError as exc:
        sys.exit(format_error(exc))
    with glove:
        # 按键是否可用在第一次开流后才知道（开流是异步的）：先等到第一帧
        frames = glove.subscribe()
        while True:
            try:
                frames.recv(timeout=1.0)
                break
            except queue.Empty:
                continue
        frames.close()
        if not glove.button.supported:
            print(f"{glove.port}：固件不支持在采集期间上报按键", file=sys.stderr)
            sys.exit(EXIT_UNSUPPORTED)

        try:
            rec = TopicRecorder(compression="lz4")
        except RevoHumanError as exc:
            sys.exit(format_error(exc))
        rec.record(glove.subscribe())            # 没在录时帧被丢弃，开录后落盘
        presses = glove.button.subscribe()
        print(f"已连接 {glove.port}，按一下手套上的按键开始录制，Ctrl-C 退出")

        handle, episode = None, 0
        try:
            while True:
                try:
                    ev = presses.recv(timeout=1.0)
                except queue.Empty:
                    continue
                if ev.missed_presses:
                    print(f"（有 {ev.missed_presses} 次按压没单独收到，以手套当前状态为准）")
                if ev.mode is ButtonMode.START and handle is None:
                    episode += 1
                    path = f"episode_{episode:03d}.mcap"
                    handle = rec.start(path)
                    print(f"开始录制 {path}")
                elif ev.mode is ButtonMode.END and handle is not None:
                    summary = handle.stop()
                    handle = None
                    print(f"结束录制：{summary['frames']} 帧")
                elif ev.mode is ButtonMode.END:
                    print("手套报「结束」（当前没在录），再按一下开始")
        except KeyboardInterrupt:
            pass
        except RevoHumanError as exc:
            sys.exit(format_error(exc))
        finally:
            presses.close()
            if handle is not None:
                summary = handle.stop()
                print(f"结束录制：{summary['frames']} 帧")
            rec.close()


if __name__ == "__main__":
    main()
