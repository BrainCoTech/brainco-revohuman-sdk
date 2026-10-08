"""RevoHuman 手套的 MuJoCo 可视化：双手正运动学 + tracker 6D 位姿 + 触觉热力图。

入口是 simulation/viewer.py（或 simulation/run_viewer.sh）。本包不随 SDK 安装，只从发布包的
simulation/ 目录里跑；它不 import SDK，数据只从 SDK 的本机广播（``revohuman-live``）订阅，
协议见 live_client.py。
"""
