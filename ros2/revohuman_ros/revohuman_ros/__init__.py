"""revohuman_ros — RevoHuman 手套 SDK 的 ROS 2 接口。

- ``glove_node``（单手，ns=/revohuman/{left|right}）
- ``bimanual_node``（双手硬件同步，ns=/revohuman/pair）

话题、参数与服务见 docs/ 下《RevoHuman SDK 参考手册》第 7 节「ROS 2」。节点永不注册 ``set_sn``。
"""
