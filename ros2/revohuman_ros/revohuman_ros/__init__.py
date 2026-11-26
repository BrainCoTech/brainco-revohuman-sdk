"""revohuman_ros — ROS 2 interface for the RevoHuman glove SDK.

- ``glove_node`` (single hand, ns=/revohuman/{left|right})
- ``bimanual_node`` (both hands, hardware-synchronized, ns=/revohuman/pair)

For topics, parameters and services, see section 7 "ROS 2" of the *RevoHuman SDK Reference Manual* under docs/.
The nodes never expose ``set_sn``.
"""
