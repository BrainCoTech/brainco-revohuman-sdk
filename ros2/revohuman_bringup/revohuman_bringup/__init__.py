"""revohuman_bringup — 只有 launch 与参数默认值，**没有节点代码**。

节点实现在 `revohuman_ros`；本包存在的意义是给这套栈一个**唯一入口**，
让「拉起来要设哪些参数」只有一份答案（`config/revohuman.yaml`），
而不是散在各人的 shell 历史里。
"""
