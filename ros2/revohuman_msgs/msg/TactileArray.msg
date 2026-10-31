# 触觉阵列一帧（设计稿 §04，2026-09-17）。
#
# 两个戳不是冗余：触觉有自己的 tick，采样时刻比同周期角度晚 +3 到 +9 ms。
# header.stamp 说真话（触觉自己的采样时刻，tact_tick 经映射）；
# frame_stamp 给精确配对键 = 同周期 joint_states 的 header.stamp，逐位相同，
# 下游按 frame_stamp 等值匹配即可，不需要 nearest（5 ms 周期上 nearest 会跨拍）。
#
# calibrated 是数据语义不是状态：false = data 只能当原始计数，不能当力。
std_msgs/Header header
builtin_interfaces/Time frame_stamp
uint32[] data
bool calibrated
