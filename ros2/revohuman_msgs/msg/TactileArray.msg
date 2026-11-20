# One frame of the tactile array.
#
# The two timestamps are not redundant: tactile has its own tick, and its sampling time is +3 to +9 ms
# later than the angles of the same cycle.
# header.stamp tells the truth (tactile's own sampling time, mapped from tact_tick);
# frame_stamp gives the exact pairing key = the header.stamp of joint_states from the same cycle,
# bit-identical; downstream just matches on equal frame_stamp, no nearest needed (on a 5 ms cycle,
# nearest would cross ticks).
#
# Exception: with split_tactile on (split tactile; on by default in revohuman_bringup, off by default as a
# node parameter), tactile is published sample by sample at the sensor module's own rate and no longer
# corresponds to any joint frame: frame_stamp is filled with the same value as header.stamp, and downstream
# must pair by nearest header.stamp, not by equal frame_stamp.
#
# calibrated is data semantics, not a status: false = data can only be treated as raw counts, not as force.
std_msgs/Header header
builtin_interfaces/Time frame_stamp
uint32[] data
bool calibrated
