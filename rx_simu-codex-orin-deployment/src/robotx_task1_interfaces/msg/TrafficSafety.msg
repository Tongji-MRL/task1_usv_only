# Source-stamped assessment from LiDAR and FCU TF only. Missing/stale is unsafe.
std_msgs/Header header
builtin_interfaces/Time path_stamp
bool clear
string reason
uint64 track_id
float64 conflict_after_s
# Conservative full-rotation envelope against surveyed and observed obstacles.
bool hold_clear
float64 hold_envelope_radius
