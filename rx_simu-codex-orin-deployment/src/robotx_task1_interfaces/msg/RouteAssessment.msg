# Advisory route timing from the source LiDAR observation, never motion permission.
std_msgs/Header header
builtin_interfaces/Time path_stamp
builtin_interfaces/Time reference_stamp
uint64 revision
float64 eta_s
# -1 means no clear departure found within the advisory wait horizon.
float64 wait_s
float64 reference_eta_s
float64 reference_wait_s
float64 switch_cost_s
float64 conflict_eta_s
# Affordable total observation time while the reference departure is unknown.
# Zero disables this option. This does not certify that the reference will clear.
float64 observation_budget_s
bool prefer_wait
string reason
