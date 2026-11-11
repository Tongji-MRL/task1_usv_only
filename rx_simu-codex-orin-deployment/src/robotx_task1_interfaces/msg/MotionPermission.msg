# Optional fail-closed task lease; never bypasses the bridge's original gates.
std_msgs/Header header
uint64 revision
bool allow
string reason
# Optional native mode intent: hold, resume, stop. Empty keeps legacy GUIDED only.
string station_action
