uint8 OFF=0
uint8 FLASH_RED=1
uint8 FLASH_GREEN=2
uint8 FLASH_BLUE=3
uint8 STEADY_BLUE=4
# Detected by LiDAR, lamp not classified yet. Only published by buoy_mapper (USV-only plan).
uint8 UNKNOWN=5
string id
geometry_msgs/Point position
uint8 state
float64 collision_radius
