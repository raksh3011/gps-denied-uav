# Single mission waypoint. Frame: "map" (ENU).

geometry_msgs/Point position
float32 yaw                 # radians, NaN = don't-care
float32 acceptance_radius   # meters

uint8 TYPE_NAV=0
uint8 TYPE_TARGET=1
uint8 TYPE_RETURN=2
uint8 TYPE_LAND=3
uint8 waypoint_type
