# Producer: Mission Manager (Person 4), loaded from mission file / operator input
# Consumers: Global Planner (Person 3)
# Rate: on-change only (latched / transient-local QoS)

std_msgs/Header header

string mission_id
uav_interfaces/Waypoint[] waypoints   # ordered list, frame "map" (ENU)
float32 max_speed                     # m/s
float32 boundary_radius               # meters, geofence radius around first waypoint (0 = no limit)
float32 min_altitude                  # meters AGL
float32 max_altitude                  # meters AGL
