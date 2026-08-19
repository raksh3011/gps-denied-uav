# Single dynamic/static obstacle, part of ObstacleSet. Frame: "map" (ENU).

uint32 id
geometry_msgs/Point position     # centroid, meters
geometry_msgs/Vector3 velocity   # m/s, zero for static obstacles
float32 radius                   # meters, bounding-sphere approximation

uint8 CLASS_UNKNOWN=0
uint8 CLASS_STATIC=1
uint8 CLASS_DYNAMIC=2
uint8 obstacle_class
