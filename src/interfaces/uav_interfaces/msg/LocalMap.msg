# Producer: World Model (Person 2)
# Consumers: Planning, Safety
# Frame: "map" (ENU), rolling local window centered on vehicle
# Rate: 5-10 Hz
# Validity: consumers must check header.stamp age vs current time; stale maps (> 1s) should be treated with caution.

std_msgs/Header header

float32 resolution              # meters per voxel, see docs/CONVENTIONS.md (default 0.2 m)
geometry_msgs/Point origin      # world-frame coordinate of voxel grid index (0,0,0)
uint32 size_x                   # voxel grid dimensions
uint32 size_y
uint32 size_z

# Occupancy: 0 = free, 1 = occupied, 255 = unknown. Length = size_x*size_y*size_z, row-major (x fastest).
uint8[] occupancy

bool map_valid
