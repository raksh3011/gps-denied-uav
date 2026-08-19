# Producer: Safety Supervisor / PX4 Interface (Person 4), derived from validated Trajectory
# Consumers: PX4 Interface -> PX4 (via MAVROS/uXRCE-DDS)
# Frame: body-relative setpoints are in "base_link" (FLU); position setpoints in "map" (ENU) then converted to NED at the PX4 boundary.
# Rate: matches PX4 offboard setpoint rate requirement (>= 2 Hz, target 20-50 Hz)

std_msgs/Header header

uint8 MODE_POSITION=0
uint8 MODE_VELOCITY=1
uint8 MODE_LAND=2
uint8 MODE_RTL=3
uint8 MODE_HOLD=4
uint8 MODE_DISARM=5
uint8 mode

geometry_msgs/Point position     # meters, used when mode == MODE_POSITION
geometry_msgs/Vector3 velocity   # m/s, used when mode == MODE_VELOCITY
float32 yaw                      # radians

bool valid                       # false => PX4 Interface must reject and hold last safe command
