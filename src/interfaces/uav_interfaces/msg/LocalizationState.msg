# Producer: Localization (Person 1)
# Consumers: World Model, Planning, Safety, Mission
# Frame: pose is expressed in the "map" frame (ENU). See docs/CONVENTIONS.md.
# Rate: 100-200 Hz (LIO output rate)
# Validity: consumers MUST check `localization_ok` and `confidence` before use.

std_msgs/Header header          # header.frame_id = "map", header.stamp = sensor time of pose estimate

geometry_msgs/Pose pose         # position [m, ENU] + orientation (quaternion, x,y,z,w, Hamilton convention)
geometry_msgs/Twist twist       # linear [m/s] and angular [rad/s] velocity, expressed in body frame "base_link"

float64[36] pose_covariance     # row-major 6x6 (x,y,z,roll,pitch,yaw), m^2 / rad^2
float64[36] twist_covariance    # row-major 6x6

float32 confidence              # [0.0, 1.0], localization quality score
bool localization_ok            # false => do not trust pose/twist, Safety must react

uint8 STATUS_NOMINAL=0
uint8 STATUS_DEGRADED=1
uint8 STATUS_LOST=2
uint8 status                    # one of the above
