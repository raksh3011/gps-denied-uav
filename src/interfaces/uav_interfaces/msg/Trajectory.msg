# Producer: Local Planner (Person 3)
# Consumers: Safety Supervisor, PX4 Interface (Person 4)
# Frame: "map" (ENU)
# Rate: 10-20 Hz (replanning rate)
# Validity: consumers must check `valid`; an invalid Trajectory must never be forwarded to PX4.

std_msgs/Header header

uav_interfaces/TrajectoryPoint[] points   # time-parameterized, points[0] ~= current state
bool valid
