# Producer: Planning (Person 3)
# Consumers: Mission Manager, Safety Supervisor
# Rate: matches Trajectory rate (10-20 Hz)

std_msgs/Header header

uint8 STATE_IDLE=0
uint8 STATE_PLANNING=1
uint8 STATE_EXECUTING=2
uint8 STATE_REPLANNING=3
uint8 STATE_GOAL_REACHED=4
uint8 STATE_FAILED=5
uint8 state

string message               # human-readable diagnostic, empty when nominal
float32 progress             # [0.0, 1.0] fraction of current leg completed
