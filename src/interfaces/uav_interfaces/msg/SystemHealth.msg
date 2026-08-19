# Producer: Safety Supervisor (Person 4)
# Consumers: Mission Manager, Ground Station (telemetry, non-blocking)
# Rate: 5-10 Hz

std_msgs/Header header

uint8 LEVEL_OK=0
uint8 LEVEL_WARN=1
uint8 LEVEL_CRITICAL=2
uint8 LEVEL_FAILSAFE=3

uint8 overall_level

uint8 localization_level
uint8 world_model_level
uint8 planning_level
uint8 vehicle_link_level

string[] active_faults        # short fault codes, e.g. ["LOC_LOST", "PLANNER_TIMEOUT"]
