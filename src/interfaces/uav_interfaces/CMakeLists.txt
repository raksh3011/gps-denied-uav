cmake_minimum_required(VERSION 3.22)
project(uav_interfaces)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  msg/LocalizationState.msg
  msg/LocalMap.msg
  msg/Obstacle.msg
  msg/ObstacleSet.msg
  msg/Waypoint.msg
  msg/Mission.msg
  msg/TrajectoryPoint.msg
  msg/Trajectory.msg
  msg/PlannerStatus.msg
  msg/SystemHealth.msg
  msg/VehicleCommand.msg
  DEPENDENCIES std_msgs geometry_msgs sensor_msgs builtin_interfaces
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()
