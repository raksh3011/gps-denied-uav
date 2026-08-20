cmake_minimum_required(VERSION 3.22)
project(uav_vehicle)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 20)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(uav_interfaces REQUIRED)
find_package(Eigen3 REQUIRED)

add_executable(mock_vehicle src/mock_vehicle.cpp)
ament_target_dependencies(mock_vehicle rclcpp std_msgs uav_interfaces)
target_compile_features(mock_vehicle PUBLIC cxx_std_20)

# Core PX4 command translation: ROS-free (same pattern as
# uav_planning_core/uav_world_model_core/uav_safety_core) so it's
# unit-testable without a ROS runtime AND without the vendored px4_msgs
# dependency — real_vehicle_node is the only place this gets wired to
# actual PX4 uORB topics. See docs/VEHICLE.md.
add_library(uav_vehicle_core
  src/px4_command_bridge.cpp
)
target_include_directories(uav_vehicle_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(uav_vehicle_core PUBLIC Eigen3::Eigen)
target_compile_features(uav_vehicle_core PUBLIC cxx_std_20)

install(TARGETS mock_vehicle DESTINATION lib/${PROJECT_NAME})
install(TARGETS uav_vehicle_core
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)
install(DIRECTORY include/ DESTINATION include)

# real_vehicle_node depends on the vendored px4_msgs package (see
# uav_vehicle.repos + docs/VEHICLE.md) — only built if it's present in
# the workspace, so uav_vehicle_core/mock_vehicle keep building and
# testing fine without it (e.g. on this repo's dev machine, which never
# vendors px4_msgs at all).
find_package(px4_msgs QUIET)
if(px4_msgs_FOUND)
  add_executable(real_vehicle src/real_vehicle_node.cpp)
  target_link_libraries(real_vehicle uav_vehicle_core)
  ament_target_dependencies(real_vehicle rclcpp uav_interfaces px4_msgs)
  target_compile_features(real_vehicle PUBLIC cxx_std_20)
  install(TARGETS real_vehicle DESTINATION lib/${PROJECT_NAME})
else()
  message(STATUS "uav_vehicle: px4_msgs not found, skipping real_vehicle (see docs/VEHICLE.md)")
endif()

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()

  find_package(ament_cmake_gtest REQUIRED)

  ament_add_gtest(test_px4_command_bridge test/test_px4_command_bridge.cpp)
  target_link_libraries(test_px4_command_bridge uav_vehicle_core)
endif()

ament_package()
