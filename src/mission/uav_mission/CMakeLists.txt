cmake_minimum_required(VERSION 3.22)
project(uav_mission)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 20)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(uav_interfaces REQUIRED)
find_package(Eigen3 REQUIRED)

add_executable(mock_mission src/mock_mission.cpp)
ament_target_dependencies(mock_mission rclcpp uav_interfaces)
target_compile_features(mock_mission PUBLIC cxx_std_20)

# Core waypoint-sequencing state machine: ROS-free (same pattern as
# uav_planning_core/uav_world_model_core/uav_safety_core/uav_vehicle_core)
# so it's unit-testable without a ROS runtime. real_mission_node is the
# only place ROS params/messages get converted into calls on this class.
add_library(uav_mission_core
  src/mission_manager.cpp
)
target_include_directories(uav_mission_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(uav_mission_core PUBLIC Eigen3::Eigen)
target_compile_features(uav_mission_core PUBLIC cxx_std_20)

add_executable(real_mission src/real_mission_node.cpp)
target_link_libraries(real_mission uav_mission_core)
ament_target_dependencies(real_mission rclcpp uav_interfaces)
target_compile_features(real_mission PUBLIC cxx_std_20)

install(TARGETS mock_mission real_mission DESTINATION lib/${PROJECT_NAME})
install(TARGETS uav_mission_core
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)
install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY config DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()

  find_package(ament_cmake_gtest REQUIRED)

  ament_add_gtest(test_mission_manager test/test_mission_manager.cpp)
  target_link_libraries(test_mission_manager uav_mission_core)
endif()

ament_package()
