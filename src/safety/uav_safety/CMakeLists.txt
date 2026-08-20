cmake_minimum_required(VERSION 3.22)
project(uav_safety)

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

add_executable(mock_safety src/mock_safety.cpp)
ament_target_dependencies(mock_safety rclcpp uav_interfaces)
target_compile_features(mock_safety PUBLIC cxx_std_20)

# Core decision logic: ROS-free (same pattern as uav_planning_core /
# uav_world_model_core) so it's unit-testable without a ROS runtime.
# real_safety_node is the only place ROS messages get converted into
# calls on this class.
add_library(uav_safety_core
  src/safety_monitor.cpp
)
target_include_directories(uav_safety_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(uav_safety_core PUBLIC Eigen3::Eigen)
target_compile_features(uav_safety_core PUBLIC cxx_std_20)

add_executable(real_safety src/real_safety_node.cpp)
target_link_libraries(real_safety uav_safety_core)
ament_target_dependencies(real_safety rclcpp uav_interfaces)
target_compile_features(real_safety PUBLIC cxx_std_20)

install(TARGETS mock_safety real_safety DESTINATION lib/${PROJECT_NAME})
install(TARGETS uav_safety_core
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)
install(DIRECTORY include/ DESTINATION include)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()

  find_package(ament_cmake_gtest REQUIRED)

  ament_add_gtest(test_safety_monitor test/test_safety_monitor.cpp)
  target_link_libraries(test_safety_monitor uav_safety_core)
endif()

ament_package()
