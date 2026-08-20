cmake_minimum_required(VERSION 3.22)
project(uav_planning)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 20)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(uav_interfaces REQUIRED)
find_package(Eigen3 REQUIRED)

add_executable(mock_planner src/mock_planner.cpp)
ament_target_dependencies(mock_planner rclcpp builtin_interfaces uav_interfaces)
target_compile_features(mock_planner PUBLIC cxx_std_20)

# Core planning algorithms: deliberately ROS-free (no rclcpp dependency in
# the library itself) so it's unit-testable without a ROS runtime. Only
# links against uav_interfaces for the TrajectoryPoint message type, which
# is a plain generated struct usable without rclcpp::init.
add_library(uav_planning_core
  src/grid3d.cpp
  src/astar_planner.cpp
  src/theta_star_planner.cpp
  src/dstar_lite_planner.cpp
  src/boundary_checker.cpp
  src/trajectory_generator.cpp
)
target_include_directories(uav_planning_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(uav_planning_core PUBLIC Eigen3::Eigen)
ament_target_dependencies(uav_planning_core PUBLIC uav_interfaces)
target_compile_features(uav_planning_core PUBLIC cxx_std_20)

add_executable(real_planner src/real_planner_node.cpp)
target_link_libraries(real_planner uav_planning_core)
ament_target_dependencies(real_planner rclcpp uav_interfaces)
target_compile_features(real_planner PUBLIC cxx_std_20)

install(TARGETS mock_planner real_planner DESTINATION lib/${PROJECT_NAME})
install(TARGETS uav_planning_core
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)
install(DIRECTORY include/ DESTINATION include)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()

  find_package(ament_cmake_gtest REQUIRED)

  ament_add_gtest(test_grid3d test/test_grid3d.cpp)
  target_link_libraries(test_grid3d uav_planning_core)

  ament_add_gtest(test_astar_planner test/test_astar_planner.cpp)
  target_link_libraries(test_astar_planner uav_planning_core)

  ament_add_gtest(test_theta_star_planner test/test_theta_star_planner.cpp)
  target_link_libraries(test_theta_star_planner uav_planning_core)

  ament_add_gtest(test_dstar_lite_planner test/test_dstar_lite_planner.cpp)
  target_link_libraries(test_dstar_lite_planner uav_planning_core)

  ament_add_gtest(test_boundary_checker test/test_boundary_checker.cpp)
  target_link_libraries(test_boundary_checker uav_planning_core)

  ament_add_gtest(test_trajectory_generator test/test_trajectory_generator.cpp)
  target_link_libraries(test_trajectory_generator uav_planning_core)
endif()

ament_package()
