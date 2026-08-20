cmake_minimum_required(VERSION 3.22)
project(uav_world_model)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 20)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(uav_interfaces REQUIRED)
find_package(Eigen3 REQUIRED)

add_executable(mock_world_model src/mock_world_model.cpp)
ament_target_dependencies(mock_world_model rclcpp geometry_msgs uav_interfaces)
target_compile_features(mock_world_model PUBLIC cxx_std_20)

# Core mapping/tracking: ROS-free (same pattern as uav_planning_core) so
# it's unit-testable without a ROS runtime. real_world_model is the only
# place PointCloud2/LocalizationState get converted into calls on it.
add_library(uav_world_model_core
  src/voxel_mapper.cpp
  src/obstacle_tracker.cpp
)
target_include_directories(uav_world_model_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(uav_world_model_core PUBLIC Eigen3::Eigen)
target_compile_features(uav_world_model_core PUBLIC cxx_std_20)

add_executable(real_world_model src/real_world_model.cpp)
target_link_libraries(real_world_model uav_world_model_core)
ament_target_dependencies(real_world_model rclcpp geometry_msgs sensor_msgs uav_interfaces)
target_compile_features(real_world_model PUBLIC cxx_std_20)

install(TARGETS mock_world_model real_world_model DESTINATION lib/${PROJECT_NAME})
install(TARGETS uav_world_model_core
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)
install(DIRECTORY include/ DESTINATION include)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()

  find_package(ament_cmake_gtest REQUIRED)

  ament_add_gtest(test_voxel_mapper test/test_voxel_mapper.cpp)
  target_link_libraries(test_voxel_mapper uav_world_model_core)

  ament_add_gtest(test_obstacle_tracker test/test_obstacle_tracker.cpp)
  target_link_libraries(test_obstacle_tracker uav_world_model_core)
endif()

ament_package()
