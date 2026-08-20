cmake_minimum_required(VERSION 3.22)
project(uav_bringup)

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
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(uav_interfaces REQUIRED)

add_executable(viz_bridge src/viz_bridge.cpp)
ament_target_dependencies(viz_bridge
  rclcpp geometry_msgs nav_msgs visualization_msgs uav_interfaces)
target_compile_features(viz_bridge PUBLIC cxx_std_20)

add_executable(demo_world src/demo_world.cpp)
ament_target_dependencies(demo_world rclcpp sensor_msgs)
target_compile_features(demo_world PUBLIC cxx_std_20)

add_executable(demo_flyer src/demo_flyer.cpp)
ament_target_dependencies(demo_flyer rclcpp uav_interfaces)
target_compile_features(demo_flyer PUBLIC cxx_std_20)

install(TARGETS viz_bridge demo_world demo_flyer DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})
install(DIRECTORY config DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
