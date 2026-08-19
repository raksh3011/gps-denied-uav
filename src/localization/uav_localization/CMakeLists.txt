cmake_minimum_required(VERSION 3.22)
project(uav_localization)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 20)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(uav_interfaces REQUIRED)

add_executable(mock_localization src/mock_localization.cpp)
ament_target_dependencies(mock_localization rclcpp uav_interfaces)
target_compile_features(mock_localization PUBLIC cxx_std_20)

add_executable(lio_state_bridge src/lio_state_bridge.cpp)
ament_target_dependencies(lio_state_bridge rclcpp nav_msgs uav_interfaces)
target_compile_features(lio_state_bridge PUBLIC cxx_std_20)

install(TARGETS mock_localization lio_state_bridge DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})
install(DIRECTORY config DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
