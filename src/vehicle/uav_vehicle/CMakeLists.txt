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

add_executable(mock_vehicle src/mock_vehicle.cpp)
ament_target_dependencies(mock_vehicle rclcpp std_msgs uav_interfaces)
target_compile_features(mock_vehicle PUBLIC cxx_std_20)

install(TARGETS mock_vehicle DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
