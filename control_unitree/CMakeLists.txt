cmake_minimum_required(VERSION 3.5)
project(control_unitree)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(action_msg REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(unitree_ros2_msgs REQUIRED)
find_package(unitree_legged_sdk REQUIRED)

add_executable(action_server_member_functions member_functions.cpp)
ament_target_dependencies(action_server_member_functions
  "rclcpp"
  "rclcpp_action"
  "action_msg"
   geometry_msgs
    rclcpp
    tf2
    tf2_ros
   nav_msgs
  std_msgs
  sensor_msgs
  unitree_ros2_msgs
  unitree_legged_sdk)

#add_executable(action_server_not_composable not_composable.cpp)
#ament_target_dependencies(action_server_not_composable
#  "rclcpp"
#  "rclcpp_action"
#  "action_msg")

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

install(TARGETS
#  action_server_not_composable
  action_server_member_functions
  DESTINATION lib/${PROJECT_NAME})

ament_package()
