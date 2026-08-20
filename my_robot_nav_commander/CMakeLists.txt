cmake_minimum_required(VERSION 3.8)
project(my_robot_nav_commander)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(my_robot_interfaces REQUIRED)

add_executable(nav_commander src/nav_commander.cpp)
ament_target_dependencies(nav_commander
  rclcpp
  rclcpp_action
  nav2_msgs
  geometry_msgs
  std_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
  my_robot_interfaces)

install(TARGETS
  nav_commander
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
