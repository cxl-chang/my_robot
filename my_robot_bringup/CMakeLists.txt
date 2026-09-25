cmake_minimum_required(VERSION 3.8)
project(my_robot_bringup)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

install (
  DIRECTORY launch config worlds
  DESTINATION share/${PROJECT_NAME}/
)

# 仿真专用小工具：把左指位置镜像给右指（见 scripts/gripper_mirror.py 的说明）
install(PROGRAMS
  scripts/gripper_mirror.py
  scripts/check_gripper.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
