cmake_minimum_required(VERSION 3.22)
project(my_robot_moveit_config)

find_package(ament_cmake REQUIRED)

ament_package()

if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/launch")
  install(
    DIRECTORY launch
    DESTINATION share/${PROJECT_NAME}
    PATTERN "setup_assistant.launch" EXCLUDE)
endif()

install(DIRECTORY config DESTINATION share/${PROJECT_NAME})
install(FILES .setup_assistant DESTINATION share/${PROJECT_NAME})

# 离线 FK/反解工具：用于生成/校验 SRDF 命名姿态（carry）与抓取位姿的关节角
install(PROGRAMS scripts/arm_kinematics.py
  DESTINATION lib/${PROJECT_NAME})
