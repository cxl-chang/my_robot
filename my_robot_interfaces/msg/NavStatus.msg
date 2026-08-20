# 导航状态反馈（由 my_robot_nav_commander 发布到 nav_status）
int32 IDLE=0
int32 RUNNING=1
int32 ARRIVED=2
int32 FAILED=3
int32 status
geometry_msgs/PoseStamped current_pose  # 机器人当前在 map 系下的位姿
