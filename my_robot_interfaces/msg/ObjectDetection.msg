# 物体检测结果（M7 感知节点发布到 /object_detection）
# 位姿默认在 camera_link 系（后续升级 YOLO 时消息格式不变）
string class_id                 # 物体类别，如 "red_cube"
geometry_msgs/PoseStamped pose  # 物体中心 6DoF 位姿（相机坐标系）
float32 confidence              # 置信度 0~1
int32[4] bbox                   # 2D 检测框 [x, y, w, h]（像素）
