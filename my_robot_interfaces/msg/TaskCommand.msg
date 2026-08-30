# 任务命令：「抓取 object_id 物体，放到 place 点」
# 由用户/上层发布到 /task_cmd，M9 编排节点接收
string object_id                # 要抓的物体类别（如 "red_cube"）
string place_frame_id           # 放置坐标系，默认 "map"
float64 place_x
float64 place_y
float64 place_yaw               # 放置朝向
