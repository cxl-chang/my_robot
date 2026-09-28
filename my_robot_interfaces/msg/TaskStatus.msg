# 任务状态反馈（M9 编排节点发布到 /task_status，沿用 nav_status 枚举风格）
int32 IDLE=0
int32 RUNNING=1
int32 NAV_TO_OBJECT=2          # 导航到物体观测点（识别用）
int32 PICKED=3                 # 已抓起（手臂已到 carry 姿态）
int32 NAV_TO_PLACE=4
int32 PLACED=5
int32 FAILED=6
# --- M12/M13 新增 ---
int32 NAV_TO_GRASP=7           # 从观测点前进到抓取点（进入机械臂可达范围）
int32 GRASPING=8               # 机械臂抓取序列执行中（预抓取→下压→夹紧→抬起→收起）
int32 PLACING=9                # 机械臂放置序列执行中（预放置→下放→松爪→退回）
int32 status
string phase                   # 当前阶段描述，例如 "导航到观测点"/"识别中"/
                               # "预抓取"/"下压抓取"/"夹紧"/"收起搬运"/"下放"/"松爪"...
