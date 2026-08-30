# 任务状态反馈（M9 编排节点发布到 /task_status，沿用 nav_status 枚举风格）
int32 IDLE=0
int32 RUNNING=1
int32 NAV_TO_OBJECT=2
int32 PICKED=3
int32 NAV_TO_PLACE=4
int32 PLACED=5
int32 FAILED=6
int32 status
string phase                    # 当前阶段描述（"空闲/导航中/识别中/抓取中/搬运中/放置中/失败"）
