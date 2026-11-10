#!/usr/bin/env bash
# 一键跑 demo，并把"屏幕上会滚掉的东西"完整存成文件。
#
# 用法:
#   tools/run_demo.sh                       # 默认参数
#   tools/run_demo.sh localization_mode:=slam
#   tools/run_demo.sh scene_place_table:=false grasp_park_margin:=0.30
#
# 输出:
#   ~/demo_logs/run_<时间戳>.log        完整控制台（所有节点按真实顺序交织）
#   ~/.ros/log/<node>_<pid>_<ts>.log    各节点自己的日志（C++/rclpy 独立节点）
#   ~/.ros/log/python3_<pid>_<ts>.log   task_orchestrator（Python 节点以解释器命名）
set -u

OUT_DIR="${DEMO_LOG_DIR:-$HOME/demo_logs}"
mkdir -p "$OUT_DIR"
LOG="$OUT_DIR/run_$(date +%Y%m%d_%H%M%S).log"

echo "=== 日志写入: $LOG ==="
echo "=== 结束后会自动打印关键里程碑；Ctrl-C 随时中止 ==="

ros2 launch my_robot_bringup demo_all.launch.py "$@" 2>&1 | tee "$LOG"
rc=${PIPESTATUS[0]}

echo
echo "================ 关键里程碑（按时间顺序，最后 60 条）================"
grep -E "视觉中值|视觉位姿|视觉 base|AMCL 机器人|抓取校验|规划场景\[|ACM 放行|蠕动|预抓取|下压抓取|吸附物体|预放置|下放|松开夹爪|任务失败|导航成功到达|导航失败|路径碰撞|只算到" "$LOG" | tail -60
echo "================================================================"
echo "完整日志: $LOG"
echo "编排器日志: $(ls -1t "${ROS_LOG_DIR:-$HOME/.ros/log}"/python3_*.log 2>/dev/null | head -1)"
exit $rc
