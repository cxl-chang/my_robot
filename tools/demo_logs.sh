#!/usr/bin/env bash
# 调试日志速查。
#   tools/demo_logs.sh where            # 最近一次 run 的日志在哪
#   tools/demo_logs.sh orch             # 最近一次编排器日志（python3_*.log）的路径
#   tools/demo_logs.sh tail [N]         # 编排器日志末尾 N 行（默认 40）
#   tools/demo_logs.sh grep <正则> [分钟]  # 在所有节点日志里搜（默认最近 60 分钟内改过的）
#   tools/demo_logs.sh run <时间戳|目录>  # 列出某次 run 的 launch.log 与相关节点日志
set -u
LOG_DIR="${ROS_LOG_DIR:-$HOME/.ros/log}"

orch() { ls -1t "$LOG_DIR"/python3_*.log 2>/dev/null | head -1; }

case "${1:-where}" in
  where)
    d=$(ls -1dt "$LOG_DIR"/*/ 2>/dev/null | head -1)
    echo "最新 run 目录 : ${d:-（无）}"
    echo "launch.log    : ${d:+${d}launch.log}   （只有 launch 级进程启停，不含节点输出）"
    echo "编排器日志    : $(orch)"
    echo "最近改过的节点日志:"
    find "$LOG_DIR" -maxdepth 1 -name '*.log' -newermt '-30 minutes' 2>/dev/null \
      | while read -r f; do echo "  $(stat -c %y "$f" | cut -d. -f1)  $(basename "$f")"; done | sort | tail -15
    ;;
  orch)  orch ;;
  tail)  n="${2:-40}"; f=$(orch); [ -n "$f" ] && tail -n "$n" "$f" || echo "没有编排器日志" ;;
  grep)
    pat="${2:?用法: demo_logs.sh grep <正则> [分钟]}"; mins="${3:-60}"
    files=$(find "$LOG_DIR" -maxdepth 1 -name '*.log' -newermt "-${mins} minutes" 2>/dev/null)
    # shellcheck disable=SC2086
    grep -HnE "$pat" $files 2>/dev/null | sed 's#'"$LOG_DIR"'/##' | tail -80
    ;;
  run)
    key="${2:?用法: demo_logs.sh run <目录名或时间戳片段>}"
    d=$(ls -1d "$LOG_DIR"/*"$key"*/ 2>/dev/null | head -1)
    [ -z "$d" ] && { echo "没找到匹配 $key 的 run 目录"; exit 1; }
    echo "run 目录: $d"; ls -lh "$d"
    ;;
  *) echo "用法: $0 {where|orch|tail [N]|grep <正则> [分钟]|run <片段>}" ;;
esac
