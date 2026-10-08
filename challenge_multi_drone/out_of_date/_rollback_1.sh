#!/usr/bin/env bash
# 回滚 cleanup_outdated_1.sh：把 out_of_date 里的文件按清单搬回原位
# 注意：traces/*.csv 已删除，无法恢复（需要的话重跑任务脚本即可重新生成）
set -u
cd ~/projects/aerostack-swarm-drone-traversal || exit 1
M=challenge_multi_drone/out_of_date/_manifest_1.txt
[ -f "$M" ] || { echo "清单不存在: $M"; exit 1; }
while IFS='|' read -r kind a b; do
  if [ "$kind" = "DEL" ]; then
    echo "[skip] 已删除、无法恢复: $a"
  else
    [ -f "$a" ] && { echo "[skip] 原位置已有文件: $a"; continue; }
    mkdir -p "$(dirname "$a")"
    mv -n "$b" "$a" && echo "restored $a"
  fi
done < "$M"
echo "回滚完成（out_of_date 里的空目录可手动删）"
