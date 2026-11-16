# 机载文件更名（2026-10-30）

本次仅调整实际文件名及其引用。工程目录、ROS包名、节点、话题、C++类、参数和算法逻辑保留原值。此前扩大到包/节点/话题的临时方案已撤回，暂存工程移入机载备份，不用于当前运行。

| 原文件名 | 新文件名 |
| --- | --- |
| ego_planner_node.cpp | planner_node.cpp |
| diff_replan_fsm.cpp | replan_fsm.cpp |
| diff_replan_fsm.h | replan_fsm.h |
| run_ego_planner_board.launch | run_planner_board.launch |
| 可执行文件 ego_planner_node | planner_node |
| ego_planner_rk3588_deploy.tar.gz | planner_rk3588_deploy.tar.gz |
| diff-yopo-demo-limits.json | yopo-demo-limits.json |

同步调整CMake源文件/目标名称、include、launch的type（可执行文件名）和脚本中的文件路径。ROS包仍为ego_planner，launch的节点name和发布/订阅话题保持不变。上位机定位助手中的launch路径同步；不修改已有录包及历史来源记录。

机载备份位于~/naming-backups/planner-20261030-094917，filename-only-changes.json记录实际文件变更。定位已迁回原正式目录，状态确认10Hz定位和30Hz转发正常；因恢复过程中定位重新初始化，需要重新选择目标点。

后续核对补齐：上位机回退文件mtime导致的旧编译缓存已失效并重构，实际启动路径恢复正式目录；record_session.py的可执行文件快照glob与ldd路径也已更新为planner_node。真实快照验证通过，原TOPICS列表保持。核对证据见analysis/localization-start-audit-20261030。
