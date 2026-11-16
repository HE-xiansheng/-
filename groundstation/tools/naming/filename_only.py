"""只改文件名称及引用；ROS包、节点、话题、C++类名保持原值。"""
import re
FILES={
 'diff_replan_fsm.h':'replan_fsm.h',
 'diff_replan_fsm.cpp':'replan_fsm.cpp',
 'diff_planner_node.cpp':'planner_node.cpp',
 'run_diff_planner_board.launch':'run_planner_board.launch',
 'diff_planner_rk3588_deploy.tar.gz':'planner_rk3588_deploy.tar.gz',
 'diff-yopo-demo-limits.json':'yopo-demo-limits.json',
}
def references(text,filename):
    for old,new in FILES.items():text=text.replace(old,new)
    text=text.replace('devel/lib/diff_planner/diff_planner_node','devel/lib/diff_planner/planner_node')
    text=text.replace(".glob('diff_planner_node')",".glob('planner_node')")
    text=text.replace('.glob("diff_planner_node")','.glob("planner_node")')
    text=re.sub(r'(\btype\s*=\s*[\"\'])diff_planner_node([\"\'])',r'\1planner_node\2',text)
    if filename=='CMakeLists.txt':text=re.sub(r'\bdiff_planner_node\b','planner_node',text)
    return text
