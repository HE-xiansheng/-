// 与实飞规划器进程隔离；使用其地图与参数快照生成只读候选轨迹。
#include <plan_manage/candidate_preview_io.h>
#include <traj_utils/diagnostics.hpp>
#include <fstream>
#include <sys/resource.h>
#include <dirent.h>
#include <unistd.h>
#include <cstdlib>

int main(int argc,char **argv)
{
  if (argc!=4) return 2;
  const std::string input=argv[1],output=argv[2],parameter_namespace=argv[3];
  // 不继承实飞节点的ROS连接、日志或锁文件描述符。
  DIR *fds=opendir("/proc/self/fd");
  if (!fds) return 6;
  const int directory_fd=dirfd(fds);
  while (const dirent *entry=readdir(fds)) {
    char *end=nullptr;const long fd=strtol(entry->d_name,&end,10);
    if (end && *end=='\0' && fd>2 && fd!=directory_fd) close(static_cast<int>(fd));
  }
  closedir(fds);
  struct rlimit memory;
  if (getrlimit(RLIMIT_AS,&memory)==0) {
    const rlim_t cap=static_cast<rlim_t>(2)*1024*1024*1024;
    if (memory.rlim_cur==RLIM_INFINITY || memory.rlim_cur>cap) memory.rlim_cur=cap;
    setrlimit(RLIMIT_AS,&memory);
  }
  const struct rlimit no_core{0,0};setrlimit(RLIMIT_CORE,&no_core);
  setpriority(PRIO_PROCESS,0,10);
  ros::init(argc,argv,"diff_planner_candidate_preview",ros::init_options::AnonymousName | ros::init_options::NoRosout);
  flight_diag::enabled()=false;
  candidate_io::Request request;
  try
  {
    std::ifstream in(input,std::ios::binary);
    if (!in || !candidate_io::readRequest(in,request)) { std::cerr<<"候选快照读取或格式校验失败"<<std::endl;return 3; }
    in.close();
    ros::NodeHandle params(parameter_namespace);
    ros::NodeHandle visual(params,"candidate_preview_worker");
    diff_planner::PlanningVisualization::Ptr visualization(new diff_planner::PlanningVisualization(visual));
    diff_planner::DiffPlannerManager manager;
    manager.initPlanModules(params,visualization,request.map);
    manager.setPreviewParameters(request.manager,request.optimizer);
    manager.traj_.swarm_traj=request.swarm;
    const Eigen::Vector3d zero=Eigen::Vector3d::Zero();
    const std::vector<Eigen::Vector3d> goals{request.goal};
    bool success=manager.planGlobalTrajWaypoints(request.start,request.velocity,request.acceleration,goals,zero,zero);
    Eigen::Vector3d local,velocity;bool touch=false;
    if (success)
    {
      manager.getLocalTarget(request.horizon,request.start,request.goal,local,velocity,touch);
      success=manager.reboundReplan(request.start,request.velocity,request.acceleration,local,velocity,true,false,touch);
    }
    std::ofstream out(output,std::ios::binary | std::ios::trunc);
    return out && candidate_io::writeResult(out,request.id,success ? &manager : nullptr) ? 0 : 4;
  }
  catch (const std::exception &e) { std::cerr<<"候选预览异常: "<<e.what()<<std::endl;return 5; }
  catch (...) { std::cerr<<"候选预览未知异常"<<std::endl;return 5; }
}
