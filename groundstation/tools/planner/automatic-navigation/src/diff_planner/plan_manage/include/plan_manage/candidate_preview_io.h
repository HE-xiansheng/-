#pragma once
// 仅用于同机预览子进程的有界二进制快照；不承载飞控命令。
#include <plan_manage/planner_manager.h>
#include <traj_utils/PolyTraj.h>
#include <cstdint>
#include <cmath>
#include <cstring>
#include <istream>
#include <ostream>

namespace candidate_io {
template<class T> inline bool put(std::ostream &out,const T &v) { out.write(reinterpret_cast<const char*>(&v),sizeof(v));return bool(out); }
template<class T> inline bool get(std::istream &in,T &v) { in.read(reinterpret_cast<char*>(&v),sizeof(v));return bool(in); }
inline bool putVec(std::ostream &out,const Eigen::Vector3d &v) { for(int i=0;i<3;++i)if(!put(out,v(i)))return false;return true; }
inline bool getVec(std::istream &in,Eigen::Vector3d &v) { for(int i=0;i<3;++i)if(!get(in,v(i)))return false;return v.allFinite(); }

inline bool putTrajectory(std::ostream &out,const poly_traj::Trajectory &traj) {
  const uint32_t n=traj.getPieceNum();if(!n || n>2000 || !put(out,n))return false;
  for(uint32_t i=0;i<n;++i) {
    const auto piece=traj.getPiece(i);const double duration=piece.getDuration();const auto coeff=piece.getCoeffMat();
    if(!std::isfinite(duration) || duration<=0 || !coeff.allFinite() || !put(out,duration))return false;
    for(int r=0;r<3;++r)for(int c=0;c<6;++c)if(!put(out,coeff(r,c)))return false;
  } return bool(out);
}
inline bool getTrajectory(std::istream &in,poly_traj::Trajectory &traj) {
  uint32_t n=0;if(!get(in,n) || !n || n>2000)return false;
  std::vector<double> durations;std::vector<poly_traj::CoefficientMat> coeffs;
  durations.reserve(n);coeffs.reserve(n);double total=0;
  for(uint32_t i=0;i<n;++i) {
    double duration;poly_traj::CoefficientMat coeff;
    if(!get(in,duration) || !std::isfinite(duration) || duration<=0 || (total+=duration)>300)return false;
    for(int r=0;r<3;++r)for(int c=0;c<6;++c)if(!get(in,coeff(r,c)) || !std::isfinite(coeff(r,c)))return false;
    durations.push_back(duration);coeffs.push_back(coeff);
  }
  traj=poly_traj::Trajectory(durations,coeffs);return true;
}

struct Request {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  uint32_t id{0};double horizon{0};Eigen::Vector3d start,goal,velocity,acceleration;
  diff_planner::PlanParameters manager;
  diff_planner::PolyTrajOptimizer::Parameters optimizer;
  diff_planner::SwarmTrajData swarm;
  GridMap::Ptr map;
};

inline bool writeRequest(std::ostream &out,const Request &r) {
  const char magic[8]={'G','C','S','R','E','Q','2',0};out.write(magic,8);
  const uint8_t multi=r.manager.use_multitopology_trajs?1:0;
  if(!put(out,r.id)||!put(out,r.horizon)||!putVec(out,r.start)||!putVec(out,r.goal)||!putVec(out,r.velocity)||!putVec(out,r.acceleration)||
     !put(out,r.manager.max_vel_)||!put(out,r.manager.max_acc_)||!put(out,r.manager.feasibility_tolerance_)||
     !put(out,r.manager.polyTraj_piece_length)||!put(out,r.manager.planning_horizen_)||!put(out,multi)||!put(out,r.manager.drone_id)||!put(out,r.optimizer.cps))return false;
  const auto &p=r.optimizer;const double values[]={p.obs,p.obs_soft,p.swarm,p.feas,p.sqrvar,p.time,p.clearance,p.clearance_soft,p.swarm_clearance,p.max_vel,p.vel_tolerance,p.max_acc,p.max_vel_z,p.acc_tolerance,p.max_jer};
  for(double v:values)if(!put(out,v))return false;
  const uint32_t n=r.swarm.size();if(n>256||!put(out,n))return false;
  for(uint32_t i=0;i<n;++i) {
    const auto &s=r.swarm[i];const uint8_t valid=s.drone_id==static_cast<int>(i)?1:0;
    if(!put(out,valid))return false;
    if(valid && (!put(out,s.drone_id)||!put(out,s.traj_id)||!put(out,s.start_time)||!put(out,s.duration)||!put(out,s.des_clearance)||!putTrajectory(out,s.traj)))return false;
  }
  return r.map && r.map->writePreviewSnapshot(out);
}

inline bool readRequest(std::istream &in,Request &r) {
  char magic[8];in.read(magic,8);uint8_t multi=0;
  if(!in||std::memcmp(magic,"GCSREQ2",7)!=0||!get(in,r.id)||!r.id||!get(in,r.horizon)||!std::isfinite(r.horizon)||r.horizon<=0||!getVec(in,r.start)||!getVec(in,r.goal)||!getVec(in,r.velocity)||!getVec(in,r.acceleration)||
     !get(in,r.manager.max_vel_)||!get(in,r.manager.max_acc_)||!get(in,r.manager.feasibility_tolerance_)||
     !get(in,r.manager.polyTraj_piece_length)||!get(in,r.manager.planning_horizen_)||!get(in,multi)||multi>1||!get(in,r.manager.drone_id)||!get(in,r.optimizer.cps))return false;
  r.manager.use_multitopology_trajs=multi;
  auto &p=r.optimizer;double *values[]={&p.obs,&p.obs_soft,&p.swarm,&p.feas,&p.sqrvar,&p.time,&p.clearance,&p.clearance_soft,&p.swarm_clearance,&p.max_vel,&p.vel_tolerance,&p.max_acc,&p.max_vel_z,&p.acc_tolerance,&p.max_jer};
  for(double *v:values)if(!get(in,*v)||!std::isfinite(*v))return false;
  if(!std::isfinite(r.manager.max_vel_)||r.manager.max_vel_<=0||!std::isfinite(r.manager.max_acc_)||r.manager.max_acc_<=0||
     !std::isfinite(r.manager.polyTraj_piece_length)||r.manager.polyTraj_piece_length<=0||p.cps<=0)return false;
  uint32_t n=0;if(!get(in,n)||n>256)return false;r.swarm.resize(n);
  for(uint32_t i=0;i<n;++i) {
    auto &s=r.swarm[i];s.drone_id=-1;s.traj_id=0;s.start_time=s.duration=s.des_clearance=0;uint8_t valid=0;
    if(!get(in,valid)||valid>1)return false;
    if(valid && (!get(in,s.drone_id)||s.drone_id!=static_cast<int>(i)||!get(in,s.traj_id)||!get(in,s.start_time)||!std::isfinite(s.start_time)||
       !get(in,s.duration)||!std::isfinite(s.duration)||s.duration<=0||!get(in,s.des_clearance)||!std::isfinite(s.des_clearance)||!getTrajectory(in,s.traj)))return false;
  }
  r.map=GridMap::readPreviewSnapshot(in);return bool(r.map);
}

inline bool writeResult(std::ostream &out,uint32_t id,const diff_planner::DiffPlannerManager *manager) {
  const char magic[8]={'G','C','S','R','E','S','1',0};out.write(magic,8);const uint8_t success=manager?1:0;
  if(!put(out,id)||!put(out,success))return false;
  return !success || (put(out,manager->pp_.drone_id)&&put(out,manager->traj_.local_traj.start_time)&&putTrajectory(out,manager->traj_.local_traj.traj));
}

inline bool readResult(std::istream &in,uint32_t expected,traj_utils::PolyTraj &poly,poly_traj::Trajectory &path,bool &success) {
  char magic[8];in.read(magic,8);uint32_t id=0;uint8_t state=0;
  if(!in||std::memcmp(magic,"GCSRES1",7)!=0||!get(in,id)||id!=expected||!get(in,state)||state>1)return false;
  success=state;if(!success)return true;
  int drone_id=0;double start=0;
  if(!get(in,drone_id)||drone_id<INT16_MIN||drone_id>INT16_MAX||!get(in,start)||!std::isfinite(start)||start<=0||!getTrajectory(in,path))return false;
  poly.drone_id=static_cast<int16_t>(drone_id);
  poly.traj_id=id;poly.start_time=ros::Time(start);poly.order=5;const int n=path.getPieceNum();
  poly.duration.resize(n);poly.coef_x.resize(6*n);poly.coef_y.resize(6*n);poly.coef_z.resize(6*n);
  for(int i=0;i<n;++i) {
    const auto piece=path.getPiece(i);poly.duration[i]=piece.getDuration();const auto c=piece.getCoeffMat();
    for(int j=0;j<6;++j){poly.coef_x[6*i+j]=c(0,j);poly.coef_y[6*i+j]=c(1,j);poly.coef_z[6*i+j]=c(2,j);}
  }
  return true;
}
}
