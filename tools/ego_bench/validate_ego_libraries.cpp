// Isolated library bench. No FC subscribers, commands, arming or mission interfaces.
#include <plan_manage/planner_manager.h>
#include <traj_utils/diagnostics.hpp>
#include <plan_manage/candidate_preview_io.h>
#include <pcl_conversions/pcl_conversions.h>
#include <sstream>
#include <map>
#include <set>
#include <stdexcept>
#include <chrono>
#include <thread>
#include <iostream>

using V = Eigen::Vector3d;
std::string snapshot(const GridMap::Ptr& m) {
  std::ostringstream out(std::ios::binary); m->writePreviewSnapshot(out); return out.str();
}
int main(int argc,char**argv) {
  rclcpp::init(argc,argv);
  std::map<std::string,bool> checks;
  int messages=0, path_points=0;
  auto check=[&](std::string name,bool value){checks[name]=value;};
  try {
    rclcpp::NodeOptions options;
    options.parameter_overrides({
      {"grid_map.resolution", .1}, {"grid_map.local_update_range_x", 4.0},
      {"grid_map.local_update_range_y", 4.0}, {"grid_map.local_update_range_z", 2.0},
      {"grid_map.obstacles_inflation", .2}, {"grid_map.fading_time", 0.0},
      {"grid_map.frame_id", "camera_init"}, {"visualization.frame_id", "camera_init"},
      {"grid_map.skip_pixel", 2}, {"grid_map.k_depth_scaling_factor", 1000.0},
      {"grid_map.depth_filter_margin", 0}, {"grid_map.fx", 386.0}, {"grid_map.fy", 386.0},
      {"grid_map.cx", 320.0}, {"grid_map.cy", 240.0},
      {"manager.max_vel", 1.0}, {"manager.max_acc", .5}, {"manager.feasibility_tolerance", .05},
      {"manager.polyTraj_piece_length", 1.5}, {"manager.planning_horizon", 4.0}, {"manager.drone_id", 0},
      {"optimization.constraint_points_perPiece", 5}, {"optimization.weight_obstacle", 10000.0},
      {"optimization.weight_obstacle_soft", 5000.0}, {"optimization.weight_swarm", 10000.0},
      {"optimization.weight_feasibility", 10000.0}, {"optimization.weight_sqrvariance", 10000.0},
      {"optimization.weight_time", 10.0}, {"optimization.obstacle_clearance", .1},
      {"optimization.obstacle_clearance_soft", .5}, {"optimization.swarm_clearance", .15},
      {"optimization.max_vel", 1.0}, {"optimization.vel_tolerance", .05},
      {"optimization.max_acc", .5}, {"optimization.max_vel_z", .3},
      {"optimization.acc_tolerance", 1.0}, {"optimization.max_jer", 10.0}
    });
    auto node=std::make_shared<rclcpp::Node>("ego_libraries_bench", "/migration_validation", options);
    auto map=std::make_shared<GridMap>(); map->initMap(node);
    auto visual=std::make_shared<ego_planner::PlanningVisualization>(node);
    auto odom_pub=node->create_publisher<nav_msgs::msg::Odometry>("grid_map/odom",10);
    auto cloud_pub=node->create_publisher<sensor_msgs::msg::PointCloud2>("grid_map/cloud",10);
    int64_t latest_cloud_stamp=0, last_output_stamp=0;
    std::set<int64_t> issued_stamps;
    bool source_stamps=true, frames=true;
    auto out=node->create_subscription<sensor_msgs::msg::PointCloud2>("grid_map/occupancy_inflate",10,
      [&](sensor_msgs::msg::PointCloud2::ConstSharedPtr msg){
        ++messages; frames &= msg->header.frame_id=="camera_init";
        const auto stamp=rclcpp::Time(msg->header.stamp).nanoseconds();
        source_stamps &= issued_stamps.count(stamp) && stamp>=last_output_stamp;
        last_output_stamp=stamp;
      });
    rclcpp::executors::SingleThreadedExecutor exec; exec.add_node(node);
    auto pump=[&](int ms){
      auto deadline=std::chrono::steady_clock::now()+std::chrono::milliseconds(ms);
      while(std::chrono::steady_clock::now()<deadline){exec.spin_some();std::this_thread::sleep_for(std::chrono::milliseconds(2));}
    };
    pump(500);
    nav_msgs::msg::Odometry odom; odom.header.frame_id="camera_init";
    odom.pose.pose.orientation.w=1.0; odom.header.stamp=node->now();
    odom_pub->publish(odom);pump(100);
    pcl::PointCloud<pcl::PointXYZ> points;
    // Small wall on the direct route; a detour is available on either side.
    for(float y=-.5;y<=.5;y+=.1)for(float z=-.5;z<=.5;z+=.1)points.push_back({1.05f,y,z});
    sensor_msgs::msg::PointCloud2 cloud;pcl::toROSMsg(points,cloud);cloud.header.frame_id="camera_init";
    for(int i=0;i<6;++i){cloud.header.stamp=node->now();latest_cloud_stamp=rclcpp::Time(cloud.header.stamp).nanoseconds();issued_stamps.insert(latest_cloud_stamp);cloud_pub->publish(cloud);pump(150);}
    check("obstacle_is_occupied",map->getOccupancy(V(1.05,.05,.05))==1);
    check("obstacle_is_inflated",map->getInflateOccupancy(V(.85,.05,.05))>0);
    check("free_ray_is_free",map->getOccupancy(V(.5,.05,.05))==0);
    check("map_output_received",messages>0);check("map_source_stamps",source_stamps);check("map_frames",frames);
    check("segment_through_wall_rejected",!map->segmentCollisionFree(V(0,.05,.05),V(2,.05,.05)));
    check("free_segment_accepted",map->segmentCollisionFree(V(-2,.05,.05),V(-.5,.05,.05)));
    check("unknown_map_segment_rejected",!map->segmentCollisionFree(V(-10,0,0),V(-9,0,0)));
    const auto before=snapshot(map);
    auto wrong=cloud;wrong.header.frame_id="wrong_frame";wrong.header.stamp=node->now();cloud_pub->publish(wrong);pump(150);
    check("wrong_frame_rejected",snapshot(map)==before);
    wrong=cloud;wrong.header.stamp=rclcpp::Time(0,0,RCL_ROS_TIME);cloud_pub->publish(wrong);pump(150);
    check("zero_stamp_rejected",snapshot(map)==before);
    cloud_pub->publish(cloud);pump(150);check("duplicate_stamp_rejected",snapshot(map)==before);
    auto copied=map->snapshotForPreview();std::istringstream input(before,std::ios::binary);
    auto restored=GridMap::readPreviewSnapshot(input);
    check("preview_snapshot_roundtrip",restored && snapshot(restored)==before && snapshot(copied)==before);
    std::istringstream truncated(before.substr(0,before.size()/2),std::ios::binary);
    check("truncated_snapshot_rejected",!GridMap::readPreviewSnapshot(truncated));
    std::istringstream invalid("not-a-map",std::ios::binary);
    check("invalid_snapshot_rejected",!GridMap::readPreviewSnapshot(invalid));
    // A* uses a frozen map. Verify every segment densely, not just waypoints.
    AStar search;search.initGridMap(copied,Eigen::Vector3i(60,60,40));
    auto result=search.AstarSearch(.1,V(0,.05,.05),V(2,.05,.05));
    auto path=search.getPath();path_points=path.size();bool collision_free=true;
    for(size_t i=1;i<path.size();++i){
      const int n=std::max(1,int(std::ceil((path[i]-path[i-1]).norm()/.02)));
      for(int j=0;j<=n;++j)collision_free &= copied->getInflateOccupancy(path[i-1]+(path[i]-path[i-1])*double(j)/n)==0;
    }
    if(!collision_free){std::cerr << "AStar result=" << int(result) << " path:";for(const auto& x:path)std::cerr << " ["<< x.transpose()<<"]";std::cerr<<std::endl;}
    check("astar_routes_around_wall",result==SUCCESS && path.size()>2 && collision_free);
    check("astar_preserves_grid_endpoints",!path.empty() && (path.front()-V(0,.05,.05)).norm()<1e-6 && (path.back()-V(2,.05,.05)).norm()<1e-6);
    check("invalid_search_rejected",search.AstarSearch(0,V(0,0,0),V(1,0,0))==INIT_ERR);
    check("failed_search_clears_previous_path",search.getPath().empty());
    // Reuse native manager on the obstacle-free side of the map; no task executor exists in this bench.
    ego_planner::EGOPlannerManager manager;manager.initPlanModules(node,visual,copied);
    const V start(-2,0,0),goal(-.5,0,0),zero=V::Zero();
    const bool success=manager.reboundReplan(start,zero,zero,goal,zero,true,false,true);
    check("manager_produces_trajectory",success);
    bool finite=true,limits=true,ends=false,clear=true;
    if(success){
      auto& traj=manager.traj_.local_traj.traj;const double total=traj.getTotalDuration();
      ends=(traj.getPos(0)-start).norm()<1e-5 && (traj.getPos(total)-goal).norm()<1e-5;
      for(double t=0;t<=total;t+=.01){
        const V pos=traj.getPos(t),vel=traj.getVel(t),acc=traj.getAcc(t);
        finite &= pos.allFinite()&&vel.allFinite()&&acc.allFinite();
        limits &= vel.norm()<=1.051 && std::abs(vel.z())<=.351 && (vel.cwiseQuotient(V(1.05,1.05,.35))).norm()<=1.001 && acc.norm()<=1.501;
        clear &= copied->getInflateOccupancy(pos)==0;
      }
    }else{finite=limits=clear=false;}
    check("trajectory_endpoints",ends);check("trajectory_finite",finite);
    check("trajectory_dynamic_limits",limits);check("trajectory_clear",clear);
    exec.remove_node(node);
    flight_diag::publisher().reset();flight_diag::owner().reset();
  } catch(const std::exception& e){std::cerr<<e.what()<<std::endl;checks["exception_free"]=false;}
  rclcpp::shutdown();
  bool passed=!checks.empty();for(auto &kv:checks)passed &= kv.second;
  std::cout<<"EGO_RESULT {\"passed\":"<<(passed?"true":"false")<<",\"checks\":{";
  bool first=true;for(auto &kv:checks){if(!first)std::cout<<",";first=false;std::cout<<"\""<<kv.first<<"\":"<<(kv.second?"true":"false");}
  std::cout<<"},\"map_messages\":"<<messages<<",\"path_points\":"<<path_points<<",\"flight_commands_sent\":false}"<<std::endl;
  return passed?0:1;
}
