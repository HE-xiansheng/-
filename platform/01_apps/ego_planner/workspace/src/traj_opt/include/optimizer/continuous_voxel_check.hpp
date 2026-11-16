#pragma once
// ROS-free conservative quintic/voxel collision check shared by isolated optimizer and tests.
// Bernstein convex hull encloses every point; de Casteljau refines ambiguous hulls.
// Unresolved contact/numerical/budget cases reject; no finite time sampling implies "free".
#include <Eigen/Core>
#include <algorithm>
#include <cmath>
#include <functional>
#include <limits>
namespace sim_collision {
using Coeff = Eigen::Matrix<double,3,6>;
inline double choose(int n,int k) {
  double v=1;for(int i=1;i<=k;++i)v=v*(n-k+i)/i;return v;
}
inline Coeff bernstein(const Coeff& descending,double duration) {
  Coeff b=Coeff::Zero();
  for(int i=0;i<6;++i)for(int k=0;k<=i;++k)
    b.col(i)+=descending.col(5-k)*std::pow(duration,k)*choose(i,k)/choose(5,k);
  return b;
}
inline void split(const Coeff& b,Coeff& left,Coeff& right) {
  Coeff w=b;left.col(0)=w.col(0);right.col(5)=w.col(5);
  for(int j=1;j<=5;++j) {
    for(int i=0;i<=5-j;++i)w.col(i)=.5*(w.col(i)+w.col(i+1));
    left.col(j)=w.col(0);right.col(5-j)=w.col(5-j);
  }
}
inline bool recurse(const Coeff& b,double resolution,const std::function<bool(const Eigen::Vector3d&)>& occupied,int depth,int& budget) {
  if(--budget<0 || !b.allFinite())return false;
  const Eigen::Vector3d low=b.rowwise().minCoeff(),high=b.rowwise().maxCoeff();
  // Include both cells adjacent to exact grid boundaries; contact is conservative collision.
  const Eigen::Array3d il=((low.array()-1e-10)/resolution).floor(),ih=((high.array()+1e-10)/resolution).floor();
  if((il.abs()>1e8).any() || (ih.abs()>1e8).any())return false;
  const Eigen::Array3d spans=ih-il+1;
  if(spans.prod()<=64) {
    bool maybe=false;
    for(int x=int(il.x());x<=int(ih.x());++x)
      for(int y=int(il.y());y<=int(ih.y());++y)
        for(int z=int(il.z());z<=int(ih.z());++z)
          if(occupied(Eigen::Vector3d(x+.5,y+.5,z+.5)*resolution))maybe=true;
    if(!maybe)return true;
  }
  Coeff left,right;split(b,left,right);
  if(occupied(b.col(0)) || occupied(b.col(5)) || occupied(left.col(5)))return false;
  if(depth>=28 || (high-low).norm()<1e-7)return false;
  return recurse(left,resolution,occupied,depth+1,budget) && recurse(right,resolution,occupied,depth+1,budget);
}
inline bool collisionFree(const Coeff& descending,double duration,double resolution,const std::function<bool(const Eigen::Vector3d&)>& occupied) {
  if(!(duration>0) || !std::isfinite(duration) || !(resolution>0) || !std::isfinite(resolution))return false;
  int budget=100000;return recurse(bernstein(descending,duration),resolution,occupied,0,budget);
}
}
