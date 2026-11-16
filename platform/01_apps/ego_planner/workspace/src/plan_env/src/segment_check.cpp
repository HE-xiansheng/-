#include <plan_env/grid_map.h>

// Closed segment / closed occupied-cell intersection. Checking only the next
// A* vertex can cut diagonally through an inflated obstacle corner.
bool GridMap::segmentCollisionFree(const Eigen::Vector3d &a, const Eigen::Vector3d &b)
{
  if (!a.allFinite() || !b.allFinite() || !mp_.have_initialized_ ||
      !isInBuf(a) || !isInBuf(b)) return false;
  const double resolution = mp_.resolution_;
  if (!std::isfinite(resolution) || resolution <= 0) return false;
  const Eigen::Array3d low = (a.cwiseMin(b).array() / resolution - 1e-10).floor();
  const Eigen::Array3d high = (a.cwiseMax(b).array() / resolution + 1e-10).floor();
  const Eigen::Array3d span = high - low + 1;
  if ((low.abs() > 1e8).any() || (high.abs() > 1e8).any() ||
      span.prod() > 4096) return false;
  const Eigen::Vector3d d = b - a;
  for (int x=int(low.x()); x<=int(high.x()); ++x)
    for (int y=int(low.y()); y<=int(high.y()); ++y)
      for (int z=int(low.z()); z<=int(high.z()); ++z) {
        const Eigen::Vector3d cell(x,y,z);
        if (!getInflateOccupancy((cell.array()+.5).matrix()*resolution)) continue;
        const Eigen::Vector3d lower = cell*resolution;
        const Eigen::Vector3d upper = (cell.array()+1).matrix()*resolution;
        double enter=0, leave=1;
        bool intersects=true;
        for (int axis=0; axis<3; ++axis) {
          if (std::abs(d[axis]) < 1e-12) {
            if (a[axis]<lower[axis]-1e-12 || a[axis]>upper[axis]+1e-12) { intersects=false; break; }
          } else {
            double t0=(lower[axis]-a[axis])/d[axis], t1=(upper[axis]-a[axis])/d[axis];
            if (t0>t1) std::swap(t0,t1);
            enter=std::max(enter,t0); leave=std::min(leave,t1);
            if (enter>leave+1e-12) { intersects=false; break; }
          }
        }
        if (intersects) return false;
      }
  return true;
}
