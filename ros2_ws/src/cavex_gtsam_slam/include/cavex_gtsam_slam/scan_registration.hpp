#ifndef CAVEX_GTSAM_SLAM__SCAN_REGISTRATION_HPP_
#define CAVEX_GTSAM_SLAM__SCAN_REGISTRATION_HPP_

#include <vector>

namespace cavex_gtsam_slam
{

struct ScanPoint
{
  double x;
  double y;
  double intensity_db;
};

// Converts a LaserScan-style (ranges, intensities) pair into Cartesian
// points in the sensor frame, silently skipping non-finite (dropped) beams
// -- a dropped beam carries no position information and must never be
// registered as if it were a real return.
std::vector<ScanPoint> laserScanToPoints(
  const std::vector<double> & ranges,
  const std::vector<double> & intensities,
  double angle_min,
  double angle_increment);

struct RegistrationResult
{
  bool converged;
  double dx;
  double dy;
  double dyaw;
  double mean_residual;
  double matched_fraction;
};

// 2D point-to-point ICP: finds the rigid transform (dx, dy, dyaw) such that
// applying it to `source` points best aligns them with `target` points.
// Nearest-neighbor correspondence (naive O(n*m) -- sonar scans here are
// ~64 points, a KD-tree buys nothing at this scale), correspondences beyond
// max_correspondence_dist rejected, rigid transform per iteration via 2D
// Kabsch/SVD on the matched pairs. `converged` is true iff the final
// matched_fraction exceeds 0.5 -- sparse/non-overlapping scans return
// converged=false rather than a confident but meaningless transform.
RegistrationResult registerScans(
  const std::vector<ScanPoint> & source,
  const std::vector<ScanPoint> & target,
  double max_correspondence_dist,
  int max_iterations);

}  // namespace cavex_gtsam_slam

#endif  // CAVEX_GTSAM_SLAM__SCAN_REGISTRATION_HPP_
