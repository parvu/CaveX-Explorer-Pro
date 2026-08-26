#include "cavex_gtsam_slam/scan_registration.hpp"
#include <cmath>
#include <limits>
#include <Eigen/Dense>

namespace cavex_gtsam_slam
{

std::vector<ScanPoint> laserScanToPoints(
  const std::vector<double> & ranges,
  const std::vector<double> & intensities,
  double angle_min,
  double angle_increment)
{
  std::vector<ScanPoint> points;
  points.reserve(ranges.size());
  for (std::size_t i = 0; i < ranges.size(); ++i) {
    if (!std::isfinite(ranges[i])) {
      continue;
    }
    double angle = angle_min + static_cast<double>(i) * angle_increment;
    double intensity = (i < intensities.size()) ? intensities[i] : 0.0;
    points.push_back(ScanPoint{
        ranges[i] * std::cos(angle),
        ranges[i] * std::sin(angle),
        intensity});
  }
  return points;
}

namespace
{

struct Transform2D
{
  double dx = 0.0;
  double dy = 0.0;
  double yaw = 0.0;
};

ScanPoint applyTransform(const ScanPoint & p, const Transform2D & t)
{
  double c = std::cos(t.yaw), s = std::sin(t.yaw);
  return ScanPoint{
    c * p.x - s * p.y + t.dx,
    s * p.x + c * p.y + t.dy,
    p.intensity_db};
}

// Weighted 2D Kabsch: best-fit rigid transform minimizing sum of squared
// distances between matched (src, tgt) pairs, weighted by the source
// point's echo intensity converted to a linear weight -- stronger, more
// reliable returns count for more, the honest use of the acoustic model.
Transform2D bestFitTransform(
  const std::vector<ScanPoint> & src, const std::vector<ScanPoint> & tgt)
{
  std::size_t n = src.size();
  double w_sum = 0.0;
  double src_cx = 0.0, src_cy = 0.0, tgt_cx = 0.0, tgt_cy = 0.0;
  std::vector<double> w(n);
  for (std::size_t i = 0; i < n; ++i) {
    // dB to a bounded linear weight: shift so the weakest plausible return
    // (~100 dB, the sonar's detection threshold default) maps near zero.
    w[i] = std::max(0.0, src[i].intensity_db - 100.0) + 1.0;
    w_sum += w[i];
    src_cx += w[i] * src[i].x; src_cy += w[i] * src[i].y;
    tgt_cx += w[i] * tgt[i].x; tgt_cy += w[i] * tgt[i].y;
  }
  if (w_sum < 1e-9) {
    return Transform2D{};
  }
  src_cx /= w_sum; src_cy /= w_sum;
  tgt_cx /= w_sum; tgt_cy /= w_sum;

  Eigen::Matrix2d H = Eigen::Matrix2d::Zero();
  for (std::size_t i = 0; i < n; ++i) {
    Eigen::Vector2d ps(src[i].x - src_cx, src[i].y - src_cy);
    Eigen::Vector2d pt(tgt[i].x - tgt_cx, tgt[i].y - tgt_cy);
    H += w[i] * ps * pt.transpose();
  }
  Eigen::JacobiSVD<Eigen::Matrix2d> svd(H, Eigen::ComputeFullU | Eigen::ComputeFullV);
  Eigen::Matrix2d R = svd.matrixV() * svd.matrixU().transpose();
  if (R.determinant() < 0) {
    Eigen::Matrix2d V = svd.matrixV();
    V.col(1) *= -1.0;
    R = V * svd.matrixU().transpose();
  }
  Transform2D t;
  t.yaw = std::atan2(R(1, 0), R(0, 0));
  Eigen::Vector2d centroid_tgt(tgt_cx, tgt_cy);
  Eigen::Vector2d centroid_src(src_cx, src_cy);
  Eigen::Vector2d translation = centroid_tgt - R * centroid_src;
  t.dx = translation.x();
  t.dy = translation.y();
  return t;
}

}  // namespace

RegistrationResult registerScans(
  const std::vector<ScanPoint> & source,
  const std::vector<ScanPoint> & target,
  double max_correspondence_dist,
  int max_iterations)
{
  RegistrationResult result{false, 0.0, 0.0, 0.0, 0.0, 0.0};
  if (source.empty() || target.empty()) {
    return result;
  }

  Transform2D accumulated;
  std::vector<ScanPoint> moving = source;
  double last_mean_residual = std::numeric_limits<double>::infinity();

  for (int iter = 0; iter < max_iterations; ++iter) {
    std::vector<ScanPoint> matched_src, matched_tgt;
    double residual_sum = 0.0;
    for (const auto & p : moving) {
      double best_d2 = std::numeric_limits<double>::infinity();
      std::size_t best_j = 0;
      for (std::size_t j = 0; j < target.size(); ++j) {
        double dx = p.x - target[j].x, dy = p.y - target[j].y;
        double d2 = dx * dx + dy * dy;
        if (d2 < best_d2) {
          best_d2 = d2;
          best_j = j;
        }
      }
      if (std::sqrt(best_d2) <= max_correspondence_dist) {
        matched_src.push_back(p);
        matched_tgt.push_back(target[best_j]);
        residual_sum += std::sqrt(best_d2);
      }
    }

    result.matched_fraction =
      static_cast<double>(matched_src.size()) / static_cast<double>(source.size());
    if (matched_src.empty()) {
      break;
    }
    result.mean_residual = residual_sum / static_cast<double>(matched_src.size());

    Transform2D step = bestFitTransform(matched_src, matched_tgt);
    // Compose step into the moving cloud and into the accumulated transform.
    for (auto & p : moving) {
      p = applyTransform(p, step);
    }
    double c1 = std::cos(step.yaw), s1 = std::sin(step.yaw);
    double new_dx = c1 * accumulated.dx - s1 * accumulated.dy + step.dx;
    double new_dy = s1 * accumulated.dx + c1 * accumulated.dy + step.dy;
    accumulated.dx = new_dx;
    accumulated.dy = new_dy;
    accumulated.yaw += step.yaw;

    double step_mag = std::sqrt(step.dx * step.dx + step.dy * step.dy) + std::abs(step.yaw);
    if (step_mag < 1e-6) {
      break;
    }
    if (std::abs(result.mean_residual - last_mean_residual) < 1e-6) {
      break;
    }
    last_mean_residual = result.mean_residual;
  }

  result.dx = accumulated.dx;
  result.dy = accumulated.dy;
  result.dyaw = accumulated.yaw;
  result.converged = result.matched_fraction > 0.5;
  return result;
}

}  // namespace cavex_gtsam_slam
