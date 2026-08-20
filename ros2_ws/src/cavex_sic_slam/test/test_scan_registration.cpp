#include <gtest/gtest.h>
#include <cmath>
#include <vector>
#include "cavex_sic_slam/scan_registration.hpp"

using namespace cavex_sic_slam;

TEST(ScanRegistration, LaserScanToPointsSkipsNonFiniteRanges) {
  std::vector<double> ranges{5.0, std::numeric_limits<double>::infinity(), 3.0};
  std::vector<double> intensities{150.0, 0.0, 140.0};
  auto pts = laserScanToPoints(ranges, intensities, -0.1, 0.1);
  ASSERT_EQ(pts.size(), 2u);
  EXPECT_NEAR(pts[0].x, 5.0 * std::cos(-0.1), 1e-9);
  EXPECT_NEAR(pts[0].y, 5.0 * std::sin(-0.1), 1e-9);
  EXPECT_NEAR(pts[1].x, 3.0 * std::cos(0.1), 1e-9);
  EXPECT_EQ(pts[1].intensity_db, 140.0);
}

TEST(ScanRegistration, IdenticalScansRegisterToZeroTransform) {
  std::vector<ScanPoint> cloud;
  for (int i = 0; i < 32; ++i) {
    double a = -1.0 + i * 0.06;
    cloud.push_back(ScanPoint{5.0 * std::cos(a), 5.0 * std::sin(a), 150.0});
  }
  auto result = registerScans(cloud, cloud, 0.5, 20);
  EXPECT_TRUE(result.converged);
  EXPECT_NEAR(result.dx, 0.0, 1e-4);
  EXPECT_NEAR(result.dy, 0.0, 1e-4);
  EXPECT_NEAR(result.dyaw, 0.0, 1e-4);
  EXPECT_NEAR(result.mean_residual, 0.0, 1e-4);
  EXPECT_GT(result.matched_fraction, 0.9);
}

TEST(ScanRegistration, RecoversKnownTranslation) {
  std::vector<ScanPoint> target;
  for (int i = 0; i < 32; ++i) {
    double a = -1.0 + i * 0.06;
    target.push_back(ScanPoint{5.0 * std::cos(a), 5.0 * std::sin(a), 150.0});
  }
  // source is target shifted by (+0.3, -0.1): a source point at p_src
  // corresponds to a target point at p_src + (0.3, -0.1), so registering
  // source onto target should recover dx=+0.3, dy=-0.1 (source pose moved
  // by that much relative to target's frame).
  std::vector<ScanPoint> source;
  for (const auto & p : target) {
    source.push_back(ScanPoint{p.x - 0.3, p.y + 0.1, p.intensity_db});
  }
  auto result = registerScans(source, target, 1.0, 30);
  EXPECT_TRUE(result.converged);
  EXPECT_NEAR(result.dx, 0.3, 0.02);
  EXPECT_NEAR(result.dy, -0.1, 0.02);
  EXPECT_NEAR(result.dyaw, 0.0, 0.02);
}

TEST(ScanRegistration, RecoversKnownRotation) {
  std::vector<ScanPoint> target;
  for (int i = 0; i < 40; ++i) {
    double a = -1.2 + i * 0.06;
    target.push_back(ScanPoint{6.0 * std::cos(a), 6.0 * std::sin(a), 150.0});
  }
  double true_yaw = 0.02;  // radians
  std::vector<ScanPoint> source;
  double c = std::cos(-true_yaw), s = std::sin(-true_yaw);
  for (const auto & p : target) {
    source.push_back(ScanPoint{c * p.x - s * p.y, s * p.x + c * p.y, p.intensity_db});
  }
  auto result = registerScans(source, target, 1.0, 30);
  EXPECT_TRUE(result.converged);
  EXPECT_NEAR(result.dyaw, true_yaw, 0.01);
}

TEST(ScanRegistration, SparseNonOverlappingScansDoNotFalselyConverge) {
  std::vector<ScanPoint> target{{0.0, 0.0, 150.0}};
  std::vector<ScanPoint> source{{50.0, 50.0, 150.0}};
  auto result = registerScans(source, target, 0.5, 20);
  EXPECT_LT(result.matched_fraction, 0.5);
}

// Regression test for a real accumulated-translation composition bug: the
// per-iteration step must rotate the OLD accumulated translation by the
// NEW step's yaw (and add the new step's own translation), not rotate the
// new step's translation by the stale old accumulated yaw. A single
// dominant ICP iteration (like RecoversKnownTranslation/RecoversKnownRotation
// above) never exercises this, because with only one nonzero-yaw step the
// two composition orders coincide. This fixture -- a dense 60-point arc
// registered under a combined rotation+translation -- was verified offline
// (via an independent Python re-implementation of this exact algorithm) to
// take 3 iterations with a nonzero (> 1e-4 rad) yaw step each, which is
// what actually exercises the bug: under the old (buggy) composition this
// converges to dx/dy off by roughly 7-8e-4 from the true values; under the
// fix it matches to within floating point noise (~1e-13).
TEST(ScanRegistration, ComposesMultipleRotationStepsCorrectly) {
  std::vector<ScanPoint> target;
  for (int i = 0; i < 60; ++i) {
    double a = -1.5 + i * 0.05;
    target.push_back(ScanPoint{8.0 * std::cos(a), 8.0 * std::sin(a), 150.0});
  }
  // True composed transform: target = R(true_yaw) * source + (true_dx, true_dy).
  double true_yaw = 0.02;
  double true_dx = 0.15;
  double true_dy = -0.05;
  double c = std::cos(true_yaw), s = std::sin(true_yaw);
  std::vector<ScanPoint> source;
  for (const auto & p : target) {
    // Invert: source = R(true_yaw)^T * (target - t).
    double tx = p.x - true_dx;
    double ty = p.y - true_dy;
    source.push_back(ScanPoint{c * tx + s * ty, -s * tx + c * ty, p.intensity_db});
  }
  auto result = registerScans(source, target, 1.0, 30);
  EXPECT_TRUE(result.converged);
  EXPECT_NEAR(result.dyaw, true_yaw, 1e-4);
  EXPECT_NEAR(result.dx, true_dx, 1e-4);
  EXPECT_NEAR(result.dy, true_dy, 1e-4);
}
