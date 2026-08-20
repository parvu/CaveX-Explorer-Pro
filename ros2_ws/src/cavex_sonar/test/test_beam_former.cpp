#include <gtest/gtest.h>
#include <cmath>
#include <limits>
#include <vector>
#include "cavex_sonar/beam_former.hpp"

using cavex_sonar::AcousticParams;
using cavex_sonar::beamScanGeometry;
using cavex_sonar::BeamFormerConfig;
using cavex_sonar::formBeams;
using cavex_sonar::incidenceAngleAt;

TEST(Incidence, FlatWallDirectlyAheadIsNearNormal) {
  // A wall perpendicular to the centre ray: ranges rise symmetrically as
  // 1/cos(theta) away from the centre. Incidence at the centre is ~0.
  const double angular_step = 0.01;
  std::vector<double> ranges;
  for (int i = -50; i <= 50; ++i) {
    ranges.push_back(5.0 / std::cos(i * angular_step));
  }
  EXPECT_NEAR(incidenceAngleAt(ranges, 50, angular_step), 0.0, 0.05);
}

TEST(Incidence, ObliqueSurfaceGivesLargerIncidenceThanPerpendicular) {
  const double angular_step = 0.01;
  std::vector<double> perpendicular, oblique;
  for (int i = -50; i <= 50; ++i) {
    perpendicular.push_back(5.0 / std::cos(i * angular_step));
    // A steadily receding surface: strong range gradient == oblique view.
    oblique.push_back(5.0 + 0.5 * i * angular_step * 10.0);
  }
  EXPECT_LT(incidenceAngleAt(perpendicular, 50, angular_step),
            incidenceAngleAt(oblique, 50, angular_step));
}

TEST(Incidence, IsAlwaysWithinZeroToHalfPi) {
  const double angular_step = 0.01;
  std::vector<double> ranges;
  for (int i = -50; i <= 50; ++i) {
    ranges.push_back(3.0 + 2.0 * std::sin(i * 0.3));
  }
  for (size_t i = 0; i < ranges.size(); ++i) {
    const double a = incidenceAngleAt(ranges, i, angular_step);
    EXPECT_GE(a, 0.0);
    EXPECT_LE(a, M_PI / 2.0 + 1e-9);
  }
}

TEST(FormBeams, ReducesDenseRaysToConfiguredBeamCount) {
  BeamFormerConfig cfg;
  cfg.rays_per_beam = 5;
  cfg.beam_count = 20;
  cfg.angular_step_rad = 0.01;
  AcousticParams p;
  std::vector<double> ranges(100, 4.0);
  const auto beams = formBeams(ranges, cfg, p, 7u, 0u);
  EXPECT_EQ(beams.size(), 20u);
}

TEST(FormBeams, IgnoresNonFiniteRaysInsteadOfPropagatingThem) {
  // gpu_lidar reports +inf for a ray that hits nothing. Averaging that in
  // would poison the whole beam, so out-of-range rays must be excluded.
  BeamFormerConfig cfg;
  cfg.rays_per_beam = 4;
  cfg.beam_count = 1;
  cfg.angular_step_rad = 0.01;
  AcousticParams p;
  std::vector<double> ranges = {4.0, std::numeric_limits<double>::infinity(), 4.0, 4.0};
  const auto beams = formBeams(ranges, cfg, p, 7u, 0u);
  ASSERT_EQ(beams.size(), 1u);
  ASSERT_TRUE(beams[0].detected);
  EXPECT_NEAR(beams[0].range_m, 4.0, 1e-6);
}

TEST(FormBeams, ReportsNoDetectionWhenEveryRayInABeamIsOutOfRange) {
  BeamFormerConfig cfg;
  cfg.rays_per_beam = 3;
  cfg.beam_count = 1;
  cfg.angular_step_rad = 0.01;
  AcousticParams p;
  const double inf = std::numeric_limits<double>::infinity();
  std::vector<double> ranges = {inf, inf, inf};
  const auto beams = formBeams(ranges, cfg, p, 7u, 0u);
  ASSERT_EQ(beams.size(), 1u);
  EXPECT_FALSE(beams[0].detected);
}

TEST(FormBeams, TotalNonDetectionCarriesNegativeInfinityIntensity) {
  // 0.0 looks like a valid moderate dB return; a total non-detection must be
  // unmistakable so downstream intensity weighting cannot treat "no data"
  // as "moderate signal".
  BeamFormerConfig cfg;
  cfg.rays_per_beam = 3;
  cfg.beam_count = 1;
  cfg.angular_step_rad = 0.01;
  AcousticParams p;
  const double inf = std::numeric_limits<double>::infinity();
  std::vector<double> ranges = {inf, inf, inf};
  const auto beams = formBeams(ranges, cfg, p, 7u, 0u);
  ASSERT_EQ(beams.size(), 1u);
  EXPECT_FALSE(beams[0].detected);
  EXPECT_TRUE(std::isinf(beams[0].intensity));
  EXPECT_LT(beams[0].intensity, 0.0);
}

TEST(BeamScanGeometry, BeamCentreOffsetMatchesRealFanGeometry) {
  // Fan matching the SDF: 512 dense rays across +/-60deg, 64 beams of 8 rays
  // each. Beam b's true centre is at input ray index b*8 + 3.5, not b*8.
  const double in_angle_min = -1.0472;
  const double in_angle_increment = 2.0944 / 511.0;
  const std::size_t rays_per_beam = 8;
  const std::size_t beam_count = 64;

  const auto geom = beamScanGeometry(
    in_angle_min, in_angle_increment, rays_per_beam);

  EXPECT_NEAR(geom.angle_min, in_angle_min + in_angle_increment * 3.5, 1e-9);
  EXPECT_NEAR(geom.angle_increment, in_angle_increment * 8.0, 1e-9);

  // The last beam's centre must not overshoot the input fan's far edge.
  const double in_angle_max = in_angle_min + in_angle_increment * 511.0;
  const double last_beam_bearing =
    geom.angle_min + static_cast<double>(beam_count - 1) * geom.angle_increment;
  EXPECT_LE(last_beam_bearing, in_angle_max);
}
