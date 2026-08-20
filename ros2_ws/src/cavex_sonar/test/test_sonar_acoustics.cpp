#include <gtest/gtest.h>
#include <cmath>
#include "cavex_sonar/sonar_acoustics.hpp"

using cavex_sonar::AcousticParams;
using cavex_sonar::transmissionLossDb;

TEST(TransmissionLoss, GrowsWithRange) {
  AcousticParams p;
  EXPECT_LT(transmissionLossDb(1.0, p), transmissionLossDb(10.0, p));
  EXPECT_LT(transmissionLossDb(10.0, p), transmissionLossDb(30.0, p));
}

TEST(TransmissionLoss, SphericalSpreadingDominatesAtShortRange) {
  // With absorption zeroed, loss must be exactly 20*log10(r).
  AcousticParams p;
  p.absorption_db_per_m = 0.0;
  EXPECT_NEAR(transmissionLossDb(10.0, p), 20.0, 1e-9);
  EXPECT_NEAR(transmissionLossDb(100.0, p), 40.0, 1e-9);
}

TEST(TransmissionLoss, AbsorptionAddsLinearTerm) {
  AcousticParams p;
  p.absorption_db_per_m = 0.5;
  const double expected = 20.0 * std::log10(10.0) + 0.5 * 10.0;
  EXPECT_NEAR(transmissionLossDb(10.0, p), expected, 1e-9);
}

TEST(TransmissionLoss, ZeroRangeIsFiniteNotNegativeInfinity) {
  // log10(0) is -inf; the implementation must floor the range so a
  // zero-range return cannot poison downstream arithmetic with NaN.
  AcousticParams p;
  EXPECT_TRUE(std::isfinite(transmissionLossDb(0.0, p)));
}

using cavex_sonar::backscatterDb;
using cavex_sonar::echoLevelDb;

TEST(Backscatter, NormalIncidenceIsStrongest) {
  AcousticParams p;
  // 0 rad == beam perpendicular to the surface == strongest return.
  EXPECT_GT(backscatterDb(0.0, p), backscatterDb(0.6, p));
  EXPECT_GT(backscatterDb(0.6, p), backscatterDb(1.2, p));
}

TEST(Backscatter, DecreasesMonotonicallyWithIncidence) {
  AcousticParams p;
  double prev = backscatterDb(0.0, p);
  for (double a = 0.05; a < 1.55; a += 0.05) {
    const double cur = backscatterDb(a, p);
    EXPECT_LE(cur, prev) << "backscatter must not increase at incidence " << a;
    prev = cur;
  }
}

TEST(Backscatter, GrazingIncidenceIsFiniteNotNegativeInfinity) {
  AcousticParams p;
  // cos(pi/2) == 0; log10(0) is -inf. Must be floored, or a grazing beam
  // poisons the echo level with NaN instead of simply being a weak return.
  EXPECT_TRUE(std::isfinite(backscatterDb(M_PI / 2.0, p)));
}

TEST(EchoLevel, FallsOffWithBothRangeAndIncidence) {
  AcousticParams p;
  const double near_normal = echoLevelDb(2.0, 0.0, p);
  const double far_normal = echoLevelDb(20.0, 0.0, p);
  const double near_grazing = echoLevelDb(2.0, 1.4, p);
  EXPECT_GT(near_normal, far_normal) << "range must reduce echo level";
  EXPECT_GT(near_normal, near_grazing) << "incidence must reduce echo level";
}

#include <cstdint>
using cavex_sonar::BeamReturn;
using cavex_sonar::applySpeckleAndThreshold;

TEST(Speckle, IsReproducibleForAFixedSeed) {
  AcousticParams p;
  const BeamReturn a = applySpeckleAndThreshold(5.0, 0.2, p, 42u, 7u);
  const BeamReturn b = applySpeckleAndThreshold(5.0, 0.2, p, 42u, 7u);
  EXPECT_EQ(a.detected, b.detected);
  EXPECT_DOUBLE_EQ(a.intensity, b.intensity);
  EXPECT_DOUBLE_EQ(a.range_m, b.range_m);
}

TEST(Speckle, DiffersBetweenBeamsWithinTheSameSeed) {
  AcousticParams p;
  // Independent speckle per beam; identical values across beams would mean
  // the beam index is not reaching the generator.
  const BeamReturn a = applySpeckleAndThreshold(5.0, 0.2, p, 42u, 1u);
  const BeamReturn b = applySpeckleAndThreshold(5.0, 0.2, p, 42u, 2u);
  EXPECT_NE(a.intensity, b.intensity);
}

TEST(Threshold, StrongNearNormalReturnIsDetected) {
  AcousticParams p;
  const BeamReturn r = applySpeckleAndThreshold(2.0, 0.0, p, 1u, 0u);
  EXPECT_TRUE(r.detected);
  EXPECT_NEAR(r.range_m, 2.0, 1e-9) << "a detected beam reports its true range";
}

TEST(Threshold, VeryWeakReturnDropsOutRatherThanReportingAWrongRange) {
  AcousticParams p;
  // Force everything below threshold: a dropout must report detected=false,
  // never a confident wrong range. Reporting a bogus range here would inject
  // false constraints straight into the SLAM factor graph.
  p.detection_threshold_db = 1e9;
  const BeamReturn r = applySpeckleAndThreshold(2.0, 0.0, p, 1u, 0u);
  EXPECT_FALSE(r.detected);
}

TEST(Threshold, DropoutRateRisesWithIncidenceAcrossManyBeams) {
  AcousticParams p;
  p.detection_threshold_db = 120.0;
  int near_normal_hits = 0, grazing_hits = 0;
  for (uint32_t i = 0; i < 500; ++i) {
    if (applySpeckleAndThreshold(8.0, 0.05, p, 99u, i).detected) {++near_normal_hits;}
    if (applySpeckleAndThreshold(8.0, 1.45, p, 99u, i).detected) {++grazing_hits;}
  }
  EXPECT_GT(near_normal_hits, grazing_hits)
      << "grazing beams must drop out more often than near-normal ones";
}
