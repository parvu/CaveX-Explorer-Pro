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
using cavex_sonar::applyClutterToEmptyBeam;

TEST(Speckle, IsReproducibleForAFixedSeed) {
  AcousticParams p;
  const BeamReturn a = applySpeckleAndThreshold(5.0, 0.2, p, 42u, 7u, 3u);
  const BeamReturn b = applySpeckleAndThreshold(5.0, 0.2, p, 42u, 7u, 3u);
  EXPECT_EQ(a.detected, b.detected);
  EXPECT_DOUBLE_EQ(a.intensity, b.intensity);
  EXPECT_DOUBLE_EQ(a.range_m, b.range_m);
}

TEST(Speckle, DiffersBetweenBeamsWithinTheSameSeed) {
  AcousticParams p;
  // Independent speckle per beam; identical values across beams would mean
  // the beam index is not reaching the generator.
  const BeamReturn a = applySpeckleAndThreshold(5.0, 0.2, p, 42u, 1u, 0u);
  const BeamReturn b = applySpeckleAndThreshold(5.0, 0.2, p, 42u, 2u, 0u);
  EXPECT_NE(a.intensity, b.intensity);
}

TEST(Speckle, DiffersBetweenPingsForTheSameBeam) {
  // Without ping_index reaching the generator, beam b would draw the
  // identical Rayleigh sample on every scan forever -- a static per-beam
  // bias rather than speckle that averages out over time.
  AcousticParams p;
  const BeamReturn ping0 = applySpeckleAndThreshold(5.0, 0.2, p, 42u, 7u, 0u);
  const BeamReturn ping1 = applySpeckleAndThreshold(5.0, 0.2, p, 42u, 7u, 1u);
  EXPECT_NE(ping0.intensity, ping1.intensity);
}

TEST(Speckle, IsReproducibleForARepeatedPingIndex) {
  // Same (seed, beam_index, ping_index) must still give bit-identical
  // output -- determinism is not sacrificed by adding the ping term.
  AcousticParams p;
  const BeamReturn a = applySpeckleAndThreshold(5.0, 0.2, p, 42u, 7u, 5u);
  const BeamReturn b = applySpeckleAndThreshold(5.0, 0.2, p, 42u, 7u, 5u);
  EXPECT_DOUBLE_EQ(a.intensity, b.intensity);
}

TEST(Threshold, StrongNearNormalReturnIsDetected) {
  AcousticParams p;
  const BeamReturn r = applySpeckleAndThreshold(2.0, 0.0, p, 1u, 0u, 0u);
  EXPECT_TRUE(r.detected);
  EXPECT_NEAR(r.range_m, 2.0, 1e-9) << "a detected beam reports its true range";
}

TEST(Threshold, VeryWeakReturnDropsOutRatherThanReportingAWrongRange) {
  AcousticParams p;
  // Force everything below threshold: a dropout must report detected=false,
  // never a confident wrong range. Reporting a bogus range here would inject
  // false constraints straight into the SLAM factor graph.
  p.detection_threshold_db = 1e9;
  const BeamReturn r = applySpeckleAndThreshold(2.0, 0.0, p, 1u, 0u, 0u);
  EXPECT_FALSE(r.detected);
}

TEST(Threshold, DropoutRateRisesWithIncidenceAcrossManyBeams) {
  AcousticParams p;
  p.detection_threshold_db = 120.0;
  int near_normal_hits = 0, grazing_hits = 0;
  for (uint32_t i = 0; i < 500; ++i) {
    if (applySpeckleAndThreshold(8.0, 0.05, p, 99u, i, 0u).detected) {++near_normal_hits;}
    if (applySpeckleAndThreshold(8.0, 1.45, p, 99u, i, 0u).detected) {++grazing_hits;}
  }
  EXPECT_GT(near_normal_hits, grazing_hits)
      << "grazing beams must drop out more often than near-normal ones";
}

TEST(Clutter, DisabledByDefaultPreservesOldDropoutBehavior) {
  AcousticParams p;
  p.detection_threshold_db = 1e9;  // force a real dropout
  // clutter_probability defaults to 0.0 -- must reproduce the exact old
  // "no wrong range on a dropout" guarantee with no config change.
  const BeamReturn r = applySpeckleAndThreshold(2.0, 0.0, p, 1u, 0u, 0u);
  EXPECT_FALSE(r.detected);
}

TEST(Clutter, CertainProbabilityInjectsAFalseShortRangeReturn) {
  AcousticParams p;
  p.detection_threshold_db = 1e9;  // real echo always drops out
  p.clutter_probability = 1.0;     // every dropout becomes clutter
  p.clutter_max_range_m = 3.0;
  const BeamReturn r = applySpeckleAndThreshold(20.0, 0.0, p, 1u, 0u, 0u);
  EXPECT_TRUE(r.detected);
  EXPECT_GE(r.range_m, p.min_range_m);
  EXPECT_LE(r.range_m, p.clutter_max_range_m);
  // Clutter must never masquerade as the real (far) range it replaced.
  EXPECT_LT(r.range_m, 20.0);
}

TEST(Clutter, NeverOverridesARealDetection) {
  AcousticParams p;
  p.clutter_probability = 1.0;  // maximally aggressive -- still must not fire
  // Strong near-normal return at close range: comfortably above threshold.
  const BeamReturn r = applySpeckleAndThreshold(2.0, 0.0, p, 1u, 0u, 0u);
  ASSERT_TRUE(r.detected);
  EXPECT_NEAR(r.range_m, 2.0, 1e-9)
      << "a real detection's range must never be overwritten by clutter logic";
}

TEST(Clutter, RateAcrossManyBeamsMatchesConfiguredProbability) {
  AcousticParams p;
  p.detection_threshold_db = 1e9;  // every beam starts as a dropout
  p.clutter_probability = 0.3;
  int hits = 0;
  const int trials = 2000;
  for (int i = 0; i < trials; ++i) {
    if (applySpeckleAndThreshold(9.0, 0.0, p, 7u, static_cast<uint32_t>(i), 0u).detected) {
      ++hits;
    }
  }
  const double rate = static_cast<double>(hits) / trials;
  EXPECT_NEAR(rate, 0.3, 0.05);
}

TEST(ClutterDrift, ZeroDriftMatchesNoCurrentBehavior) {
  AcousticParams p;
  p.detection_threshold_db = 1e9;
  p.clutter_probability = 1.0;
  p.current_drift_range_m = 0.0;  // explicit no-op, same as the default
  const BeamReturn r = applySpeckleAndThreshold(20.0, 0.0, p, 1u, 0u, 0u, /*beam_angle_rad=*/0.7);
  EXPECT_TRUE(r.detected);
  EXPECT_GE(r.range_m, p.min_range_m);
  EXPECT_LE(r.range_m, p.clutter_max_range_m);
}

TEST(ClutterDrift, UpstreamBeamGetsCloserClutterThanDownstream) {
  // Same seed/beam/ping (so the SAME base random range is drawn both
  // times) but different beam_angle_rad relative to current_direction_rad
  // -- isolates the directional bias from the base randomness.
  AcousticParams p;
  p.detection_threshold_db = 1e9;
  p.clutter_probability = 1.0;
  p.clutter_max_range_m = 5.0;
  p.current_direction_rad = 0.0;  // current flows toward world +X
  p.current_drift_range_m = 2.0;  // strong, obvious drift for the test

  const double downstream_angle = 0.0;         // looking WITH the flow
  const double upstream_angle = M_PI;           // looking INTO the flow
  const BeamReturn down = applySpeckleAndThreshold(
    20.0, 0.0, p, 3u, 0u, 0u, downstream_angle);
  const BeamReturn up = applySpeckleAndThreshold(
    20.0, 0.0, p, 3u, 0u, 0u, upstream_angle);
  ASSERT_TRUE(down.detected);
  ASSERT_TRUE(up.detected);
  EXPECT_LT(up.range_m, down.range_m)
      << "a beam looking upstream (into where the current comes from) must "
      << "see closer clutter than one looking downstream, for the same "
      << "drift magnitude";
}

TEST(ClutterDrift, StaysClampedToClutterRangeEvenWithHugeDrift) {
  AcousticParams p;
  p.detection_threshold_db = 1e9;
  p.clutter_probability = 1.0;
  p.clutter_max_range_m = 4.0;
  p.current_direction_rad = 0.0;
  p.current_drift_range_m = 1000.0;  // unrealistically large, real request:
                                     // must not blow past the clamp
  const BeamReturn r = applySpeckleAndThreshold(20.0, 0.0, p, 5u, 0u, 0u, /*beam_angle_rad=*/0.0);
  ASSERT_TRUE(r.detected);
  EXPECT_GE(r.range_m, p.min_range_m);
  EXPECT_LE(r.range_m, p.clutter_max_range_m);
}

TEST(ClutterDrift, EmptyBeamGetsSameClutterChanceAsWeakReturn) {
  // Real fix, 2026-08-22: a beam with zero valid rays previously never
  // got a clutter chance at all -- applyClutterToEmptyBeam must give it
  // the same treatment as applySpeckleAndThreshold's own dropout path.
  AcousticParams p;
  p.clutter_probability = 1.0;
  p.clutter_max_range_m = 3.0;
  const BeamReturn r = applyClutterToEmptyBeam(p, 9u, 0u, 0u, /*beam_angle_rad=*/0.0);
  EXPECT_TRUE(r.detected);
  EXPECT_GE(r.range_m, p.min_range_m);
  EXPECT_LE(r.range_m, p.clutter_max_range_m);
}

TEST(ClutterDrift, EmptyBeamDisabledByDefaultStaysANonDetection) {
  AcousticParams p;  // clutter_probability defaults to 0.0
  const BeamReturn r = applyClutterToEmptyBeam(p, 9u, 0u, 0u);
  EXPECT_FALSE(r.detected);
  EXPECT_FALSE(std::isfinite(r.intensity));
}
