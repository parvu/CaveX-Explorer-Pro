#include <gtest/gtest.h>

#include <cmath>
#include <random>

#include "cavex_sic_slam/continuous_current_estimator.hpp"

using cavex_sic_slam::ContinuousCurrentEstimator;
using gtsam::Vector3;

namespace {
// Bounded, slowly-varying -- matches this project's real
// current_field_node.py profiles (constant/step/sinusoidal), never an
// unbounded drift. See plan's "Why these parameters".
Vector3 trueCurrent(double t) {
  return Vector3(
    0.3 + 0.1 * std::sin(0.05 * t),
    -0.2 + 0.15 * std::sin(0.03 * t + 1.0),
    0.0);
}
}  // namespace

TEST(ContinuousCurrentEstimator, SmoothsRegularKeyframeSamplesToWithinTolerance) {
  std::mt19937 rng(11);
  std::normal_distribution<double> noise(0.0, 0.05);
  ContinuousCurrentEstimator est(90.0, 6);

  bool any_fit = false;
  for (int i = 0; i < 60; ++i) {
    double t = i * 3.0;  // ~3s keyframe cadence
    Vector3 c = trueCurrent(t) + Vector3(noise(rng), noise(rng), noise(rng));
    est.addSample(t, c);
    if (i > 0 && i % 5 == 0) {
      if (est.refit()) {any_fit = true;}
    }
  }
  ASSERT_TRUE(any_fit) << "estimator never produced a fit";

  double max_err = 0.0;
  bool any_eval = false;
  for (double t = 90.0; t <= 177.0; t += 2.0) {
    auto c = est.evaluate(t);
    if (!c) {continue;}
    any_eval = true;
    max_err = std::max(max_err, (*c - trueCurrent(t)).norm());
  }
  ASSERT_TRUE(any_eval) << "estimator never produced an evaluable point";
  EXPECT_LT(max_err, 0.15);
}

TEST(ContinuousCurrentEstimator, EvaluateReturnsNulloptBeforeAnyFit) {
  ContinuousCurrentEstimator est(90.0, 6);
  EXPECT_FALSE(est.evaluate(0.0).has_value());
}

TEST(ContinuousCurrentEstimator, EvaluateWorksBetweenRefitsNotJustAtRefitTime) {
  // Regression test for a real bug found via live-node smoke testing: an
  // earlier version rejected any t > domain_b_ (the last sample time AT
  // the moment of the last refit), so evaluate() only ever succeeded on
  // the exact keyframe a refit happened, returning nullopt on every
  // keyframe in between -- confirmed live (the published topic only
  // fired on refit keyframes). This test replicates the live incremental
  // pattern (addSample+refit called every step, "now" always advancing)
  // instead of the other tests' single-shot fit-then-query-the-past
  // pattern, which did not catch this.
  std::mt19937 rng(5);
  std::normal_distribution<double> noise(0.0, 0.05);
  ContinuousCurrentEstimator est(90.0, 6);

  int between_refit_successes = 0;
  for (int i = 0; i < 60; ++i) {
    double t = i * 3.0;
    est.addSample(t, trueCurrent(t) + Vector3(noise(rng), noise(rng), noise(rng)));
    bool did_refit = (i > 0 && i % 5 == 0) ? est.refit() : false;
    auto smoothed = est.evaluate(t);
    if (!did_refit && smoothed.has_value()) {
      ++between_refit_successes;
    }
  }
  EXPECT_GT(between_refit_successes, 0)
    << "evaluate() never succeeded on a non-refit keyframe -- the between-refits bug is back";
}

TEST(ContinuousCurrentEstimator, EvaluateReturnsNulloptOutsideFitDomain) {
  ContinuousCurrentEstimator est(90.0, 6);
  std::mt19937 rng(3);
  std::normal_distribution<double> noise(0.0, 0.05);
  for (int i = 0; i < 20; ++i) {
    double t = i * 3.0;
    est.addSample(t, trueCurrent(t) + Vector3(noise(rng), noise(rng), noise(rng)));
  }
  ASSERT_TRUE(est.refit());
  EXPECT_FALSE(est.evaluate(10000.0).has_value());
}
