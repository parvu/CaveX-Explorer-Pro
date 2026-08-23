#ifndef CAVEX_SIC_SLAM__CONTINUOUS_CURRENT_ESTIMATOR_HPP_
#define CAVEX_SIC_SLAM__CONTINUOUS_CURRENT_ESTIMATOR_HPP_

#include <gtsam/basis/BasisFactors.h>
#include <gtsam/basis/Chebyshev2.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/Values.h>

#include <deque>
#include <optional>
#include <utility>
#include <vector>

namespace cavex_sic_slam
{

// Smooths a sequence of discrete, per-keyframe current-vector estimates
// (each already computed by the existing discrete CurrentFactor / iSAM2
// graph) into a continuous-time function via a GTSAM Chebyshev2 basis
// fit, refit periodically over a rolling time window.
//
// Does NOT independently re-derive current from raw measurements -- its
// input is the discrete graph's own C(i) estimates, so it cannot disagree
// with the discrete estimate at the sample points; it only fills in a
// continuous, queryable, smoothed function between and slightly beyond
// them. Does NOT touch pose (X/V) estimation at all.
class ContinuousCurrentEstimator
{
public:
  // window_seconds: how much sample history to refit over each time.
  // basis_degree: Chebyshev basis size. Must stay well below the number
  // of samples the window will hold at the caller's sample rate -- see
  // this plan's Global Constraints for the validated (window, degree)
  // pair for a realistic ~3s keyframe cadence.
  //
  // Retention (how long a sample is kept before being pruned) is
  // deliberately 2x window_seconds_, not 1x -- refitDelayed() needs
  // samples older than (newest - lag_seconds) to still be present, and
  // 1x retention alone would leave too little margin as lag_seconds
  // approaches window_seconds_. Sample density (and therefore the
  // N-vs-sample-count stability ratio validated for this estimator) is
  // unaffected -- doubling the time span at the same sample rate doubles
  // the sample count too, so the oversampling ratio stays the same or
  // improves.
  ContinuousCurrentEstimator(double window_seconds, size_t basis_degree)
  : window_seconds_(window_seconds), N_(basis_degree) {}

  void addSample(double t, const gtsam::Vector3 & current)
  {
    samples_.emplace_back(t, current);
    while (!samples_.empty() && t - samples_.front().first > 2.0 * window_seconds_) {
      samples_.pop_front();
    }
  }

  // Refits the basis over the current window. Returns false (leaving any
  // prior fit untouched) if there are too few samples for a stable fit.
  bool refit()
  {
    if (samples_.size() < N_ * 3) {
      return false;
    }
    double a = samples_.front().first;
    double b = samples_.back().first;
    if (b <= a) {
      return false;
    }

    gtsam::NonlinearFactorGraph graph;
    const gtsam::Key key = gtsam::Symbol('q', 0);
    auto sample_model = gtsam::noiseModel::Isotropic::Sigma(3, 0.15);
    for (const auto & sample : samples_) {
      graph.emplace_shared<gtsam::VectorEvaluationFactor<gtsam::Chebyshev2, 3>>(
        key, sample.second, sample_model, N_, sample.first, a, b);
    }
    // Required smoothness prior -- see Task 1's plan
    // (.superpowers/plans/2026-08-23-continuous-time-fluid-slam.md), "Why
    // the smoothness prior is required": without it, this fit is
    // noise-dominated. Assumes a bounded, slowly-varying current (see
    // this plan's Global Constraints) -- do not tighten this to fit an
    // unrealistic unbounded-drift signal.
    auto smooth_model = gtsam::noiseModel::Isotropic::Sigma(3, 0.02);
    for (double t = a; t <= b; t += 1.0) {
      graph.emplace_shared<gtsam::VectorDerivativeFactor<gtsam::Chebyshev2, 3>>(
        key, gtsam::Vector3::Zero(), smooth_model, N_, t, a, b);
    }

    gtsam::Values initial;
    initial.insert(key, gtsam::ParameterMatrix<3>(N_));
    gtsam::LevenbergMarquardtParams params;
    params.setVerbosityLM("SILENT");
    gtsam::Values result = gtsam::LevenbergMarquardtOptimizer(graph, initial, params).optimize();

    fitted_ = result.at<gtsam::ParameterMatrix<3>>(key);
    domain_a_ = a;
    domain_b_ = b;
    return true;
  }

  // Returns the smoothed current at time t, or std::nullopt if no fit
  // exists yet, t is before any data this estimator has ever seen, or the
  // fit is stale (more than one full window old -- should never happen in
  // practice since refit() is called periodically).
  //
  // Real bug found via live-node smoke testing (not caught by the
  // standalone unit test, which only ever refit once): in the live node,
  // refit() runs periodically while "now" keeps advancing every keyframe,
  // so domain_b_ (the last sample time AT the moment of the last refit)
  // is immediately behind "now" on every subsequent keyframe until the
  // next refit -- a strict `t > domain_b_` rejection meant evaluate()
  // only ever returned a value on the exact keyframe a refit happened,
  // and nullopt on every keyframe between refits (confirmed live: the
  // published topic only ever fired on refit keyframes, never in
  // between). Allowing evaluation up to one window past domain_b_
  // permits the small forward extrapolation this always needs between
  // refits (a few keyframes' worth of time, tiny relative to
  // window_seconds_) while still refusing a genuinely stale fit.
  std::optional<gtsam::Vector3> evaluate(double t) const
  {
    if (!fitted_ || t < domain_a_ || t > domain_b_ + window_seconds_) {
      return std::nullopt;
    }
    gtsam::Chebyshev2::VectorEvaluationFunctor<3> f(N_, t, domain_a_, domain_b_);
    return f(*fitted_);
  }

  // Builds a SEPARATE fit (fitted_delayed_) using only samples strictly
  // older than (newest_sample_time - lag_seconds) -- deliberately
  // withholding the most recent lag_seconds of data. Returns false
  // (leaving any prior delayed fit untouched) if fewer than N_*3 such
  // samples exist.
  //
  // This exists specifically so evaluateForFeedback() can be queried at
  // t=now via forward extrapolation of a fit built from data that's at
  // least lag_seconds old -- a forecast, not a lookup. Two earlier
  // attempts at feeding this estimator's prediction back into the
  // discrete graph as a C(curr) prior both failed (see history.txt,
  // 2026-08-23): feeding evaluate()'s raw value back double-counted
  // information (its fit isn't independent evidence of the chain it fed
  // into), and gating the QUERY time instead of the fit's input data
  // (evaluateSettled(), removed) was structurally unsatisfiable -- it
  // was always called with t=now, so requiring t to be old could never
  // pass. This method is the third, correctly-shaped attempt: gate the
  // fit's OWN data, then extrapolate forward to now.
  bool refitDelayed(double lag_seconds)
  {
    if (samples_.empty()) {
      return false;
    }
    // Bounded to window_seconds_ wide, same as refit() -- NOT "all
    // retained history up to the cutoff". Real bug found via this
    // class's own test suite: an unbounded truncated window grows
    // without limit as samples accumulate (up to 2x window_seconds_,
    // double the width validated stable for this basis degree), causing
    // a real, growing extrapolation error (not just noise -- it tracked
    // the refit cadence exactly, worse right before each refit).
    double cutoff = samples_.back().first - lag_seconds;
    double window_start = cutoff - window_seconds_;
    std::vector<std::pair<double, gtsam::Vector3>> truncated;
    for (const auto & sample : samples_) {
      if (sample.first <= cutoff && sample.first >= window_start) {
        truncated.push_back(sample);
      }
    }
    if (truncated.size() < N_ * 3) {
      return false;
    }
    double a = truncated.front().first;
    double b = truncated.back().first;
    if (b <= a) {
      return false;
    }

    gtsam::NonlinearFactorGraph graph;
    const gtsam::Key key = gtsam::Symbol('r', 0);
    auto sample_model = gtsam::noiseModel::Isotropic::Sigma(3, 0.15);
    for (const auto & sample : truncated) {
      graph.emplace_shared<gtsam::VectorEvaluationFactor<gtsam::Chebyshev2, 3>>(
        key, sample.second, sample_model, N_, sample.first, a, b);
    }
    auto smooth_model = gtsam::noiseModel::Isotropic::Sigma(3, 0.02);
    for (double t = a; t <= b; t += 1.0) {
      graph.emplace_shared<gtsam::VectorDerivativeFactor<gtsam::Chebyshev2, 3>>(
        key, gtsam::Vector3::Zero(), smooth_model, N_, t, a, b);
    }

    gtsam::Values initial;
    initial.insert(key, gtsam::ParameterMatrix<3>(N_));
    gtsam::LevenbergMarquardtParams params;
    params.setVerbosityLM("SILENT");
    gtsam::Values result = gtsam::LevenbergMarquardtOptimizer(graph, initial, params).optimize();

    fitted_delayed_ = result.at<gtsam::ParameterMatrix<3>>(key);
    domain_a_delayed_ = a;
    domain_b_delayed_ = b;
    return true;
  }

  // Queries the delayed (deliberately-stale) fit, with the same
  // forward-extrapolation allowance evaluate() uses relative to its own
  // domain. This is the method to call for feeding a prediction back
  // into the same system that produced this estimator's samples --
  // never evaluate() (see refitDelayed()'s comment for why).
  std::optional<gtsam::Vector3> evaluateForFeedback(double t) const
  {
    if (!fitted_delayed_ || t < domain_a_delayed_ || t > domain_b_delayed_ + window_seconds_) {
      return std::nullopt;
    }
    gtsam::Chebyshev2::VectorEvaluationFunctor<3> f(N_, t, domain_a_delayed_, domain_b_delayed_);
    return f(*fitted_delayed_);
  }

private:
  double window_seconds_;
  size_t N_;
  std::deque<std::pair<double, gtsam::Vector3>> samples_;
  std::optional<gtsam::ParameterMatrix<3>> fitted_;
  double domain_a_ = 0.0, domain_b_ = 0.0;
  std::optional<gtsam::ParameterMatrix<3>> fitted_delayed_;
  double domain_a_delayed_ = 0.0, domain_b_delayed_ = 0.0;
};

}  // namespace cavex_sic_slam

#endif  // CAVEX_SIC_SLAM__CONTINUOUS_CURRENT_ESTIMATOR_HPP_
