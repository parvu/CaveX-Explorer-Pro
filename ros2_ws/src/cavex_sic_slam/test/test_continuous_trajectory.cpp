#include <gtest/gtest.h>

#include <gtsam/basis/BasisFactors.h>
#include <gtsam/basis/Chebyshev2.h>
#include <gtsam/base/numericalDerivative.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/Values.h>

#include <algorithm>
#include <cmath>
#include <random>

#include "cavex_sic_slam/velocity_coupling_factor.hpp"

using namespace gtsam;
using cavex_sic_slam::VelocityCouplingFactor;

namespace {
// Ground truth used by both tests below: a curved trajectory and a
// smoothly time-varying current, both nontrivial enough that a wrong
// coupling sign or Jacobian would visibly fail to converge/verify.
Vector3 truePosition(double t) {
  return Vector3(2.0 * t + 0.3 * std::sin(t), 1.5 * std::sin(0.5 * t), -0.1 * t);
}
Vector3 trueVelocity(double t) {
  return Vector3(2.0 + 0.3 * std::cos(t), 0.75 * std::cos(0.5 * t), -0.1);
}
Vector3 trueCurrent(double t) {
  return Vector3(0.3 + 0.1 * std::sin(0.3 * t), -0.05 * t, 0.0);
}
}  // namespace

TEST(VelocityCouplingFactor, JacobiansMatchNumericalDerivative) {
  const size_t N = 15;
  const double t = 7.3, a = 0.0, b = 20.0;
  const Vector3 measuredVelocity(0.1, -0.2, 0.05);
  auto model = noiseModel::Isotropic::Sigma(3, 0.02);
  VelocityCouplingFactor factor(1, 2, N, t, a, b, measuredVelocity, model);

  Eigen::Matrix<double, 3, -1> pm(3, N), cm(3, N);
  for (size_t i = 0; i < N; ++i) {
    pm.col(i) = Vector3(0.1 * i, -0.05 * i, 0.02 * i);
    cm.col(i) = Vector3(0.01 * i, 0.02, -0.01 * i);
  }
  ParameterMatrix<3> posv(pm), curv(cm);

  Matrix H1, H2;
  factor.evaluateError(posv, curv, H1, H2);

  auto f = [&](const ParameterMatrix<3> &p, const ParameterMatrix<3> &c) {
    return factor.evaluateError(p, c);
  };
  constexpr int Dim = 3 * 15;  // 3 (M) * N -- must match N above
  Matrix numH1 = numericalDerivative21<Vector, ParameterMatrix<3>, ParameterMatrix<3>, Dim>(f, posv, curv);
  Matrix numH2 = numericalDerivative22<Vector, ParameterMatrix<3>, ParameterMatrix<3>, Dim>(f, posv, curv);

  EXPECT_LT((H1 - numH1).array().abs().maxCoeff(), 1e-8);
  EXPECT_LT((H2 - numH2).array().abs().maxCoeff(), 1e-8);
}

TEST(ContinuousTrajectory, JointlyRecoversPositionAndCurrentFromSparseIrregularPings) {
  const double T0 = 0.0, T1 = 20.0;
  // See CMakeLists/plan Global Constraints: N must stay well below the
  // measurement count for a stable unregularized polynomial fit.
  const size_t N = 15;
  const Key posKey = Symbol('p', 0);
  const Key curKey = Symbol('c', 0);

  std::mt19937 rng(7);
  std::normal_distribution<double> posNoise(0.0, 0.05);
  std::normal_distribution<double> velNoise(0.0, 0.02);
  std::uniform_real_distribution<double> timeDist(T0, T1);

  NonlinearFactorGraph graph;
  auto posModel = noiseModel::Isotropic::Sigma(3, 0.05);
  auto velModel = noiseModel::Isotropic::Sigma(3, 0.02);

  // Irregularly-timed position pings -- NOT a fixed grid, the whole point
  // of continuous time. 300 over 20s is a plausible per-beam sonar rate.
  for (int i = 0; i < 300; ++i) {
    double t = timeDist(rng);
    Vector3 z = truePosition(t) + Vector3(posNoise(rng), posNoise(rng), posNoise(rng));
    graph.emplace_shared<VectorEvaluationFactor<Chebyshev2, 3>>(posKey, z, posModel, N, t, T0, T1);
  }

  // Irregularly-timed IMU-style body velocity measurements, coupling the
  // trajectory's derivative to the current field. 300 over 20s (15Hz) is
  // deliberately still sparse relative to a real IMU (100-400Hz) -- this
  // is the harder case.
  for (int i = 0; i < 300; ++i) {
    double t = timeDist(rng);
    Vector3 zvel = (trueVelocity(t) - trueCurrent(t)) +
                   Vector3(velNoise(rng), velNoise(rng), velNoise(rng));
    graph.emplace_shared<VelocityCouplingFactor>(posKey, curKey, N, t, T0, T1, zvel, velModel);
  }

  // Required smoothness prior -- see plan's "Why the smoothness prior is
  // required". Not tunable away: removing this makes the test fail.
  for (double t = T0; t <= T1; t += 1.0) {
    graph.emplace_shared<VectorDerivativeFactor<Chebyshev2, 3>>(
        curKey, Vector3(0, 0, 0), noiseModel::Isotropic::Sigma(3, 0.02), N, t, T0, T1);
  }

  Values initial;
  initial.insert(posKey, ParameterMatrix<3>(N));
  initial.insert(curKey, ParameterMatrix<3>(N));

  LevenbergMarquardtParams params;
  params.setVerbosityLM("SILENT");
  Values result = LevenbergMarquardtOptimizer(graph, initial, params).optimize();

  auto posResult = result.at<ParameterMatrix<3>>(posKey);
  auto curResult = result.at<ParameterMatrix<3>>(curKey);

  // Verify at times NEVER used as a measurement -- this tests real
  // continuous interpolation/extrapolation, not just curve-fitting error
  // at the sample points.
  double maxPosErr = 0.0, maxCurErr = 0.0;
  for (double t = T0; t <= T1; t += 0.37) {
    Chebyshev2::VectorEvaluationFunctor<3> posEval(N, t, T0, T1);
    Chebyshev2::VectorEvaluationFunctor<3> curEval(N, t, T0, T1);
    maxPosErr = std::max(maxPosErr, (posEval(posResult) - truePosition(t)).norm());
    maxCurErr = std::max(maxCurErr, (curEval(curResult) - trueCurrent(t)).norm());
  }

  EXPECT_LT(maxPosErr, 0.15) << "position trajectory did not converge";
  EXPECT_LT(maxCurErr, 0.10) << "current field did not converge";
}
