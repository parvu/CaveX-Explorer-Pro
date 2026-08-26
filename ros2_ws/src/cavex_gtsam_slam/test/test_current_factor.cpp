#include <gtest/gtest.h>
#include <gtsam/base/numericalDerivative.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/geometry/Rot3.h>
#include "cavex_gtsam_slam/current_factor.hpp"
#include "cavex_gtsam_slam/dynamics_model.hpp"

using namespace cavex_gtsam_slam;
using gtsam::Pose3;
using gtsam::Rot3;
using gtsam::Point3;
using gtsam::Vector3;

namespace
{
gtsam::Vector evalErrorWrapper(
  const CurrentFactor & f, const Pose3 & X, const Vector3 & V, const Vector3 & C)
{
  return f.evaluateError(X, V, C);
}
}  // namespace

TEST(CurrentFactor, ZeroCurrentZeroResidualWhenVelocityMatchesPrediction) {
  auto geom = defaultBlueRov2Geometry();
  auto drag = defaultBlueRov2Drag();
  std::array<double, 6> thrust{-10, -10, 10, 10, 0, 0};
  auto model = gtsam::noiseModel::Isotropic::Sigma(3, 0.1);
  CurrentFactor factor(1, 2, 3, thrust, geom, drag, model);

  Pose3 X = Pose3::Identity();
  Vector3 v_pred = predictBodyVelocity(thrust, geom, drag);
  Vector3 V = v_pred;  // identity rotation, zero current: V should equal v_pred
  Vector3 C = Vector3::Zero();

  gtsam::Vector residual = factor.evaluateError(X, V, C);
  EXPECT_NEAR(residual.norm(), 0.0, 1e-9);
}

TEST(CurrentFactor, NonZeroCurrentShowsUpAsResidualWithoutIt) {
  auto geom = defaultBlueRov2Geometry();
  auto drag = defaultBlueRov2Drag();
  std::array<double, 6> thrust{-10, -10, 10, 10, 0, 0};
  auto model = gtsam::noiseModel::Isotropic::Sigma(3, 0.1);
  CurrentFactor factor(1, 2, 3, thrust, geom, drag, model);

  Pose3 X = Pose3::Identity();
  Vector3 v_pred = predictBodyVelocity(thrust, geom, drag);
  Vector3 true_current(0.2, 0.0, 0.0);
  Vector3 V = v_pred + true_current;  // over-ground velocity includes current
  Vector3 C = Vector3::Zero();  // current estimate not yet caught up

  gtsam::Vector residual = factor.evaluateError(X, V, C);
  EXPECT_NEAR(residual.x(), 0.2, 1e-9);

  // Once C matches the true current, residual returns to zero.
  gtsam::Vector residual2 = factor.evaluateError(X, V, true_current);
  EXPECT_NEAR(residual2.norm(), 0.0, 1e-9);
}

TEST(CurrentFactor, JacobiansMatchNumericalDerivative) {
  auto geom = defaultBlueRov2Geometry();
  auto drag = defaultBlueRov2Drag();
  std::array<double, 6> thrust{-3, 7, 2, -5, 4, -1};
  auto model = gtsam::noiseModel::Isotropic::Sigma(3, 0.1);
  CurrentFactor factor(1, 2, 3, thrust, geom, drag, model);

  Pose3 X(Rot3::RzRyRx(0.1, -0.2, 0.3), Point3(1.0, -2.0, 0.5));
  Vector3 V(0.4, -0.1, 0.05);
  Vector3 C(0.1, 0.1, -0.02);

  gtsam::Matrix H1, H2, H3;
  factor.evaluateError(X, V, C, H1, H2, H3);

  auto f = [&](const Pose3 & Xi, const Vector3 & Vi, const Vector3 & Ci) {
      return evalErrorWrapper(factor, Xi, Vi, Ci);
    };
  gtsam::Matrix numH1 = gtsam::numericalDerivative31<gtsam::Vector, Pose3, Vector3, Vector3>(
    f, X, V, C, 1e-6);
  gtsam::Matrix numH2 = gtsam::numericalDerivative32<gtsam::Vector, Pose3, Vector3, Vector3>(
    f, X, V, C, 1e-6);
  gtsam::Matrix numH3 = gtsam::numericalDerivative33<gtsam::Vector, Pose3, Vector3, Vector3>(
    f, X, V, C, 1e-6);

  EXPECT_TRUE(gtsam::assert_equal(numH1, H1, 1e-4));
  EXPECT_TRUE(gtsam::assert_equal(numH2, H2, 1e-6));
  EXPECT_TRUE(gtsam::assert_equal(numH3, H3, 1e-6));
}
