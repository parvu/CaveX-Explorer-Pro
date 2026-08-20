#include <gtest/gtest.h>
#include <cmath>
#include "cavex_sic_slam/dynamics_model.hpp"

using cavex_sic_slam::defaultBlueRov2Geometry;
using cavex_sic_slam::defaultBlueRov2Drag;
using cavex_sic_slam::bodyForce;
using cavex_sic_slam::quasiSteadyVelocity;
using cavex_sic_slam::predictBodyVelocity;

TEST(DynamicsModel, ZeroThrustGivesZeroForceAndZeroVelocity) {
  auto geom = defaultBlueRov2Geometry();
  auto drag = defaultBlueRov2Drag();
  std::array<double, 6> thrust{0, 0, 0, 0, 0, 0};
  Eigen::Vector3d f = bodyForce(thrust, geom);
  EXPECT_NEAR(f.norm(), 0.0, 1e-9);
  Eigen::Vector3d v = predictBodyVelocity(thrust, geom, drag);
  EXPECT_NEAR(v.norm(), 0.0, 1e-9);
}

TEST(DynamicsModel, AllFourHorizontalThrustersForwardGivesPureSurge) {
  // T1..T4 each pushing with a +0.7071/-0.7071 X/Y component; commanding
  // T1=T4=+F (their -X-ish/+X-ish components) tuned so the Y components
  // cancel: T1 dir=(-.7071,-.7071), T2 dir=(-.7071,.7071),
  // T3 dir=(.7071,-.7071), T4 dir=(.7071,.7071). Commanding T3=T4=+F and
  // T1=T2=-F gives net force (4*.7071*F, 0, 0) -- pure +X surge, no sway.
  auto geom = defaultBlueRov2Geometry();
  auto drag = defaultBlueRov2Drag();
  std::array<double, 6> thrust{-10, -10, 10, 10, 0, 0};
  Eigen::Vector3d f = bodyForce(thrust, geom);
  EXPECT_NEAR(f.y(), 0.0, 1e-6);
  EXPECT_NEAR(f.z(), 0.0, 1e-6);
  EXPECT_GT(f.x(), 0.0);
  double expected_fx = 4.0 * 0.70710678 * 10.0;
  EXPECT_NEAR(f.x(), expected_fx, 1e-3);
}

TEST(DynamicsModel, VerticalThrustersGiveHeaveOnly) {
  auto geom = defaultBlueRov2Geometry();
  auto drag = defaultBlueRov2Drag();
  std::array<double, 6> thrust{0, 0, 0, 0, 5, 5};
  Eigen::Vector3d f = bodyForce(thrust, geom);
  EXPECT_NEAR(f.x(), 0.0, 1e-9);
  EXPECT_NEAR(f.y(), 0.0, 1e-9);
  EXPECT_NEAR(f.z(), -10.0, 1e-9);
}

TEST(DynamicsModel, QuasiSteadyVelocitySolvesQuadraticDragPerAxis) {
  cavex_sic_slam::DragCoefficients drag{-33.732, -54.16, -73.225};
  // v*|v| = F/|coef| => v = sign(F)*sqrt(|F|/|coef|)
  Eigen::Vector3d force(33.732, 0.0, 0.0);  // chosen so |F|/|coef| == 1
  Eigen::Vector3d v = quasiSteadyVelocity(force, drag);
  EXPECT_NEAR(v.x(), 1.0, 1e-6);
  EXPECT_NEAR(v.y(), 0.0, 1e-9);
  EXPECT_NEAR(v.z(), 0.0, 1e-9);
}

TEST(DynamicsModel, QuasiSteadyVelocityPreservesForceSign) {
  cavex_sic_slam::DragCoefficients drag{-33.732, -54.16, -73.225};
  Eigen::Vector3d force(-33.732, 0.0, 0.0);
  Eigen::Vector3d v = quasiSteadyVelocity(force, drag);
  EXPECT_LT(v.x(), 0.0);
  EXPECT_NEAR(v.x(), -1.0, 1e-6);
}

TEST(DynamicsModel, PredictBodyVelocityComposesForceAndDrag) {
  auto geom = defaultBlueRov2Geometry();
  auto drag = defaultBlueRov2Drag();
  std::array<double, 6> thrust{0, 0, 0, 0, 20, 20};
  // Fz = -40 N -> |Fz|/73.225 = 0.5464.. -> vz = -sqrt(0.5464..)
  Eigen::Vector3d v = predictBodyVelocity(thrust, geom, drag);
  EXPECT_NEAR(v.x(), 0.0, 1e-9);
  EXPECT_NEAR(v.y(), 0.0, 1e-9);
  double expected_vz = -std::sqrt(40.0 / 73.225);
  EXPECT_NEAR(v.z(), expected_vz, 1e-6);
}
