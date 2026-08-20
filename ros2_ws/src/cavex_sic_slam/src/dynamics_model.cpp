/*
 * Copyright 2026 CaveX Explorer Pro
 * Licensed under MIT
 */

#include "cavex_sic_slam/dynamics_model.hpp"
#include <cmath>

namespace cavex_sic_slam
{

ThrusterGeometry defaultBlueRov2Geometry()
{
  ThrusterGeometry g;
  constexpr double kD = 0.70710678118654752;
  g.position = {
    Eigen::Vector3d(0.14, -0.092, 0.0),
    Eigen::Vector3d(0.14, 0.092, 0.0),
    Eigen::Vector3d(-0.14, -0.092, 0.0),
    Eigen::Vector3d(-0.14, 0.092, 0.0),
    Eigen::Vector3d(0.0, -0.109, 0.077),
    Eigen::Vector3d(0.0, 0.109, 0.077),
  };
  g.direction = {
    Eigen::Vector3d(-kD, -kD, 0.0),
    Eigen::Vector3d(-kD, kD, 0.0),
    Eigen::Vector3d(kD, -kD, 0.0),
    Eigen::Vector3d(kD, kD, 0.0),
    Eigen::Vector3d(0.0, 0.0, -1.0),
    Eigen::Vector3d(0.0, 0.0, -1.0),
  };
  return g;
}

DragCoefficients defaultBlueRov2Drag()
{
  return DragCoefficients{-33.732, -54.16, -73.225};
}

Eigen::Vector3d bodyForce(
  const std::array<double, 6> & thrust_n,
  const ThrusterGeometry & geom)
{
  Eigen::Vector3d f = Eigen::Vector3d::Zero();
  for (std::size_t i = 0; i < 6; ++i) {
    f += thrust_n[i] * geom.direction[i];
  }
  return f;
}

namespace
{
double solveAxis(double force, double coef)
{
  // coef is negative; |coef| is the drag magnitude.
  double mag = std::abs(coef);
  if (mag < 1e-12) {
    return 0.0;
  }
  double v_sq = std::abs(force) / mag;
  double v = std::sqrt(v_sq);
  return force >= 0.0 ? v : -v;
}
}  // namespace

Eigen::Vector3d quasiSteadyVelocity(
  const Eigen::Vector3d & force,
  const DragCoefficients & drag)
{
  return Eigen::Vector3d(
    solveAxis(force.x(), drag.x_uu),
    solveAxis(force.y(), drag.y_vv),
    solveAxis(force.z(), drag.z_ww));
}

Eigen::Vector3d predictBodyVelocity(
  const std::array<double, 6> & thrust_n,
  const ThrusterGeometry & geom,
  const DragCoefficients & drag)
{
  return quasiSteadyVelocity(bodyForce(thrust_n, geom), drag);
}

}  // namespace cavex_sic_slam
