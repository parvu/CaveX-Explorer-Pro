#ifndef CAVEX_SIC_SLAM__DYNAMICS_MODEL_HPP_
#define CAVEX_SIC_SLAM__DYNAMICS_MODEL_HPP_

#include <array>
#include <Eigen/Dense>

namespace cavex_sic_slam
{

// Position and thrust-direction unit vector (body frame) for each of the
// BlueROV2's six thrusters. See the plan's Global Constraints table --
// derived from the vehicle's real SDF joint poses and axes, not guessed.
struct ThrusterGeometry
{
  std::array<Eigen::Vector3d, 6> position;
  std::array<Eigen::Vector3d, 6> direction;
};

ThrusterGeometry defaultBlueRov2Geometry();

// Diagonal quadratic-drag coefficients (N per (m/s)^2), always negative
// (drag opposes motion). Matches the BlueROV2 SDF exactly: no added mass,
// no linear damping, quadratic drag only.
struct DragCoefficients
{
  double x_uu;
  double y_vv;
  double z_ww;
};

DragCoefficients defaultBlueRov2Drag();

// Sum of thrust_n[i] * direction[i] over all six thrusters -- the net
// body-frame force a set of commanded per-thruster forces (Newtons)
// produces. No torque terms: CurrentFactor only needs linear velocity.
Eigen::Vector3d bodyForce(
  const std::array<double, 6> & thrust_n,
  const ThrusterGeometry & geom);

// Per-axis quasi-steady solve of force + drag*v*|v| == 0, i.e. the
// through-water velocity at which quadratic drag exactly balances the
// applied force: v = sign(F) * sqrt(|F| / |coef|).
Eigen::Vector3d quasiSteadyVelocity(
  const Eigen::Vector3d & force,
  const DragCoefficients & drag);

// Convenience composition: bodyForce() then quasiSteadyVelocity().
Eigen::Vector3d predictBodyVelocity(
  const std::array<double, 6> & thrust_n,
  const ThrusterGeometry & geom,
  const DragCoefficients & drag);

}  // namespace cavex_sic_slam

#endif  // CAVEX_SIC_SLAM__DYNAMICS_MODEL_HPP_
