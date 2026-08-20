#ifndef CAVEX_SIC_SLAM__CURRENT_FACTOR_HPP_
#define CAVEX_SIC_SLAM__CURRENT_FACTOR_HPP_

#include <array>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/nonlinear/NonlinearFactor.h>
#include <gtsam/base/Matrix.h>
#include "cavex_sic_slam/dynamics_model.hpp"

namespace cavex_sic_slam
{

// residual = V - [ R(X) * v_body_predicted(thrust, drag) + C ]
//
// v_body_predicted is a CONSTANT for a given factor instance (it depends
// only on the fixed thrust measurement captured at construction, not on
// any optimization variable), precomputed once here.
//
// This is the observability mechanism for water current: the dynamics
// model predicts velocity THROUGH the water; V(i) is velocity OVER GROUND
// (constrained by sonar + IMU); the difference is current.
class CurrentFactor : public gtsam::NoiseModelFactor3<gtsam::Pose3, gtsam::Vector3, gtsam::Vector3>
{
public:
  CurrentFactor(
    gtsam::Key poseKey, gtsam::Key velKey, gtsam::Key currentKey,
    const std::array<double, 6> & thrust_n,
    const ThrusterGeometry & geom,
    const DragCoefficients & drag,
    const gtsam::SharedNoiseModel & model)
  : gtsam::NoiseModelFactor3<gtsam::Pose3, gtsam::Vector3, gtsam::Vector3>(
      model, poseKey, velKey, currentKey),
    v_pred_(predictBodyVelocity(thrust_n, geom, drag))
  {
  }

  gtsam::Vector evaluateError(
    const gtsam::Pose3 & X, const gtsam::Vector3 & V, const gtsam::Vector3 & C,
    boost::optional<gtsam::Matrix &> H1 = boost::none,
    boost::optional<gtsam::Matrix &> H2 = boost::none,
    boost::optional<gtsam::Matrix &> H3 = boost::none) const override
  {
    const gtsam::Rot3 & R = X.rotation();
    gtsam::Vector3 predicted_ground_v = R.rotate(v_pred_);
    gtsam::Vector3 residual = V - predicted_ground_v - C;

    if (H1) {
      // d(residual)/d(pose tangent (w,t)) = [ R*skew(v_pred) | 0_3x3 ]
      // (right-perturbation convention: X' = X * Expmap(w,t))
      gtsam::Matrix3 dR = R.matrix() * gtsam::skewSymmetric(v_pred_);
      gtsam::Matrix H1full(3, 6);
      H1full.block<3, 3>(0, 0) = dR;
      H1full.block<3, 3>(0, 3) = gtsam::Matrix3::Zero();
      *H1 = H1full;
    }
    if (H2) {
      *H2 = gtsam::Matrix3::Identity();
    }
    if (H3) {
      *H3 = -gtsam::Matrix3::Identity();
    }
    return residual;
  }

private:
  gtsam::Vector3 v_pred_;
};

}  // namespace cavex_sic_slam

#endif  // CAVEX_SIC_SLAM__CURRENT_FACTOR_HPP_
