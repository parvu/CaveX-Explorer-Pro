#ifndef CAVEX_GTSAM_SLAM__VELOCITY_COUPLING_FACTOR_HPP_
#define CAVEX_GTSAM_SLAM__VELOCITY_COUPLING_FACTOR_HPP_

#include <gtsam/basis/BasisFactors.h>
#include <gtsam/basis/Chebyshev2.h>
#include <gtsam/nonlinear/NonlinearFactor.h>

namespace cavex_gtsam_slam
{

// Couples a continuous-time position trajectory (Chebyshev2 basis,
// ParameterMatrix<3>) to a continuous-time current field (same basis
// type) through the real physical relationship this project's discrete
// CurrentFactor also uses: measured_velocity = d(position)/dt -
// current(t). Both underlying GTSAM functors provide exact analytic
// Jacobians w.r.t. their own ParameterMatrix, passed straight through --
// verified against gtsam::numericalDerivative (see
// test_continuous_trajectory.cpp), not hand-derived and unchecked.
//
// NOTE: current(t) is a function of TIME ONLY, not a spatial field. See
// the plan's "Out of scope" section for what a real spatially-correlated
// field would need instead.
class VelocityCouplingFactor
: public gtsam::NoiseModelFactorN<gtsam::ParameterMatrix<3>, gtsam::ParameterMatrix<3>>
{
  using Base = gtsam::NoiseModelFactorN<gtsam::ParameterMatrix<3>, gtsam::ParameterMatrix<3>>;
  gtsam::Chebyshev2::VectorDerivativeFunctor<3> posDeriv_;
  gtsam::Chebyshev2::VectorEvaluationFunctor<3> currentEval_;
  gtsam::Vector3 measuredVelocity_;

public:
  VelocityCouplingFactor(
    gtsam::Key posKey, gtsam::Key currentKey, size_t N, double t, double a, double b,
    const gtsam::Vector3 & measuredVelocity, const gtsam::SharedNoiseModel & model)
  : Base(model, posKey, currentKey),
    posDeriv_(N, t, a, b),
    currentEval_(N, t, a, b),
    measuredVelocity_(measuredVelocity)
  {}

  gtsam::Vector evaluateError(
    const gtsam::ParameterMatrix<3> & posParams,
    const gtsam::ParameterMatrix<3> & currentParams,
    boost::optional<gtsam::Matrix &> H1 = boost::none,
    boost::optional<gtsam::Matrix &> H2 = boost::none) const override
  {
    gtsam::Matrix Hpos, Hcur;
    gtsam::Vector3 velPred = posDeriv_(posParams, H1 ? &Hpos : nullptr);
    gtsam::Vector3 curPred = currentEval_(currentParams, H2 ? &Hcur : nullptr);
    if (H1) {*H1 = Hpos;}
    if (H2) {*H2 = -Hcur;}
    return (velPred - curPred) - measuredVelocity_;
  }
};

}  // namespace cavex_gtsam_slam

#endif  // CAVEX_GTSAM_SLAM__VELOCITY_COUPLING_FACTOR_HPP_
