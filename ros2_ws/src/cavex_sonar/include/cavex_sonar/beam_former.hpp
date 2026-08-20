#ifndef CAVEX_SONAR__BEAM_FORMER_HPP_
#define CAVEX_SONAR__BEAM_FORMER_HPP_

#include <cstddef>
#include <cstdint>
#include <vector>

#include "cavex_sonar/sonar_acoustics.hpp"

namespace cavex_sonar
{

/// Geometry of the mapping from dense lidar rays onto sonar beams.
struct BeamFormerConfig
{
  /// Dense rays integrated into each sonar beam (the main-lobe width).
  std::size_t rays_per_beam = 8;
  /// Number of sonar beams produced across the fan.
  std::size_t beam_count = 64;
  /// Angular spacing between adjacent dense rays, radians.
  double angular_step_rad = 0.002;
};

/// Estimate the incidence angle at ray `index` from the local range gradient
/// of neighbouring rays. Returns a value in [0, pi/2].
///
/// A flat surface viewed head-on has near-zero range gradient and therefore
/// near-zero incidence; a steeply oblique surface has a large gradient.
double incidenceAngleAt(
  const std::vector<double> & ranges, std::size_t index, double angular_step_rad);

/// Integrate dense rays into sonar beams and sound each one.
///
/// Non-finite ranges (a gpu_lidar ray that hit nothing) are excluded rather
/// than averaged in. A beam whose rays are all out of range reports no
/// detection.
std::vector<BeamReturn> formBeams(
  const std::vector<double> & ranges, const BeamFormerConfig & cfg,
  const AcousticParams & p, uint32_t seed);

}  // namespace cavex_sonar

#endif  // CAVEX_SONAR__BEAM_FORMER_HPP_
