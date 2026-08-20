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
/// detection, with `intensity` set to negative infinity (zero linear echo
/// amplitude) rather than 0.0, so a total non-detection cannot be mistaken
/// for a moderate dB return. `ping_index` must be a monotonically
/// increasing per-scan counter; it is threaded into the speckle seed so
/// speckle varies from ping to ping instead of freezing into a static
/// per-beam bias, while remaining fully deterministic for a fixed
/// (seed, beam_index, ping_index).
std::vector<BeamReturn> formBeams(
  const std::vector<double> & ranges, const BeamFormerConfig & cfg,
  const AcousticParams & p, uint32_t seed, uint32_t ping_index);

/// Angular description of the output beam scan, in the same units/frame as
/// the input dense-ray scan.
struct BeamScanGeometry
{
  double angle_min;
  double angle_increment;
};

/// Compute the bearing of beam 0's centre and the spacing between beam
/// centres, given the input dense-ray scan's angle_min/angle_increment.
///
/// Beam `b` integrates input rays `[b*rays_per_beam, b*rays_per_beam +
/// rays_per_beam - 1]`, so its true centre sits at input ray index
/// `b*rays_per_beam + (rays_per_beam-1)/2`, not at `b*rays_per_beam`.
BeamScanGeometry beamScanGeometry(
  double in_angle_min, double in_angle_increment, std::size_t rays_per_beam);

}  // namespace cavex_sonar

#endif  // CAVEX_SONAR__BEAM_FORMER_HPP_
