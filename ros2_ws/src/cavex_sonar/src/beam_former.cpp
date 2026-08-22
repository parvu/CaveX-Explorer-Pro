#include "cavex_sonar/beam_former.hpp"

#include <algorithm>
#include <cmath>

namespace cavex_sonar
{

double incidenceAngleAt(
  const std::vector<double> & ranges, std::size_t index, double angular_step_rad)
{
  if (ranges.size() < 2 || angular_step_rad <= 0.0) {
    return 0.0;
  }

  // Central difference where possible, one-sided at the ends.
  const std::size_t lo = (index == 0) ? 0 : index - 1;
  const std::size_t hi = std::min(index + 1, ranges.size() - 1);
  if (lo == hi) {
    return 0.0;
  }

  const double r = ranges[index];
  if (!std::isfinite(r) || !std::isfinite(ranges[lo]) || !std::isfinite(ranges[hi])) {
    return 0.0;
  }

  const double dr_dtheta =
    (ranges[hi] - ranges[lo]) / (static_cast<double>(hi - lo) * angular_step_rad);

  // For a surface in polar form r(theta), the angle between the line of sight
  // and the surface normal satisfies tan(incidence) = (dr/dtheta) / r.
  if (r <= 0.0) {
    return 0.0;
  }
  const double angle = std::atan(std::abs(dr_dtheta) / r);
  return std::clamp(angle, 0.0, M_PI / 2.0);
}

std::vector<BeamReturn> formBeams(
  const std::vector<double> & ranges, const BeamFormerConfig & cfg,
  const AcousticParams & p, uint32_t seed, uint32_t ping_index)
{
  std::vector<BeamReturn> beams;
  beams.reserve(cfg.beam_count);
  if (cfg.rays_per_beam == 0) {
    return beams;
  }

  const auto geom = beamScanGeometry(cfg.angle_min_rad, cfg.angular_step_rad, cfg.rays_per_beam);

  for (std::size_t b = 0; b < cfg.beam_count; ++b) {
    const std::size_t begin = b * cfg.rays_per_beam;

    // Integrate the main lobe: mean range and mean incidence over the rays
    // that actually returned. A single infinite ray must not poison the beam.
    double range_sum = 0.0, incidence_sum = 0.0;
    std::size_t valid = 0;
    for (std::size_t k = 0; k < cfg.rays_per_beam; ++k) {
      const std::size_t i = begin + k;
      if (i >= ranges.size()) {
        break;
      }
      if (!std::isfinite(ranges[i])) {
        continue;
      }
      range_sum += ranges[i];
      incidence_sum += incidenceAngleAt(ranges, i, cfg.angular_step_rad);
      ++valid;
    }

    if (valid == 0) {
      // Every ray in this beam hit nothing: an honest non-detection --
      // UNLESS clutter fires (real fix, 2026-08-22: this is the most
      // realistic case for turbidity clutter, "nothing real was there").
      // applyClutterToEmptyBeam is a no-op (returns the same -inf/false
      // non-detection) when clutter_probability is 0.0, the default.
      const double beam_angle = geom.angle_min + static_cast<double>(b) * geom.angle_increment;
      beams.push_back(
        applyClutterToEmptyBeam(p, seed, static_cast<uint32_t>(b), ping_index, beam_angle));
      continue;
    }

    const double mean_range = range_sum / static_cast<double>(valid);
    const double mean_incidence = incidence_sum / static_cast<double>(valid);
    const double beam_angle = geom.angle_min + static_cast<double>(b) * geom.angle_increment;
    beams.push_back(applySpeckleAndThreshold(
      mean_range, mean_incidence, p, seed, static_cast<uint32_t>(b), ping_index, beam_angle));
  }

  return beams;
}

BeamScanGeometry beamScanGeometry(
  double in_angle_min, double in_angle_increment, std::size_t rays_per_beam)
{
  const double half_span = static_cast<double>(rays_per_beam - 1) / 2.0;
  return BeamScanGeometry{
    in_angle_min + in_angle_increment * half_span,
    in_angle_increment * static_cast<double>(rays_per_beam)};
}

}  // namespace cavex_sonar
