#include "cavex_sonar/sonar_acoustics.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <random>

namespace cavex_sonar
{

double transmissionLossDb(double range_m, const AcousticParams & p)
{
  // Floor the range before taking a logarithm: a zero-range return would
  // otherwise yield -inf and propagate NaN through every downstream term.
  const double r = std::max(range_m, p.min_range_m);
  return 20.0 * std::log10(r) + p.absorption_db_per_m * r;
}

double backscatterDb(double incidence_rad, const AcousticParams & p)
{
  // Lambertian-style falloff. Floor cos before the logarithm so a grazing
  // beam yields a very weak -- but finite -- return rather than -inf.
  const double c = std::max(std::cos(incidence_rad), p.min_cos_incidence);
  return p.backscatter_normal_db + 10.0 * p.backscatter_exponent * std::log10(c);
}

double echoLevelDb(double range_m, double incidence_rad, const AcousticParams & p)
{
  // Two-way transmission loss: the pulse travels out and the echo comes back.
  return p.source_level_db - 2.0 * transmissionLossDb(range_m, p) +
         backscatterDb(incidence_rad, p);
}

namespace
{
// Shared by both applySpeckleAndThreshold's dropout path and
// applyClutterToEmptyBeam -- factored out (real fix, 2026-08-22) so a beam
// with ZERO valid rays (true open-water non-detection, previously
// bypassed clutter entirely -- see BeamFormer::formBeams) gets the exact
// same clutter chance as a beam whose real echo fell below threshold.
// `out` must already represent a non-detection; mutates it in place only
// if clutter fires.
void tryInjectClutter(
  BeamReturn & out, const AcousticParams & p, std::mt19937 & gen, double beam_angle_rad)
{
  if (out.detected || p.clutter_probability <= 0.0) {
    return;
  }
  std::uniform_real_distribution<double> clutter_roll(0.0, 1.0);
  if (clutter_roll(gen) >= p.clutter_probability) {
    return;
  }
  std::uniform_real_distribution<double> clutter_range(
    p.min_range_m, std::max(p.min_range_m, p.clutter_max_range_m));
  double range = clutter_range(gen);
  // Current-correlated drift (real request, 2026-08-22): a beam looking
  // upstream (into where the current comes from -- bearing
  // current_direction_rad+pi) sees the particle field swept toward it
  // over time -- closer clutter. A beam looking downstream (bearing
  // current_direction_rad) sees it swept away -- farther clutter.
  // cos(beam - current_direction) is +1 downstream, -1 upstream, so
  // ADDING drift*cos pushes downstream clutter farther and pulls
  // upstream clutter closer. Checked numerically before writing this: do
  // not flip the sign without re-verifying both cases.
  range += p.current_drift_range_m * std::cos(beam_angle_rad - p.current_direction_rad);
  range = std::clamp(range, p.min_range_m, std::max(p.min_range_m, p.clutter_max_range_m));
  out.detected = true;
  out.range_m = range;
  // Borderline by construction -- clutter is a noise-triggered false
  // detection, not a strong real echo.
  out.intensity = p.detection_threshold_db;
}
}  // namespace

BeamReturn applySpeckleAndThreshold(
  double range_m, double incidence_rad, const AcousticParams & p,
  uint32_t seed, uint32_t beam_index, uint32_t ping_index, double beam_angle_rad)
{
  // Seed per (seed, beam, ping) so output does not depend on call order or
  // threading, yet still varies from one ping to the next. A single shared
  // generator would make the A/B evaluation irreproducible for reasons that
  // are very hard to track down later; omitting ping_index would freeze
  // speckle into a static per-beam bias instead of averaging out over time.
  std::seed_seq seq{seed, beam_index, ping_index};
  std::mt19937 gen(seq);

  // Rayleigh-distributed amplitude is the standard first-order model for
  // coherent acoustic speckle. Drawn via its inverse CDF from a uniform.
  std::uniform_real_distribution<double> uni(1e-12, 1.0);
  const double rayleigh = std::sqrt(-2.0 * std::log(uni(gen)));

  const double level = echoLevelDb(range_m, incidence_rad, p) +
    20.0 * std::log10(std::max(rayleigh, 1e-12));

  BeamReturn out;
  out.intensity = level;
  out.detected = level >= p.detection_threshold_db;
  // Deliberately leave range_m at 0.0 when undetected. Callers must gate on
  // `detected`; a dropout carries no range information whatsoever.
  out.range_m = out.detected ? range_m : 0.0;

  // Spurious clutter (real gap, 2026-08-22): only rolled for beams that
  // did NOT get a real detection, using the SAME (seed,beam,ping) stream
  // so results stay reproducible. A real echo always wins over injected
  // clutter -- this only fills in returns that would otherwise be silent.
  tryInjectClutter(out, p, gen, beam_angle_rad);
  return out;
}

BeamReturn applyClutterToEmptyBeam(
  const AcousticParams & p, uint32_t seed, uint32_t beam_index, uint32_t ping_index,
  double beam_angle_rad)
{
  // Same seeding scheme as applySpeckleAndThreshold, so a beam that DOES
  // have valid rays vs one that doesn't still draws from an independent,
  // reproducible stream per (seed, beam, ping) -- no cross-talk between
  // the two call sites for the same (beam, ping).
  std::seed_seq seq{seed, beam_index, ping_index};
  std::mt19937 gen(seq);
  BeamReturn out;
  out.intensity = -std::numeric_limits<double>::infinity();
  out.detected = false;
  out.range_m = 0.0;
  tryInjectClutter(out, p, gen, beam_angle_rad);
  return out;
}

}  // namespace cavex_sonar
