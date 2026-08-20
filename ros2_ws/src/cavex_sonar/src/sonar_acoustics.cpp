#include "cavex_sonar/sonar_acoustics.hpp"

#include <algorithm>
#include <cmath>
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

BeamReturn applySpeckleAndThreshold(
  double range_m, double incidence_rad, const AcousticParams & p,
  uint32_t seed, uint32_t beam_index)
{
  // Seed per (seed, beam) so output does not depend on call order or
  // threading. A single shared generator would make the A/B evaluation
  // irreproducible for reasons that are very hard to track down later.
  std::seed_seq seq{seed, beam_index};
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
  return out;
}

}  // namespace cavex_sonar
