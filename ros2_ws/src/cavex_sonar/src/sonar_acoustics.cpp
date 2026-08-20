#include "cavex_sonar/sonar_acoustics.hpp"

#include <algorithm>
#include <cmath>

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

}  // namespace cavex_sonar
