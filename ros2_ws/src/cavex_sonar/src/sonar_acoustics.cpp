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

}  // namespace cavex_sonar
