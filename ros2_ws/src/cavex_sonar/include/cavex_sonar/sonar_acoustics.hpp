#ifndef CAVEX_SONAR__SONAR_ACOUSTICS_HPP_
#define CAVEX_SONAR__SONAR_ACOUSTICS_HPP_

namespace cavex_sonar
{

/// Tunable parameters of the simulated acoustic channel.
///
/// Defaults correspond loosely to a ~1 MHz short-range imaging sonar in fresh
/// water. They are physically motivated but NOT calibrated against hardware --
/// see the honesty note in the README.
struct AcousticParams
{
  /// Absorption coefficient, dB per metre of one-way travel.
  double absorption_db_per_m = 0.4;
  /// Ranges below this (metres) are floored, so log10 cannot return -inf.
  double min_range_m = 0.05;
};

/// One-way transmission loss in dB: spherical spreading plus absorption.
double transmissionLossDb(double range_m, const AcousticParams & p);

}  // namespace cavex_sonar

#endif  // CAVEX_SONAR__SONAR_ACOUSTICS_HPP_
