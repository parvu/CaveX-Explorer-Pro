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
  /// Source level in dB, the transmitted acoustic power.
  double source_level_db = 200.0;
  /// Backscatter strength at normal incidence, dB.
  double backscatter_normal_db = -10.0;
  /// Lambertian falloff exponent. Higher == more specular, weaker at grazing.
  double backscatter_exponent = 1.5;
  /// Floor on cos(incidence) before the logarithm, so grazing beams stay finite.
  double min_cos_incidence = 1e-3;
};

/// One-way transmission loss in dB: spherical spreading plus absorption.
double transmissionLossDb(double range_m, const AcousticParams & p);

/// Backscatter strength in dB for a given incidence angle in radians, where
/// 0 means the beam is perpendicular to the surface.
double backscatterDb(double incidence_rad, const AcousticParams & p);

/// Full one-way-out-and-back echo level in dB at the receiver.
double echoLevelDb(double range_m, double incidence_rad, const AcousticParams & p);

}  // namespace cavex_sonar

#endif  // CAVEX_SONAR__SONAR_ACOUSTICS_HPP_
