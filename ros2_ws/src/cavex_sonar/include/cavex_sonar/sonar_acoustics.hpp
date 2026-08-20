#ifndef CAVEX_SONAR__SONAR_ACOUSTICS_HPP_
#define CAVEX_SONAR__SONAR_ACOUSTICS_HPP_

#include <cstdint>

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
  /// Echo level in dB below which a beam reports no detection at all.
  double detection_threshold_db = 100.0;
};

/// One-way transmission loss in dB: spherical spreading plus absorption.
double transmissionLossDb(double range_m, const AcousticParams & p);

/// Backscatter strength in dB for a given incidence angle in radians, where
/// 0 means the beam is perpendicular to the surface.
double backscatterDb(double incidence_rad, const AcousticParams & p);

/// Two-way (out-and-back) echo level in dB at the receiver.
double echoLevelDb(double range_m, double incidence_rad, const AcousticParams & p);

/// The outcome of sounding one beam.
struct BeamReturn
{
  /// False means no detection. A non-detection reports NO range at all --
  /// it must never be turned into a confident wrong range downstream.
  bool detected = false;
  /// Valid only when detected is true.
  double range_m = 0.0;
  /// Post-speckle echo level in dB, carried to ROS in LaserScan.intensities.
  /// A total non-detection (every ray in the beam out of range) carries
  /// negative infinity here, not 0.0 -- 0.0 would look like a valid
  /// moderate return on a dB scale. Callers must check `isfinite()` before
  /// doing arithmetic on this field.
  double intensity = 0.0;
};

/// Sound one beam: apply Rayleigh speckle to the echo level and threshold it.
///
/// Randomness is derived deterministically from (seed, beam_index,
/// ping_index) rather than from shared mutable generator state, so results
/// are reproducible regardless of evaluation order or threading -- the same
/// triple always yields bit-identical output, and a run replayed from the
/// same seed reproduces exactly. `ping_index` is what keeps speckle from
/// degenerating into a static per-beam bias: without it, beam `b` would draw
/// the identical Rayleigh sample on every scan forever, which is the worst
/// case for downstream SLAM (a constant bias gets absorbed into the map
/// instead of averaging out). The A/B evaluation depends on this.
BeamReturn applySpeckleAndThreshold(
  double range_m, double incidence_rad, const AcousticParams & p,
  uint32_t seed, uint32_t beam_index, uint32_t ping_index);

}  // namespace cavex_sonar

#endif  // CAVEX_SONAR__SONAR_ACOUSTICS_HPP_
