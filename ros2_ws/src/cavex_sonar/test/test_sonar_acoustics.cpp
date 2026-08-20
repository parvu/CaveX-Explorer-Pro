#include <gtest/gtest.h>
#include <cmath>
#include "cavex_sonar/sonar_acoustics.hpp"

using cavex_sonar::AcousticParams;
using cavex_sonar::transmissionLossDb;

TEST(TransmissionLoss, GrowsWithRange) {
  AcousticParams p;
  EXPECT_LT(transmissionLossDb(1.0, p), transmissionLossDb(10.0, p));
  EXPECT_LT(transmissionLossDb(10.0, p), transmissionLossDb(30.0, p));
}

TEST(TransmissionLoss, SphericalSpreadingDominatesAtShortRange) {
  // With absorption zeroed, loss must be exactly 20*log10(r).
  AcousticParams p;
  p.absorption_db_per_m = 0.0;
  EXPECT_NEAR(transmissionLossDb(10.0, p), 20.0, 1e-9);
  EXPECT_NEAR(transmissionLossDb(100.0, p), 40.0, 1e-9);
}

TEST(TransmissionLoss, AbsorptionAddsLinearTerm) {
  AcousticParams p;
  p.absorption_db_per_m = 0.5;
  const double expected = 20.0 * std::log10(10.0) + 0.5 * 10.0;
  EXPECT_NEAR(transmissionLossDb(10.0, p), expected, 1e-9);
}

TEST(TransmissionLoss, ZeroRangeIsFiniteNotNegativeInfinity) {
  // log10(0) is -inf; the implementation must floor the range so a
  // zero-range return cannot poison downstream arithmetic with NaN.
  AcousticParams p;
  EXPECT_TRUE(std::isfinite(transmissionLossDb(0.0, p)));
}
