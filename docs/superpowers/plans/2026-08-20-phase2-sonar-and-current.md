# Phase 2 Sensing Substrate: Acoustic Sonar + Ocean Current — Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Give the BlueROV2 a real acoustic sonar and put it in a real water current, so the SIC-SLAM factor graph (a following plan) has genuine sonar returns and a genuine disturbance to estimate.

**Architecture:** The acoustic physics lives in a host-agnostic C++ library (`sonar_acoustics`) with no Gazebo and no ROS dependencies, so it is unit-testable off-simulator. A dense `gpu_lidar` declared in the BlueROV2 SDF acts purely as the ray engine; a thin ROS 2 node feeds its returns through the library and republishes them as `sensor_msgs/LaserScan` with echo strength in `intensities`. Water current comes from the stock `Hydrodynamics` plugin's existing `/ocean_current` support — no new physics code.

**Tech Stack:** C++17, ROS 2 Jazzy (`ament_cmake`), Gazebo Harmonic 8.14.0, `ros_gz_bridge`, GoogleTest, pytest, GTSAM 4.2 (spike only in this plan).

**Spec:** `docs/superpowers/specs/2026-08-20-phase2-sic-slam-dcs-design.md`

## Global Constraints

- ROS 2 **Jazzy**; Gazebo Sim **8.14.0** (Harmonic). Build with `colcon build --symlink-install`.
- New ROS package name: **`cavex_sonar`** (`ament_cmake`). Do **not** name anything `sic_slam` in this plan — the repo already has two unrelated `sic_slam` nodes, and the real one is claimed by the following plan.
- The `sonar_acoustics` library must have **zero** ROS and **zero** Gazebo includes. This is what makes it testable and relocatable; a PR adding either is wrong.
- Emit **`sensor_msgs/LaserScan`** only. No custom message types, no custom bridge code.
- All randomness takes an explicit **seed parameter**. Fixed seed must produce identical output — required for the A/B evaluation in the following plan.
- Ground-truth topics (`/cavex/current_ground_truth`) are **scorer-only**. No perception or control node may subscribe to them.
- BlueROV2 physical constants, copied verbatim from `ros2_ws/src/cavex_tracked_vehicle/models/bluerov2/model.sdf`: mass `10.0` kg, `fluid_density` `998.0`, quadratic drag `xUabsU=-33.732`, `yVabsV=-54.16`, `zWabsW=-73.225`, `kPabsP=mQabsQ=nRabsR=-3.992`. **Added mass and linear damping are all zero.**
- Source `ros2_ws/ardupilot_gazebo_env.sh` before any `gz sim` run; it sets `GZ_SIM_SYSTEM_PLUGIN_PATH`, `GZ_SIM_RESOURCE_PATH`, and the WSL D3D12 GPU passthrough vars.
- Honesty conventions are enforced in this repo: never describe the sonar as hardware-validated. See Task 9.

---

### Task 1: Spike — GTSAM 4.2 links against ROS 2 Jazzy

Throwaway. Answers one yes/no question that determines the following plan's build instructions. Nothing here is kept.

**Files:**
- Create: `/tmp/gtsam_spike/` (throwaway, never committed)

- [ ] **Step 1: Install GTSAM**

```bash
sudo apt-get update && sudo apt-get install -y libgtsam-dev libgtsam-unstable-dev
```

- [ ] **Step 2: Write a spike program that uses the exact GTSAM features the next plan needs**

Create `/tmp/gtsam_spike/main.cpp`:

```cpp
#include <gtsam/navigation/CombinedImuFactor.h>
#include <gtsam/nonlinear/ISAM2.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/inference/Symbol.h>
#include <iostream>

int main() {
  auto params = gtsam::PreintegratedCombinedMeasurements::Params::MakeSharedU(9.81);
  gtsam::imuBias::ConstantBias zeroBias;
  gtsam::PreintegratedCombinedMeasurements pim(params, zeroBias);
  pim.integrateMeasurement(gtsam::Vector3(0, 0, 9.81), gtsam::Vector3(0, 0, 0), 0.01);

  gtsam::ISAM2 isam;
  gtsam::NonlinearFactorGraph graph;
  gtsam::Values values;
  using gtsam::symbol_shorthand::C;
  graph.addPrior(C(0), gtsam::Vector3(0, 0, 0),
                 gtsam::noiseModel::Isotropic::Sigma(3, 0.1));
  values.insert(C(0), gtsam::Vector3(0, 0, 0));
  isam.update(graph, values);

  std::cout << "GTSAM OK, preintegrated dT=" << pim.deltaTij() << std::endl;
  return 0;
}
```

- [ ] **Step 3: Build it against Jazzy's ABI**

The point is compiling in an environment where ROS 2 Jazzy is sourced, so any Boost/C++ ABI conflict surfaces now rather than mid-implementation.

```bash
source /opt/ros/jazzy/setup.bash
cd /tmp/gtsam_spike
g++ -std=c++17 main.cpp -o spike $(pkg-config --cflags --libs eigen3) -lgtsam -ltbb && ./spike
```

Expected: prints `GTSAM OK, preintegrated dT=0.01`.

- [ ] **Step 4: Record the finding**

If it succeeds, note the GTSAM version (`apt-cache policy libgtsam-dev`) — the next plan depends on apt GTSAM.

If it **fails** to compile or link, do not attempt fixes here. Record the exact error and stop; the next plan switches to a source build of GTSAM pinned to `4.2a9`. Either way this is a finding, not a blocker for the rest of *this* plan — Tasks 2-9 do not use GTSAM.

- [ ] **Step 5: Clean up**

```bash
rm -rf /tmp/gtsam_spike
```

Nothing is committed for this task.

---

### Task 2: Package skeleton and the acoustic transmission-loss model

Creates the `cavex_sonar` package and the first piece of real physics.

**Files:**
- Create: `ros2_ws/src/cavex_sonar/package.xml`
- Create: `ros2_ws/src/cavex_sonar/CMakeLists.txt`
- Create: `ros2_ws/src/cavex_sonar/include/cavex_sonar/sonar_acoustics.hpp`
- Create: `ros2_ws/src/cavex_sonar/src/sonar_acoustics.cpp`
- Test: `ros2_ws/src/cavex_sonar/test/test_sonar_acoustics.cpp`

**Interfaces:**
- Consumes: nothing.
- Produces: `namespace cavex_sonar`, `struct AcousticParams`, and
  `double transmissionLossDb(double range_m, const AcousticParams &p)`.

- [ ] **Step 1: Write the failing test**

Create `ros2_ws/src/cavex_sonar/test/test_sonar_acoustics.cpp`:

```cpp
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
```

- [ ] **Step 2: Create the package files so the test can build**

Create `ros2_ws/src/cavex_sonar/package.xml`:

```xml
<?xml version="1.0"?>
<?xml-model href="http://download.ros.org/schema/package_format3.xsd" schematypens="http://www.w3.org/2001/XMLSchema"?>
<package format="3">
  <name>cavex_sonar</name>
  <version>1.0.0</version>
  <description>Acoustic sonar model for the CaveX BlueROV2: a host-agnostic acoustic physics library plus a ROS 2 node that converts dense gpu_lidar returns into simulated sonar beams.</description>
  <maintainer email="petrisor.parvu@upb.ro">CaveX Team</maintainer>
  <license>Apache-2.0</license>

  <buildtool_depend>ament_cmake</buildtool_depend>
  <buildtool_depend>ament_cmake_python</buildtool_depend>

  <depend>rclcpp</depend>
  <depend>rclpy</depend>
  <depend>sensor_msgs</depend>
  <depend>geometry_msgs</depend>

  <test_depend>ament_lint_auto</test_depend>
  <test_depend>ament_lint_common</test_depend>
  <test_depend>ament_cmake_gtest</test_depend>

  <export>
    <build_type>ament_cmake</build_type>
  </export>
</package>
```

Create `ros2_ws/src/cavex_sonar/CMakeLists.txt`:

```cmake
cmake_minimum_required(VERSION 3.8)
project(cavex_sonar)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

# Host-agnostic acoustic physics. Deliberately has NO ROS and NO Gazebo
# dependencies so it can be unit-tested off-simulator and relocated into a
# gz-sim system plugin later without touching its physics or its tests.
add_library(sonar_acoustics src/sonar_acoustics.cpp)
target_include_directories(sonar_acoustics PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

install(TARGETS sonar_acoustics
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib)
install(DIRECTORY include/ DESTINATION include)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()

  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_sonar_acoustics test/test_sonar_acoustics.cpp)
  target_link_libraries(test_sonar_acoustics sonar_acoustics)
endif()

ament_package()
```

Create the header `ros2_ws/src/cavex_sonar/include/cavex_sonar/sonar_acoustics.hpp`:

```cpp
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
```

Create a stub `ros2_ws/src/cavex_sonar/src/sonar_acoustics.cpp` that compiles but is wrong, so the test genuinely fails:

```cpp
#include "cavex_sonar/sonar_acoustics.hpp"

namespace cavex_sonar
{

double transmissionLossDb(double, const AcousticParams &)
{
  return 0.0;
}

}  // namespace cavex_sonar
```

- [ ] **Step 3: Run the test to verify it fails**

```bash
cd /home/parvu/CaveX-Explorer-Pro/ros2_ws
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install --packages-select cavex_sonar
colcon test --packages-select cavex_sonar --event-handlers console_direct+
```

Expected: `TransmissionLoss.GrowsWithRange` and the other three FAIL (everything returns 0.0).

- [ ] **Step 4: Write the real implementation**

Replace `ros2_ws/src/cavex_sonar/src/sonar_acoustics.cpp`:

```cpp
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
```

- [ ] **Step 5: Run the tests to verify they pass**

```bash
cd /home/parvu/CaveX-Explorer-Pro/ros2_ws
colcon build --symlink-install --packages-select cavex_sonar
colcon test --packages-select cavex_sonar --event-handlers console_direct+
colcon test-result --verbose --test-result-base build/cavex_sonar
```

Expected: 4 tests, all PASS.

- [ ] **Step 6: Commit**

```bash
cd /home/parvu/CaveX-Explorer-Pro
git add ros2_ws/src/cavex_sonar
git commit -m "Add cavex_sonar package with acoustic transmission-loss model

Spherical spreading plus linear absorption, with the range floored before
the logarithm so a zero-range return cannot propagate NaN downstream.

The sonar_acoustics library deliberately has no ROS and no Gazebo includes,
so it is unit-testable off-simulator and can be relocated into a gz-sim
system plugin later without touching its physics or its tests."
```

---

### Task 3: Incidence-angle backscatter and echo level

The term that makes a smooth tunnel wall a weak, ambiguous target — the effect that drives the degeneracy risk the spec flags.

**Files:**
- Modify: `ros2_ws/src/cavex_sonar/include/cavex_sonar/sonar_acoustics.hpp`
- Modify: `ros2_ws/src/cavex_sonar/src/sonar_acoustics.cpp`
- Modify: `ros2_ws/src/cavex_sonar/test/test_sonar_acoustics.cpp`

**Interfaces:**
- Consumes: `AcousticParams`, `transmissionLossDb` from Task 2.
- Produces: `double backscatterDb(double incidence_rad, const AcousticParams &p)` and `double echoLevelDb(double range_m, double incidence_rad, const AcousticParams &p)`.

- [ ] **Step 1: Write the failing tests**

Append to `ros2_ws/src/cavex_sonar/test/test_sonar_acoustics.cpp`:

```cpp
using cavex_sonar::backscatterDb;
using cavex_sonar::echoLevelDb;

TEST(Backscatter, NormalIncidenceIsStrongest) {
  AcousticParams p;
  // 0 rad == beam perpendicular to the surface == strongest return.
  EXPECT_GT(backscatterDb(0.0, p), backscatterDb(0.6, p));
  EXPECT_GT(backscatterDb(0.6, p), backscatterDb(1.2, p));
}

TEST(Backscatter, DecreasesMonotonicallyWithIncidence) {
  AcousticParams p;
  double prev = backscatterDb(0.0, p);
  for (double a = 0.05; a < 1.55; a += 0.05) {
    const double cur = backscatterDb(a, p);
    EXPECT_LE(cur, prev) << "backscatter must not increase at incidence " << a;
    prev = cur;
  }
}

TEST(Backscatter, GrazingIncidenceIsFiniteNotNegativeInfinity) {
  AcousticParams p;
  // cos(pi/2) == 0; log10(0) is -inf. Must be floored, or a grazing beam
  // poisons the echo level with NaN instead of simply being a weak return.
  EXPECT_TRUE(std::isfinite(backscatterDb(M_PI / 2.0, p)));
}

TEST(EchoLevel, FallsOffWithBothRangeAndIncidence) {
  AcousticParams p;
  const double near_normal = echoLevelDb(2.0, 0.0, p);
  const double far_normal = echoLevelDb(20.0, 0.0, p);
  const double near_grazing = echoLevelDb(2.0, 1.4, p);
  EXPECT_GT(near_normal, far_normal) << "range must reduce echo level";
  EXPECT_GT(near_normal, near_grazing) << "incidence must reduce echo level";
}
```

- [ ] **Step 2: Run the tests to verify they fail**

```bash
cd /home/parvu/CaveX-Explorer-Pro/ros2_ws
colcon build --symlink-install --packages-select cavex_sonar
```

Expected: **compile error** — `backscatterDb` and `echoLevelDb` are not declared. That is the correct failure for this step.

- [ ] **Step 3: Declare the new functions**

Add to `AcousticParams` in `sonar_acoustics.hpp`, inside the struct:

```cpp
  /// Source level in dB, the transmitted acoustic power.
  double source_level_db = 200.0;
  /// Backscatter strength at normal incidence, dB.
  double backscatter_normal_db = -10.0;
  /// Lambertian falloff exponent. Higher == more specular, weaker at grazing.
  double backscatter_exponent = 1.5;
  /// Floor on cos(incidence) before the logarithm, so grazing beams stay finite.
  double min_cos_incidence = 1e-3;
```

Add to the `cavex_sonar` namespace in `sonar_acoustics.hpp`, after `transmissionLossDb`:

```cpp
/// Backscatter strength in dB for a given incidence angle in radians, where
/// 0 means the beam is perpendicular to the surface.
double backscatterDb(double incidence_rad, const AcousticParams & p);

/// Full one-way-out-and-back echo level in dB at the receiver.
double echoLevelDb(double range_m, double incidence_rad, const AcousticParams & p);
```

- [ ] **Step 4: Implement them**

Append to `ros2_ws/src/cavex_sonar/src/sonar_acoustics.cpp`, inside the namespace:

```cpp
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
```

- [ ] **Step 5: Run the tests to verify they pass**

```bash
cd /home/parvu/CaveX-Explorer-Pro/ros2_ws
colcon build --symlink-install --packages-select cavex_sonar
colcon test --packages-select cavex_sonar --event-handlers console_direct+
colcon test-result --verbose --test-result-base build/cavex_sonar
```

Expected: 8 tests, all PASS.

- [ ] **Step 6: Commit**

```bash
cd /home/parvu/CaveX-Explorer-Pro
git add ros2_ws/src/cavex_sonar
git commit -m "Add incidence-angle backscatter and two-way echo level

Lambertian-style falloff with the cosine floored before the logarithm, so a
grazing beam produces a weak but finite return instead of NaN.

This is the term that makes a smooth tunnel wall a weak, ambiguous target,
which is the mechanism behind the featureless-corridor degeneracy risk the
spec flags as measure-don't-assume."
```

---

### Task 4: Rayleigh speckle and detection threshold

Turns a deterministic echo level into a realistic, seed-reproducible detection with honest dropouts.

**Files:**
- Modify: `ros2_ws/src/cavex_sonar/include/cavex_sonar/sonar_acoustics.hpp`
- Modify: `ros2_ws/src/cavex_sonar/src/sonar_acoustics.cpp`
- Modify: `ros2_ws/src/cavex_sonar/test/test_sonar_acoustics.cpp`

**Interfaces:**
- Consumes: `AcousticParams`, `echoLevelDb` from Task 3.
- Produces: `struct BeamReturn { bool detected; double range_m; double intensity; };` and `BeamReturn applySpeckleAndThreshold(double range_m, double incidence_rad, const AcousticParams &p, uint32_t seed, uint32_t beam_index)`.

- [ ] **Step 1: Write the failing tests**

Append to `ros2_ws/src/cavex_sonar/test/test_sonar_acoustics.cpp`:

```cpp
#include <cstdint>
using cavex_sonar::BeamReturn;
using cavex_sonar::applySpeckleAndThreshold;

TEST(Speckle, IsReproducibleForAFixedSeed) {
  AcousticParams p;
  const BeamReturn a = applySpeckleAndThreshold(5.0, 0.2, p, 42u, 7u);
  const BeamReturn b = applySpeckleAndThreshold(5.0, 0.2, p, 42u, 7u);
  EXPECT_EQ(a.detected, b.detected);
  EXPECT_DOUBLE_EQ(a.intensity, b.intensity);
  EXPECT_DOUBLE_EQ(a.range_m, b.range_m);
}

TEST(Speckle, DiffersBetweenBeamsWithinTheSameSeed) {
  AcousticParams p;
  // Independent speckle per beam; identical values across beams would mean
  // the beam index is not reaching the generator.
  const BeamReturn a = applySpeckleAndThreshold(5.0, 0.2, p, 42u, 1u);
  const BeamReturn b = applySpeckleAndThreshold(5.0, 0.2, p, 42u, 2u);
  EXPECT_NE(a.intensity, b.intensity);
}

TEST(Threshold, StrongNearNormalReturnIsDetected) {
  AcousticParams p;
  const BeamReturn r = applySpeckleAndThreshold(2.0, 0.0, p, 1u, 0u);
  EXPECT_TRUE(r.detected);
  EXPECT_NEAR(r.range_m, 2.0, 1e-9) << "a detected beam reports its true range";
}

TEST(Threshold, VeryWeakReturnDropsOutRatherThanReportingAWrongRange) {
  AcousticParams p;
  // Force everything below threshold: a dropout must report detected=false,
  // never a confident wrong range. Reporting a bogus range here would inject
  // false constraints straight into the SLAM factor graph.
  p.detection_threshold_db = 1e9;
  const BeamReturn r = applySpeckleAndThreshold(2.0, 0.0, p, 1u, 0u);
  EXPECT_FALSE(r.detected);
}

TEST(Threshold, DropoutRateRisesWithIncidenceAcrossManyBeams) {
  AcousticParams p;
  p.detection_threshold_db = 120.0;
  int near_normal_hits = 0, grazing_hits = 0;
  for (uint32_t i = 0; i < 500; ++i) {
    if (applySpeckleAndThreshold(8.0, 0.05, p, 99u, i).detected) { ++near_normal_hits; }
    if (applySpeckleAndThreshold(8.0, 1.45, p, 99u, i).detected) { ++grazing_hits; }
  }
  EXPECT_GT(near_normal_hits, grazing_hits)
      << "grazing beams must drop out more often than near-normal ones";
}
```

- [ ] **Step 2: Run the tests to verify they fail**

```bash
cd /home/parvu/CaveX-Explorer-Pro/ros2_ws
colcon build --symlink-install --packages-select cavex_sonar
```

Expected: **compile error** — `BeamReturn` and `applySpeckleAndThreshold` are not declared.

- [ ] **Step 3: Declare the new type and function**

Add to `AcousticParams` in `sonar_acoustics.hpp`, inside the struct:

```cpp
  /// Echo level in dB below which a beam reports no detection at all.
  double detection_threshold_db = 100.0;
```

Add `#include <cstdint>` at the top of `sonar_acoustics.hpp`, then add to the namespace:

```cpp
/// The outcome of sounding one beam.
struct BeamReturn
{
  /// False means no detection. A non-detection reports NO range at all --
  /// it must never be turned into a confident wrong range downstream.
  bool detected = false;
  /// Valid only when detected is true.
  double range_m = 0.0;
  /// Post-speckle echo level in dB, carried to ROS in LaserScan.intensities.
  double intensity = 0.0;
};

/// Sound one beam: apply Rayleigh speckle to the echo level and threshold it.
///
/// Randomness is derived deterministically from (seed, beam_index) rather than
/// from shared mutable generator state, so results are reproducible regardless
/// of evaluation order or threading. The A/B evaluation depends on this.
BeamReturn applySpeckleAndThreshold(
  double range_m, double incidence_rad, const AcousticParams & p,
  uint32_t seed, uint32_t beam_index);
```

- [ ] **Step 4: Implement it**

Add `#include <random>` to `sonar_acoustics.cpp`, then append inside the namespace:

```cpp
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
```

- [ ] **Step 5: Run the tests to verify they pass**

```bash
cd /home/parvu/CaveX-Explorer-Pro/ros2_ws
colcon build --symlink-install --packages-select cavex_sonar
colcon test --packages-select cavex_sonar --event-handlers console_direct+
colcon test-result --verbose --test-result-base build/cavex_sonar
```

Expected: 13 tests, all PASS.

- [ ] **Step 6: Commit**

```bash
cd /home/parvu/CaveX-Explorer-Pro
git add ros2_ws/src/cavex_sonar
git commit -m "Add Rayleigh speckle and detection threshold to sonar model

Randomness is derived from (seed, beam_index) rather than shared generator
state, so output is reproducible regardless of call order or threading --
the A/B current-suppression evaluation depends on that.

A sub-threshold beam reports detected=false and carries no range at all,
rather than a confident wrong range that would inject false constraints
into the SLAM factor graph."
```

---

### Task 5: Beam forming — dense rays to sonar beams

Integrates several adjacent dense `gpu_lidar` rays into one sonar beam and estimates surface normals from the scan, supplying the incidence angle the previous tasks need.

**Files:**
- Create: `ros2_ws/src/cavex_sonar/include/cavex_sonar/beam_former.hpp`
- Create: `ros2_ws/src/cavex_sonar/src/beam_former.cpp`
- Create: `ros2_ws/src/cavex_sonar/test/test_beam_former.cpp`
- Modify: `ros2_ws/src/cavex_sonar/CMakeLists.txt`

**Interfaces:**
- Consumes: `AcousticParams`, `BeamReturn`, `applySpeckleAndThreshold` from Task 4.
- Produces: `struct BeamFormerConfig`, `double incidenceAngleAt(const std::vector<double>&, size_t, double)`, and `std::vector<BeamReturn> formBeams(const std::vector<double>&, const BeamFormerConfig&, const AcousticParams&, uint32_t)`.

- [ ] **Step 1: Write the failing tests**

Create `ros2_ws/src/cavex_sonar/test/test_beam_former.cpp`:

```cpp
#include <gtest/gtest.h>
#include <cmath>
#include <vector>
#include "cavex_sonar/beam_former.hpp"

using cavex_sonar::AcousticParams;
using cavex_sonar::BeamFormerConfig;
using cavex_sonar::formBeams;
using cavex_sonar::incidenceAngleAt;

TEST(Incidence, FlatWallDirectlyAheadIsNearNormal) {
  // A wall perpendicular to the centre ray: ranges rise symmetrically as
  // 1/cos(theta) away from the centre. Incidence at the centre is ~0.
  const double angular_step = 0.01;
  std::vector<double> ranges;
  for (int i = -50; i <= 50; ++i) {
    ranges.push_back(5.0 / std::cos(i * angular_step));
  }
  EXPECT_NEAR(incidenceAngleAt(ranges, 50, angular_step), 0.0, 0.05);
}

TEST(Incidence, ObliqueSurfaceGivesLargerIncidenceThanPerpendicular) {
  const double angular_step = 0.01;
  std::vector<double> perpendicular, oblique;
  for (int i = -50; i <= 50; ++i) {
    perpendicular.push_back(5.0 / std::cos(i * angular_step));
    // A steadily receding surface: strong range gradient == oblique view.
    oblique.push_back(5.0 + 0.5 * i * angular_step * 10.0);
  }
  EXPECT_LT(incidenceAngleAt(perpendicular, 50, angular_step),
            incidenceAngleAt(oblique, 50, angular_step));
}

TEST(Incidence, IsAlwaysWithinZeroToHalfPi) {
  const double angular_step = 0.01;
  std::vector<double> ranges;
  for (int i = -50; i <= 50; ++i) { ranges.push_back(3.0 + 2.0 * std::sin(i * 0.3)); }
  for (size_t i = 0; i < ranges.size(); ++i) {
    const double a = incidenceAngleAt(ranges, i, angular_step);
    EXPECT_GE(a, 0.0);
    EXPECT_LE(a, M_PI / 2.0 + 1e-9);
  }
}

TEST(FormBeams, ReducesDenseRaysToConfiguredBeamCount) {
  BeamFormerConfig cfg;
  cfg.rays_per_beam = 5;
  cfg.beam_count = 20;
  cfg.angular_step_rad = 0.01;
  AcousticParams p;
  std::vector<double> ranges(100, 4.0);
  const auto beams = formBeams(ranges, cfg, p, 7u);
  EXPECT_EQ(beams.size(), 20u);
}

TEST(FormBeams, IgnoresNonFiniteRaysInsteadOfPropagatingThem) {
  // gpu_lidar reports +inf for a ray that hits nothing. Averaging that in
  // would poison the whole beam, so out-of-range rays must be excluded.
  BeamFormerConfig cfg;
  cfg.rays_per_beam = 4;
  cfg.beam_count = 1;
  cfg.angular_step_rad = 0.01;
  AcousticParams p;
  std::vector<double> ranges = {4.0, std::numeric_limits<double>::infinity(), 4.0, 4.0};
  const auto beams = formBeams(ranges, cfg, p, 7u);
  ASSERT_EQ(beams.size(), 1u);
  if (beams[0].detected) { EXPECT_NEAR(beams[0].range_m, 4.0, 1e-6); }
}

TEST(FormBeams, ReportsNoDetectionWhenEveryRayInABeamIsOutOfRange) {
  BeamFormerConfig cfg;
  cfg.rays_per_beam = 3;
  cfg.beam_count = 1;
  cfg.angular_step_rad = 0.01;
  AcousticParams p;
  const double inf = std::numeric_limits<double>::infinity();
  std::vector<double> ranges = {inf, inf, inf};
  const auto beams = formBeams(ranges, cfg, p, 7u);
  ASSERT_EQ(beams.size(), 1u);
  EXPECT_FALSE(beams[0].detected);
}
```

- [ ] **Step 2: Register the test so it builds, then run it to verify it fails**

Add to `ros2_ws/src/cavex_sonar/CMakeLists.txt` — extend the library sources and add the test. Replace the `add_library` line with:

```cmake
add_library(sonar_acoustics src/sonar_acoustics.cpp src/beam_former.cpp)
```

and inside the `if(BUILD_TESTING)` block, after the existing gtest registration:

```cmake
  ament_add_gtest(test_beam_former test/test_beam_former.cpp)
  target_link_libraries(test_beam_former sonar_acoustics)
```

```bash
cd /home/parvu/CaveX-Explorer-Pro/ros2_ws
colcon build --symlink-install --packages-select cavex_sonar
```

Expected: **build failure** — `beam_former.hpp` and `src/beam_former.cpp` do not exist.

- [ ] **Step 3: Write the header**

Create `ros2_ws/src/cavex_sonar/include/cavex_sonar/beam_former.hpp`:

```cpp
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
/// detection.
std::vector<BeamReturn> formBeams(
  const std::vector<double> & ranges, const BeamFormerConfig & cfg,
  const AcousticParams & p, uint32_t seed);

}  // namespace cavex_sonar

#endif  // CAVEX_SONAR__BEAM_FORMER_HPP_
```

- [ ] **Step 4: Write the implementation**

Create `ros2_ws/src/cavex_sonar/src/beam_former.cpp`:

```cpp
#include "cavex_sonar/beam_former.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

namespace cavex_sonar
{

double incidenceAngleAt(
  const std::vector<double> & ranges, std::size_t index, double angular_step_rad)
{
  if (ranges.size() < 2 || angular_step_rad <= 0.0) { return 0.0; }

  // Central difference where possible, one-sided at the ends.
  const std::size_t lo = (index == 0) ? 0 : index - 1;
  const std::size_t hi = std::min(index + 1, ranges.size() - 1);
  if (lo == hi) { return 0.0; }

  const double r = ranges[index];
  if (!std::isfinite(r) || !std::isfinite(ranges[lo]) || !std::isfinite(ranges[hi])) {
    return 0.0;
  }

  const double dr_dtheta =
    (ranges[hi] - ranges[lo]) / (static_cast<double>(hi - lo) * angular_step_rad);

  // For a surface in polar form r(theta), the angle between the line of sight
  // and the surface normal satisfies tan(incidence) = (dr/dtheta) / r.
  if (r <= 0.0) { return 0.0; }
  const double angle = std::atan(std::abs(dr_dtheta) / r);
  return std::clamp(angle, 0.0, M_PI / 2.0);
}

std::vector<BeamReturn> formBeams(
  const std::vector<double> & ranges, const BeamFormerConfig & cfg,
  const AcousticParams & p, uint32_t seed)
{
  std::vector<BeamReturn> beams;
  beams.reserve(cfg.beam_count);
  if (cfg.rays_per_beam == 0) { return beams; }

  for (std::size_t b = 0; b < cfg.beam_count; ++b) {
    const std::size_t begin = b * cfg.rays_per_beam;

    // Integrate the main lobe: mean range and mean incidence over the rays
    // that actually returned. A single infinite ray must not poison the beam.
    double range_sum = 0.0, incidence_sum = 0.0;
    std::size_t valid = 0;
    for (std::size_t k = 0; k < cfg.rays_per_beam; ++k) {
      const std::size_t i = begin + k;
      if (i >= ranges.size()) { break; }
      if (!std::isfinite(ranges[i])) { continue; }
      range_sum += ranges[i];
      incidence_sum += incidenceAngleAt(ranges, i, cfg.angular_step_rad);
      ++valid;
    }

    if (valid == 0) {
      // Every ray in this beam hit nothing: report an honest non-detection.
      beams.push_back(BeamReturn{});
      continue;
    }

    const double mean_range = range_sum / static_cast<double>(valid);
    const double mean_incidence = incidence_sum / static_cast<double>(valid);
    beams.push_back(applySpeckleAndThreshold(
      mean_range, mean_incidence, p, seed, static_cast<uint32_t>(b)));
  }

  return beams;
}

}  // namespace cavex_sonar
```

- [ ] **Step 5: Run the tests to verify they pass**

```bash
cd /home/parvu/CaveX-Explorer-Pro/ros2_ws
colcon build --symlink-install --packages-select cavex_sonar
colcon test --packages-select cavex_sonar --event-handlers console_direct+
colcon test-result --verbose --test-result-base build/cavex_sonar
```

Expected: 19 tests total across both gtest binaries, all PASS.

- [ ] **Step 6: Commit**

```bash
cd /home/parvu/CaveX-Explorer-Pro
git add ros2_ws/src/cavex_sonar
git commit -m "Add beam forming from dense rays to sonar beams

Integrates several adjacent dense rays per beam (the main lobe) and estimates
incidence from the local range gradient via tan(incidence) = (dr/dtheta)/r.

Non-finite rays -- a gpu_lidar ray that hit nothing -- are excluded rather
than averaged in, and a beam whose rays all miss reports an honest
non-detection instead of a fabricated range."
```

---

### Task 6: Mount the dense lidar ray engine on the BlueROV2

**Files:**
- Modify: `ros2_ws/src/cavex_tracked_vehicle/models/bluerov2/model.sdf`
- Modify: `ros2_ws/src/cavex_tracked_vehicle/models/bluerov2/model.sdf.in`
- Modify: `ros2_ws/src/cavex_tracked_vehicle/config/gazebo_tracked_vehicle_bridge.yaml`

**Interfaces:**
- Consumes: nothing from earlier tasks.
- Produces: ROS topic `/bluerov2/sonar_rays` (`sensor_msgs/LaserScan`), 512 dense rays across a 120° fan, consumed by Task 7.

- [ ] **Step 1: Add the sensor to the SDF**

In **both** `model.sdf` and `model.sdf.in`, inside the `base_link` element immediately after the existing `imu_sensor` block, add:

```xml
      <!-- Dense ray engine for the simulated acoustic sonar.
           Gazebo Harmonic ships no sonar sensor type, and gz-sim8's Sensors
           system will not instantiate a custom rendering sensor (the same
           reason gz::sensors::DopplerVelocityLog is unusable here). So this
           gpu_lidar supplies geometrically correct ranges only; all acoustic
           physics -- beam spread, transmission loss, incidence backscatter,
           speckle, dropout -- lives in cavex_sonar and is applied downstream.
           Do not treat this topic as sonar output. -->
      <sensor name="sonar_ray_engine" type="gpu_lidar">
        <pose>0.15 0 0 0 0 0</pose>
        <always_on>1</always_on>
        <update_rate>10</update_rate>
        <topic>bluerov2/sonar_rays</topic>
        <lidar>
          <scan>
            <horizontal>
              <samples>512</samples>
              <resolution>1</resolution>
              <min_angle>-1.0472</min_angle>
              <max_angle>1.0472</max_angle>
            </horizontal>
            <vertical>
              <samples>1</samples>
              <min_angle>0</min_angle>
              <max_angle>0</max_angle>
            </vertical>
          </scan>
          <range>
            <min>0.2</min>
            <max>30.0</max>
            <resolution>0.01</resolution>
          </range>
        </lidar>
      </sensor>
```

Note the fan is 120° (±1.0472 rad) across 512 rays, giving an angular step of about 0.004 rad. Task 7's config must match this.

- [ ] **Step 2: Verify the SDF parses**

```bash
cd /home/parvu/CaveX-Explorer-Pro
source ros2_ws/ardupilot_gazebo_env.sh
gz sdf --check ros2_ws/src/cavex_tracked_vehicle/models/bluerov2/model.sdf
```

Expected: `Valid.` If it reports an unresolved `model://` URI for the meshes, that is expected outside the resource path and is not an SDF error.

- [ ] **Step 3: Add the bridge entry**

Append to `ros2_ws/src/cavex_tracked_vehicle/config/gazebo_tracked_vehicle_bridge.yaml`:

```yaml
- ros_topic_name: "/bluerov2/sonar_rays"
  gz_topic_name: "/bluerov2/sonar_rays"
  ros_type_name: "sensor_msgs/msg/LaserScan"
  gz_type_name: "gz.msgs.LaserScan"
  direction: GZ_TO_ROS
```

- [ ] **Step 4: Verify the sensor produces real returns in simulation**

Launch the simulation as the README documents, then:

```bash
source /opt/ros/jazzy/setup.bash && source ros2_ws/install/setup.bash
ros2 topic hz /bluerov2/sonar_rays
ros2 topic echo /bluerov2/sonar_rays --once --field ranges
```

Expected: about 10 Hz, and 512 range values. Confirm they are **not** all `inf` — that would mean the sensor sees no geometry, which must be fixed here rather than papered over downstream. Record the observed real-time factor.

- [ ] **Step 5: Commit**

```bash
cd /home/parvu/CaveX-Explorer-Pro
git add ros2_ws/src/cavex_tracked_vehicle/models/bluerov2/model.sdf \
        ros2_ws/src/cavex_tracked_vehicle/models/bluerov2/model.sdf.in \
        ros2_ws/src/cavex_tracked_vehicle/config/gazebo_tracked_vehicle_bridge.yaml
git commit -m "Add dense gpu_lidar ray engine to BlueROV2 for sonar simulation

512 rays across a 120-degree fan at 10 Hz. This supplies geometry only --
Gazebo Harmonic has no sonar sensor type and gz-sim8 will not instantiate a
custom rendering sensor, the same constraint that rules out the DVL. All
acoustic physics is applied downstream in cavex_sonar."
```

---

### Task 7: The sonar node

**Files:**
- Create: `ros2_ws/src/cavex_sonar/src/sonar_node.cpp`
- Modify: `ros2_ws/src/cavex_sonar/CMakeLists.txt`

**Interfaces:**
- Consumes: `BeamFormerConfig`, `formBeams` (Task 5); `/bluerov2/sonar_rays` (Task 6).
- Produces: ROS topic `/bluerov2/sonar` (`sensor_msgs/LaserScan`), `ranges` plus echo strength in `intensities`. Consumed by the following plan's SIC-SLAM front-end.

- [ ] **Step 1: Write the node**

Create `ros2_ws/src/cavex_sonar/src/sonar_node.cpp`:

```cpp
// Converts the BlueROV2's dense gpu_lidar returns into simulated acoustic
// sonar beams. The lidar supplies geometry only; every acoustic effect comes
// from the cavex_sonar library, which is deliberately free of ROS and Gazebo
// dependencies so it can be tested off-simulator.
//
// This models a real sonar's behaviour. It is NOT calibrated against hardware.
#include <memory>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>

#include "cavex_sonar/beam_former.hpp"

namespace cavex_sonar
{

class SonarNode : public rclcpp::Node
{
public:
  SonarNode()
  : Node("sonar_node")
  {
    cfg_.rays_per_beam = static_cast<std::size_t>(
      this->declare_parameter<int>("rays_per_beam", 8));
    cfg_.beam_count = static_cast<std::size_t>(
      this->declare_parameter<int>("beam_count", 64));
    params_.absorption_db_per_m =
      this->declare_parameter<double>("absorption_db_per_m", 0.4);
    params_.source_level_db =
      this->declare_parameter<double>("source_level_db", 200.0);
    params_.detection_threshold_db =
      this->declare_parameter<double>("detection_threshold_db", 100.0);
    params_.backscatter_exponent =
      this->declare_parameter<double>("backscatter_exponent", 1.5);
    seed_ = static_cast<uint32_t>(this->declare_parameter<int>("seed", 42));

    pub_ = this->create_publisher<sensor_msgs::msg::LaserScan>("/bluerov2/sonar", 10);
    sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
      "/bluerov2/sonar_rays", rclcpp::SensorDataQoS(),
      [this](sensor_msgs::msg::LaserScan::SharedPtr msg) { this->onRays(*msg); });

    RCLCPP_INFO(
      this->get_logger(),
      "sonar_node ready: /bluerov2/sonar_rays -> /bluerov2/sonar "
      "(%zu beams, %zu rays/beam, seed %u). Simulated acoustic model, "
      "not calibrated against hardware.",
      cfg_.beam_count, cfg_.rays_per_beam, seed_);
  }

private:
  void onRays(const sensor_msgs::msg::LaserScan & in)
  {
    // Derive the angular step from the incoming scan rather than assuming it,
    // so a change to the SDF fan cannot silently desynchronise the incidence
    // estimate from the real geometry.
    cfg_.angular_step_rad = in.angle_increment;

    std::vector<double> ranges(in.ranges.begin(), in.ranges.end());
    const auto beams = formBeams(ranges, cfg_, params_, seed_);

    sensor_msgs::msg::LaserScan out;
    out.header = in.header;
    out.angle_min = in.angle_min;
    out.angle_max = in.angle_max;
    out.angle_increment =
      beams.empty() ? in.angle_increment
                    : (in.angle_max - in.angle_min) / static_cast<float>(beams.size());
    out.time_increment = in.time_increment;
    out.scan_time = in.scan_time;
    out.range_min = in.range_min;
    out.range_max = in.range_max;
    out.ranges.reserve(beams.size());
    out.intensities.reserve(beams.size());

    for (const auto & b : beams) {
      // A non-detection is published as +inf, the LaserScan convention for
      // "no return", never as a fabricated range.
      out.ranges.push_back(
        b.detected ? static_cast<float>(b.range_m)
                   : std::numeric_limits<float>::infinity());
      out.intensities.push_back(static_cast<float>(b.intensity));
    }
    pub_->publish(out);
  }

  BeamFormerConfig cfg_;
  AcousticParams params_;
  uint32_t seed_ = 42;
  rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr pub_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr sub_;
};

}  // namespace cavex_sonar

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<cavex_sonar::SonarNode>());
  rclcpp::shutdown();
  return 0;
}
```

- [ ] **Step 2: Register the executable**

Add to `ros2_ws/src/cavex_sonar/CMakeLists.txt`, before the `if(BUILD_TESTING)` block:

```cmake
add_executable(sonar_node src/sonar_node.cpp)
target_link_libraries(sonar_node sonar_acoustics)
ament_target_dependencies(sonar_node rclcpp sensor_msgs)

install(TARGETS sonar_node DESTINATION lib/${PROJECT_NAME})
```

- [ ] **Step 3: Build**

```bash
cd /home/parvu/CaveX-Explorer-Pro/ros2_ws
colcon build --symlink-install --packages-select cavex_sonar
```

Expected: builds clean, no warnings from `-Wall -Wextra -Wpedantic`.

- [ ] **Step 4: Verify against the running simulation**

With the simulation running:

```bash
source /opt/ros/jazzy/setup.bash && source ros2_ws/install/setup.bash
ros2 run cavex_sonar sonar_node &
ros2 topic hz /bluerov2/sonar
ros2 topic echo /bluerov2/sonar --once
```

Expected: 64 values in `ranges` and 64 in `intensities` at about 10 Hz. Confirm a mixture of finite ranges and `inf` dropouts — all-finite would mean the threshold is too permissive to be modelling anything, all-`inf` too strict. Record the dropout fraction; the following plan needs it to reason about degeneracy.

- [ ] **Step 5: Commit**

```bash
cd /home/parvu/CaveX-Explorer-Pro
git add ros2_ws/src/cavex_sonar
git commit -m "Add sonar_node: dense lidar rays to simulated acoustic beams

Publishes /bluerov2/sonar as sensor_msgs/LaserScan with echo strength in
intensities, so stock ros_gz_bridge and standard tooling carry it with no
custom message type.

The angular step is derived from the incoming scan rather than assumed, so
changing the SDF fan cannot silently desynchronise the incidence estimate
from the real geometry. Non-detections are published as +inf per the
LaserScan convention, never as a fabricated range."
```

---

### Task 8: Ocean current

Uses the stock `Hydrodynamics` plugin's existing current support. No new physics code.

**Files:**
- Modify: `ros2_ws/src/cavex_tracked_vehicle/models/bluerov2/model.sdf`
- Modify: `ros2_ws/src/cavex_tracked_vehicle/models/bluerov2/model.sdf.in`
- Create: `ros2_ws/src/cavex_sonar/cavex_sonar/current_field_node.py`
- Create: `ros2_ws/src/cavex_sonar/test/test_current_profiles.py`
- Modify: `ros2_ws/src/cavex_sonar/CMakeLists.txt`

**Interfaces:**
- Consumes: nothing from earlier tasks.
- Produces: Gazebo topic `/ocean_current` (`gz.msgs.Vector3d`), ROS topic `/cavex/current_ground_truth` (`geometry_msgs/Vector3Stamped`, **scorer-only**), and `current_at(profile, t, params) -> (x, y, z)`.

- [ ] **Step 1: Write the failing test for the profile maths**

Create `ros2_ws/src/cavex_sonar/test/test_current_profiles.py`:

```python
"""Unit tests for the current profile generator in current_field_node.py.

Pure maths, no ROS: the profile functions are deliberately importable without
a running ROS graph so they can be tested directly.
"""
import math
import pytest
from cavex_sonar.current_field_node import current_at


def test_constant_profile_is_time_invariant():
    p = {"vx": 0.3, "vy": -0.1, "vz": 0.0}
    assert current_at("constant", 0.0, p) == pytest.approx((0.3, -0.1, 0.0))
    assert current_at("constant", 123.4, p) == pytest.approx((0.3, -0.1, 0.0))


def test_step_profile_is_zero_before_the_step_and_full_after():
    p = {"vx": 0.4, "vy": 0.0, "vz": 0.0, "step_time": 10.0}
    assert current_at("step", 9.9, p) == pytest.approx((0.0, 0.0, 0.0))
    assert current_at("step", 10.1, p) == pytest.approx((0.4, 0.0, 0.0))


def test_sinusoidal_profile_oscillates_within_amplitude():
    p = {"vx": 0.5, "vy": 0.0, "vz": 0.0, "period_s": 20.0}
    for t in [i * 0.5 for i in range(80)]:
        x, _, _ = current_at("sinusoidal", t, p)
        assert -0.5 - 1e-9 <= x <= 0.5 + 1e-9


def test_sinusoidal_profile_completes_one_cycle_per_period():
    p = {"vx": 0.5, "vy": 0.0, "vz": 0.0, "period_s": 20.0}
    assert current_at("sinusoidal", 0.0, p)[0] == pytest.approx(
        current_at("sinusoidal", 20.0, p)[0], abs=1e-9)


def test_unknown_profile_raises_rather_than_silently_returning_zero():
    # A typo in a launch argument must fail loudly, not quietly disable the
    # disturbance and invalidate an entire evaluation run.
    with pytest.raises(ValueError):
        current_at("sinusiodal", 1.0, {"vx": 0.5})
```

- [ ] **Step 2: Run it to verify it fails**

```bash
cd /home/parvu/CaveX-Explorer-Pro/ros2_ws/src/cavex_sonar
python3 -m pytest test/test_current_profiles.py -v
```

Expected: `ModuleNotFoundError` or import error — `current_field_node` does not exist.

- [ ] **Step 3: Write the node**

Create `ros2_ws/src/cavex_sonar/cavex_sonar/__init__.py` as an empty file, then create `ros2_ws/src/cavex_sonar/cavex_sonar/current_field_node.py`:

```python
#!/usr/bin/env python3
"""
current_field_node.py

Drives the water current the BlueROV2 experiences, by publishing to the
Gazebo topic /ocean_current that the stock Hydrodynamics plugin already
supports. No new physics is implemented here -- the force on the vehicle is
computed by upstream Gazebo, not by this project.

Also republishes the commanded current on /cavex/current_ground_truth for
EVALUATION ONLY. No perception or control node may ever subscribe to that
topic; it exists so a scorer can compare an estimate against the truth, the
same discipline this project already applies to ground-truth pose.
"""
import math

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Vector3Stamped


def current_at(profile, t, params):
    """Return the (x, y, z) water current in m/s at time t seconds.

    Pure function, no ROS, so it is directly unit-testable.
    """
    vx = params.get("vx", 0.0)
    vy = params.get("vy", 0.0)
    vz = params.get("vz", 0.0)

    if profile == "constant":
        return (vx, vy, vz)
    if profile == "step":
        return (vx, vy, vz) if t >= params.get("step_time", 10.0) else (0.0, 0.0, 0.0)
    if profile == "sinusoidal":
        period = params.get("period_s", 20.0)
        s = math.sin(2.0 * math.pi * t / period)
        return (vx * s, vy * s, vz * s)

    # Fail loudly. A typo in a launch argument must not silently disable the
    # disturbance and quietly invalidate a whole evaluation run.
    raise ValueError(
        f"unknown current profile {profile!r}; "
        "expected 'constant', 'step' or 'sinusoidal'")


class CurrentFieldNode(Node):
    def __init__(self):
        super().__init__("current_field_node")
        self.declare_parameter("profile", "constant")
        self.declare_parameter("vx", 0.3)
        self.declare_parameter("vy", 0.0)
        self.declare_parameter("vz", 0.0)
        self.declare_parameter("step_time", 10.0)
        self.declare_parameter("period_s", 20.0)
        self.declare_parameter("publish_rate_hz", 10.0)

        self.profile = self.get_parameter("profile").value
        self.params = {
            "vx": self.get_parameter("vx").value,
            "vy": self.get_parameter("vy").value,
            "vz": self.get_parameter("vz").value,
            "step_time": self.get_parameter("step_time").value,
            "period_s": self.get_parameter("period_s").value,
        }

        # Validate the profile once at startup rather than failing per-tick.
        current_at(self.profile, 0.0, self.params)

        self.truth_pub = self.create_publisher(
            Vector3Stamped, "/cavex/current_ground_truth", 10)
        self.t0 = self.get_clock().now()
        rate = self.get_parameter("publish_rate_hz").value
        self.timer = self.create_timer(1.0 / rate, self._tick)

        self.get_logger().info(
            f"current_field_node: profile={self.profile} params={self.params}. "
            "Publishing to Gazebo /ocean_current and, for scoring only, "
            "/cavex/current_ground_truth.")

    def _tick(self):
        t = (self.get_clock().now() - self.t0).nanoseconds * 1e-9
        vx, vy, vz = current_at(self.profile, t, self.params)

        msg = Vector3Stamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "map"
        msg.vector.x, msg.vector.y, msg.vector.z = vx, vy, vz
        self.truth_pub.publish(msg)
        self._publish_to_gazebo(vx, vy, vz)

    def _publish_to_gazebo(self, vx, vy, vz):
        # Published via `gz topic` rather than a ros_gz_bridge entry: the
        # bridge maps ROS->GZ per-topic in a config file, and this is the only
        # GZ-bound topic in the project, so a subprocess keeps the bridge
        # config untouched. Verified against the real topic at Step 5.
        import subprocess
        subprocess.run(
            ["gz", "topic", "-t", "/ocean_current",
             "-m", "gz.msgs.Vector3d",
             "-p", f"x: {vx}, y: {vy}, z: {vz}"],
            check=False, capture_output=True)


def main(args=None):
    rclpy.init(args=args)
    node = CurrentFieldNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
```

- [ ] **Step 4: Wire the Python package into the build, then run the tests**

Add to `ros2_ws/src/cavex_sonar/CMakeLists.txt`, before `ament_package()`:

```cmake
ament_python_install_package(${PROJECT_NAME})

install(PROGRAMS
  cavex_sonar/current_field_node.py
  DESTINATION lib/${PROJECT_NAME}
)
```

```bash
cd /home/parvu/CaveX-Explorer-Pro/ros2_ws/src/cavex_sonar
python3 -m pytest test/test_current_profiles.py -v
```

Expected: 5 tests, all PASS.

- [ ] **Step 5: Add the default current to the SDF and verify the topic is live**

In **both** `model.sdf` and `model.sdf.in`, inside the `Hydrodynamics` plugin block, immediately after `<water_density>`, add:

```xml
      <!-- Baseline water current. Overridden at runtime by current_field_node
           publishing to the /ocean_current topic this plugin already listens
           on -- current support is stock upstream Gazebo, not added here. -->
      <default_current>0 0 0</default_current>
```

With the simulation running, confirm the plugin really is listening:

```bash
source ros2_ws/ardupilot_gazebo_env.sh
gz topic -l | grep ocean_current
gz topic -t /ocean_current -m gz.msgs.Vector3d -p "x: 0.3, y: 0.0, z: 0.0"
```

Expected: the topic is listed, and with a current applied the stationary ROV visibly drifts. If `gz topic -l` does not list it, check the plugin's expected topic name in its own source before changing anything downstream.

- [ ] **Step 6: Commit**

```bash
cd /home/parvu/CaveX-Explorer-Pro
git add ros2_ws/src/cavex_sonar \
        ros2_ws/src/cavex_tracked_vehicle/models/bluerov2/model.sdf \
        ros2_ws/src/cavex_tracked_vehicle/models/bluerov2/model.sdf.in
git commit -m "Add ocean current driver using stock Hydrodynamics support

The Hydrodynamics plugin already implements ocean current and listens on
/ocean_current, so no new physics is written here -- current_field_node only
scripts constant, step and sinusoidal profiles for repeatable experiments.

An unknown profile name raises rather than returning zero, so a typo in a
launch argument fails loudly instead of silently disabling the disturbance
and invalidating an evaluation run.

/cavex/current_ground_truth is published for scoring only and must never be
subscribed to by a perception or control node."
```

---

### Task 9: Launch integration and honest documentation

**Files:**
- Create: `ros2_ws/src/cavex_sonar/launch/sonar_and_current.launch.py`
- Modify: `ros2_ws/src/cavex_sonar/CMakeLists.txt`
- Modify: `README.md`

**Interfaces:**
- Consumes: `sonar_node` (Task 7), `current_field_node.py` (Task 8).
- Produces: one launch file bringing up both.

- [ ] **Step 1: Write the launch file**

Create `ros2_ws/src/cavex_sonar/launch/sonar_and_current.launch.py`:

```python
"""Brings up the simulated acoustic sonar and the water current driver.

Run alongside the existing tracked-vehicle simulation launch, once the
BlueROV2 has been released into the water section.
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    profile = LaunchConfiguration("profile")
    vx = LaunchConfiguration("vx")
    seed = LaunchConfiguration("seed")

    return LaunchDescription([
        DeclareLaunchArgument("profile", default_value="constant",
                              description="constant, step or sinusoidal"),
        DeclareLaunchArgument("vx", default_value="0.3",
                              description="current along world X, m/s"),
        DeclareLaunchArgument("seed", default_value="42",
                              description="sonar speckle seed; fix for reproducible runs"),
        Node(
            package="cavex_sonar",
            executable="sonar_node",
            name="sonar_node",
            output="screen",
            parameters=[{"seed": seed}],
        ),
        Node(
            package="cavex_sonar",
            executable="current_field_node.py",
            name="current_field_node",
            output="screen",
            parameters=[{"profile": profile, "vx": vx}],
        ),
    ])
```

- [ ] **Step 2: Install the launch directory**

Add to `ros2_ws/src/cavex_sonar/CMakeLists.txt`, before `ament_package()`:

```cmake
install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})
```

- [ ] **Step 3: Build and verify the launch works end to end**

```bash
cd /home/parvu/CaveX-Explorer-Pro/ros2_ws
colcon build --symlink-install --packages-select cavex_sonar
source install/setup.bash
ros2 launch cavex_sonar sonar_and_current.launch.py profile:=constant vx:=0.3
```

In another terminal, confirm both outputs are live:

```bash
ros2 topic hz /bluerov2/sonar
ros2 topic echo /cavex/current_ground_truth --once
```

Expected: sonar at about 10 Hz; current ground truth reporting `x: 0.3`.

- [ ] **Step 4: Update the README honestly**

In `README.md`, replace the existing `**On "sonar"**:` paragraph (currently stating there is no sonar in the simulation) with:

```markdown
**On "sonar"**: the BlueROV2 now carries a simulated acoustic sonar
(`cavex_sonar`), publishing real per-beam ranges and echo strengths on
`/bluerov2/sonar`. Because Gazebo Harmonic ships no sonar sensor type — and
`gz-sim8` will not instantiate a custom rendering sensor, the same constraint
that rules out its DVL — a dense `gpu_lidar` supplies the ray geometry, and the
acoustic physics (beam spread, spherical spreading and absorption,
incidence-dependent backscatter, Rayleigh speckle, detection threshold and
dropout) is applied on top by the `sonar_acoustics` library. That library has
no ROS or Gazebo dependencies and is unit-tested off-simulator.

This is a physically-motivated model, **not calibrated against real sonar
hardware**, and it has not been validated against real sonar data. Cite it as a
simulation result. The frontend's `sonarActive`/`sonarDepth`/`sonarEchoStrength`
fields remain concept-demo values and are still not wired to this topic.

**On water current**: current is real upstream Gazebo physics — the stock
`Hydrodynamics` plugin's own ocean-current support, driven by
`current_field_node.py` over the `/ocean_current` topic. The disturbance is not
faked, but it is one we inject ourselves rather than a measured real-world flow.
```

- [ ] **Step 5: Run the full test suite once more**

```bash
cd /home/parvu/CaveX-Explorer-Pro/ros2_ws
colcon test --packages-select cavex_sonar --event-handlers console_direct+
colcon test-result --verbose --test-result-base build/cavex_sonar
cd src/cavex_sonar && python3 -m pytest test/ -v
```

Expected: all gtest and pytest tests PASS. Report the actual counts; do not claim success without reading the output.

- [ ] **Step 6: Commit**

```bash
cd /home/parvu/CaveX-Explorer-Pro
git add ros2_ws/src/cavex_sonar README.md
git commit -m "Add sonar+current launch file and update README honestly

Replaces the README's 'there is no sonar in this simulation' note with an
accurate description: the sonar is real within the simulation, built on a
gpu_lidar ray engine because Gazebo Harmonic has no sonar sensor type, and
is explicitly NOT calibrated against real hardware.

Records that water current is stock upstream Gazebo physics rather than
something this project fakes, while being clear it is an injected
disturbance and not a measured real-world flow."
```

---

## Self-Review

**Spec coverage.** This plan implements the spec's Component 1 (Tasks 2-7) and Component 2 (Task 8), and de-risks Component 3 via the Task 1 spike. Components 3 (SIC-SLAM factor graph) and 4 (DCS controller), plus the evaluation harness and the naming cleanup, are **deliberately deferred to a following plan** — see below.

**Why the split.** The spec identifies featureless-tunnel degeneracy as its highest risk and explicitly requires it be measured, not assumed. The SIC-SLAM registration front-end, the current random-walk process noise, and the DCS confidence gate should all be designed against the sonar's *actual* measured behaviour — dropout fraction, return quality along a smooth tunnel wall, achieved real-time factor. Tasks 6, 7 and 8 produce exactly those numbers. Writing the factor-graph plan before they exist would mean inventing tuning constants and then rewriting them.

**Placeholder scan.** No TBDs, no "add error handling", no "similar to Task N". Every code step carries real code; every test step carries real assertions and the exact command plus expected outcome.

**Type consistency.** `AcousticParams` gains fields across Tasks 2-4 and is used consistently. `BeamReturn{detected, range_m, intensity}` is defined in Task 4 and consumed unchanged in Tasks 5 and 7. `BeamFormerConfig{rays_per_beam, beam_count, angular_step_rad}` is defined in Task 5 and used in Task 7. `current_at(profile, t, params)` is defined and tested in Task 8. The `/bluerov2/sonar_rays` topic produced in Task 6 is the topic consumed in Task 7; the 120° / 512-ray fan set there matches the angular step Task 7 derives at runtime rather than hardcodes.
