// Converts the BlueROV2's dense gpu_lidar returns into simulated acoustic
// sonar beams. The lidar supplies geometry only; every acoustic effect comes
// from the cavex_sonar library, which is deliberately free of ROS and Gazebo
// dependencies so it can be tested off-simulator.
//
// This models a real sonar's behaviour. It is NOT calibrated against hardware.
//
// Frame ID: by default the output LaserScan carries the input scan's
// header.frame_id verbatim (typically Gazebo's scoped sensor name). Setting
// the `frame_id` parameter to a non-empty string overrides it, e.g. to
// publish under a stable TF frame such as `bluerov2/sonar` that a
// static_transform_publisher ties to `base_link`.
//
// A total non-detection beam (formBeams() found no valid rays at all) is
// published with intensity -inf, never 0.0 -- consumers must check
// isfinite() before doing arithmetic on LaserScan.intensities.
#include <cmath>
#include <limits>
#include <memory>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <geometry_msgs/msg/vector3.hpp>

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
    params_.clutter_probability =
      this->declare_parameter<double>("clutter_probability", 0.0);
    params_.clutter_max_range_m =
      this->declare_parameter<double>("clutter_max_range_m", 2.0);
    // Real, informational per-beam timing (real gap, 2026-08-22): the
    // underlying gz-sim gpu_lidar source samples every ray at one
    // simulated instant -- there is no real motion distortion within a
    // scan to correct for here. This does NOT fabricate that distortion;
    // it populates LaserScan's own standard time_increment/scan_time
    // fields with the timing a real mechanically-scanned sonar (e.g.
    // Ping360) would have for this many beams over scan_duration_s, so a
    // consumer that DOES want motion-distortion correction has the real
    // field a real sensor driver would provide, instead of it silently
    // being left at 0.0/uninitialized.
    scan_duration_s_ = this->declare_parameter<double>("scan_duration_s", 1.0);
    seed_ = static_cast<uint32_t>(this->declare_parameter<int>("seed", 42));
    frame_id_ = this->declare_parameter<std::string>("frame_id", "");

    pub_ = this->create_publisher<sensor_msgs::msg::LaserScan>("/bluerov2/sonar", 10);
    sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
      "/bluerov2/sonar_rays", rclcpp::SensorDataQoS(),
      [this](sensor_msgs::msg::LaserScan::SharedPtr msg) {this->onRays(*msg);});
    // Real current, for clutter drift only (real request, 2026-08-22) --
    // this is simulator-side sensor-noise physics (analogous to how
    // gz-sim's own Hydrodynamics plugin already consumes this same
    // topic), not a perception node reading ground truth to cheat.
    // Defaults to (0,0) until a message arrives, matching
    // current_direction_rad=0/current_drift_range_m=0's documented
    // no-op default.
    current_sub_ = this->create_subscription<geometry_msgs::msg::Vector3>(
      "/ocean_current", 10,
      [this](geometry_msgs::msg::Vector3::SharedPtr msg) {
        current_vx_ = msg->x;
        current_vy_ = msg->y;
      });
    start_time_ = this->now();

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
    cfg_.angle_min_rad = in.angle_min;

    // Real current direction/drift for clutter only (real request,
    // 2026-08-22) -- current_vx_/current_vy_ default to 0,0 until a
    // message arrives on /ocean_current, which makes current_drift_range_m
    // 0.0 too, a documented no-op matching clutter_probability's own
    // default-off behavior.
    params_.current_direction_rad = std::atan2(current_vy_, current_vx_);
    const double current_speed = std::hypot(current_vx_, current_vy_);
    const double elapsed_s = (this->now() - start_time_).seconds();
    params_.current_drift_range_m = current_speed * elapsed_s;

    if (in.ranges.empty()) {
      RCLCPP_WARN_ONCE(
        this->get_logger(),
        "sonar_node: incoming scan on /bluerov2/sonar_rays has no rays -- "
        "the sonar ray engine produced no returns. Dropping this scan.");
      return;
    }

    std::vector<double> ranges(in.ranges.begin(), in.ranges.end());
    const auto beams = formBeams(ranges, cfg_, params_, seed_, ping_index_++);

    sensor_msgs::msg::LaserScan out;
    out.header = in.header;
    if (!frame_id_.empty()) {
      out.header.frame_id = frame_id_;
    }

    // Each beam integrates rays_per_beam dense rays, so its true bearing is
    // the centre of that group, not the group's start. beamScanGeometry()
    // computes that centre offset so ranges[i] decodes to the beam's actual
    // pointing direction, not a naive even split of the input fan.
    if (beams.empty()) {
      out.angle_min = in.angle_min;
      out.angle_max = in.angle_max;
      out.angle_increment = in.angle_increment;
    } else {
      const auto geom = beamScanGeometry(
        in.angle_min, in.angle_increment, cfg_.rays_per_beam);
      out.angle_min = static_cast<float>(geom.angle_min);
      out.angle_increment = static_cast<float>(geom.angle_increment);
      out.angle_max = static_cast<float>(
        geom.angle_min + static_cast<double>(beams.size() - 1) * geom.angle_increment);
    }
    // Real, informational per-beam timing (see the constructor comment for
    // why this doesn't fabricate real motion distortion). Falls back to
    // passing the input scan's own values through if there are no beams
    // to time.
    if (beams.empty()) {
      out.time_increment = in.time_increment;
      out.scan_time = in.scan_time;
    } else {
      out.time_increment = static_cast<float>(scan_duration_s_ / static_cast<double>(beams.size()));
      out.scan_time = static_cast<float>(scan_duration_s_);
    }
    out.range_min = in.range_min;
    out.range_max = in.range_max;
    out.ranges.reserve(beams.size());
    out.intensities.reserve(beams.size());

    for (const auto & b : beams) {
      // A non-detection is published as +inf, the LaserScan convention for
      // "no return", never as a fabricated range.
      out.ranges.push_back(
        b.detected ? static_cast<float>(b.range_m) :
          std::numeric_limits<float>::infinity());
      out.intensities.push_back(static_cast<float>(b.intensity));
    }
    pub_->publish(out);
  }

  BeamFormerConfig cfg_;
  AcousticParams params_;
  uint32_t seed_ = 42;
  std::string frame_id_;
  // Monotonically increasing per-scan counter, fed into the speckle seed so
  // consecutive pings draw independent Rayleigh samples instead of freezing
  // into a static per-beam bias (see the FIX 1 note in sonar_acoustics.hpp).
  uint32_t ping_index_ = 0;
  rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr pub_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr sub_;
  rclcpp::Subscription<geometry_msgs::msg::Vector3>::SharedPtr current_sub_;
  double current_vx_ = 0.0;
  double current_vy_ = 0.0;
  rclcpp::Time start_time_;
  double scan_duration_s_ = 1.0;
};

}  // namespace cavex_sonar

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<cavex_sonar::SonarNode>());
  rclcpp::shutdown();
  return 0;
}
