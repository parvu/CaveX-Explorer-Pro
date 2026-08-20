// Converts the BlueROV2's dense gpu_lidar returns into simulated acoustic
// sonar beams. The lidar supplies geometry only; every acoustic effect comes
// from the cavex_sonar library, which is deliberately free of ROS and Gazebo
// dependencies so it can be tested off-simulator.
//
// This models a real sonar's behaviour. It is NOT calibrated against hardware.
#include <limits>
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
      [this](sensor_msgs::msg::LaserScan::SharedPtr msg) {this->onRays(*msg);});

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
        in.angle_min, in.angle_increment, cfg_.rays_per_beam, beams.size());
      out.angle_min = static_cast<float>(geom.angle_min);
      out.angle_increment = static_cast<float>(geom.angle_increment);
      out.angle_max = static_cast<float>(
        geom.angle_min + static_cast<double>(beams.size() - 1) * geom.angle_increment);
    }
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
        b.detected ? static_cast<float>(b.range_m) :
          std::numeric_limits<float>::infinity());
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
