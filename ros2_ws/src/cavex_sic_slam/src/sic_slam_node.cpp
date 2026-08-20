#include <memory>
#include <mutex>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <sensor_msgs/msg/point_cloud.hpp>
#include <geometry_msgs/msg/pose_array.hpp>
#include <geometry_msgs/msg/point32.hpp>
#include <geometry_msgs/msg/twist_with_covariance_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <std_msgs/msg/float64.hpp>

#include <gtsam/navigation/CombinedImuFactor.h>
#include <gtsam/nonlinear/ISAM2.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/nonlinear/Marginals.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/slam/BetweenFactor.h>

#include "cavex_sic_slam/scan_registration.hpp"
#include "cavex_sic_slam/current_factor.hpp"

using gtsam::symbol_shorthand::X;
using gtsam::symbol_shorthand::V;
using gtsam::symbol_shorthand::B;
using gtsam::symbol_shorthand::C;

namespace cavex_sic_slam
{

class SicSlamNode : public rclcpp::Node
{
public:
  SicSlamNode()
  : Node("sic_slam_node"),
    keyframe_index_(0),
    have_prev_imu_time_(false)
  {
    keyframe_period_s_ = this->declare_parameter<double>("keyframe_period_s", 0.5);
    keyframe_distance_m_ = this->declare_parameter<double>("keyframe_distance_m", 0.3);
    keyframe_rotation_rad_ = this->declare_parameter<double>("keyframe_rotation_rad", 0.2);

    auto imu_params = gtsam::PreintegratedCombinedMeasurements::Params::MakeSharedU(9.81);
    // Roll-180 mount into the ArduPilot body frame: accel/gyro measurements
    // must be rotated into the graph's body frame before preintegration.
    imu_params->setBodyPSensor(gtsam::Pose3(
        gtsam::Rot3::Rx(M_PI), gtsam::Point3(0, 0, 0)));
    imu_params_ = imu_params;
    gtsam::imuBias::ConstantBias zero_bias;
    preint_ = std::make_shared<gtsam::PreintegratedCombinedMeasurements>(
      imu_params_, zero_bias);

    gtsam::ISAM2Params isam_params;
    isam_ = gtsam::ISAM2(isam_params);

    gtsam::Pose3 prior_pose = gtsam::Pose3::Identity();
    gtsam::Vector3 prior_vel = gtsam::Vector3::Zero();
    gtsam::imuBias::ConstantBias prior_bias;
    values_.insert(X(0), prior_pose);
    values_.insert(V(0), prior_vel);
    values_.insert(B(0), prior_bias);

    auto pose_noise = gtsam::noiseModel::Diagonal::Sigmas(
      (gtsam::Vector(6) << 0.05, 0.05, 0.05, 0.1, 0.1, 0.1).finished());
    auto vel_noise = gtsam::noiseModel::Isotropic::Sigma(3, 0.1);
    auto bias_noise = gtsam::noiseModel::Isotropic::Sigma(6, 0.01);
    graph_.addPrior(X(0), prior_pose, pose_noise);
    graph_.addPrior(V(0), prior_vel, vel_noise);
    graph_.addPrior(B(0), prior_bias, bias_noise);

    gtsam::Vector3 prior_current = gtsam::Vector3::Zero();
    values_.insert(C(0), prior_current);
    auto current_noise = gtsam::noiseModel::Isotropic::Sigma(3, 0.05);
    graph_.addPrior(C(0), prior_current, current_noise);

    isam_.update(graph_, values_);
    graph_.resize(0);
    values_.clear();
    last_pose_ = prior_pose;
    last_vel_ = prior_vel;
    last_bias_ = prior_bias;
    last_current_ = prior_current;

    thruster_geom_ = cavex_sic_slam::defaultBlueRov2Geometry();
    thruster_drag_ = cavex_sic_slam::defaultBlueRov2Drag();
    thrust_n_.fill(0.0);
    for (int i = 0; i < 6; ++i) {
      std::string topic = "/bluerov2/thruster" + std::to_string(i + 1) + "/cmd_thrust";
      thruster_subs_[i] = this->create_subscription<std_msgs::msg::Float64>(
        topic, rclcpp::SensorDataQoS(),
        [this, i](const std_msgs::msg::Float64::SharedPtr msg) {
          std::lock_guard<std::mutex> lock(mutex_);
          thrust_n_[i] = msg->data;
        });
    }
    current_pub_ = this->create_publisher<geometry_msgs::msg::TwistWithCovarianceStamped>(
      "/sic_slam/current_estimate", 10);

    imu_sub_ = this->create_subscription<sensor_msgs::msg::Imu>(
      "/bluerov2/imu", rclcpp::SensorDataQoS(),
      std::bind(&SicSlamNode::imuCallback, this, std::placeholders::_1));
    odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>("/sic_slam/odometry", 10);

    sonar_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
      "/bluerov2/sonar", rclcpp::SensorDataQoS(),
      std::bind(&SicSlamNode::sonarCallback, this, std::placeholders::_1));
    map_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud>("/sic_slam/map", 10);
    keyframes_pub_ = this->create_publisher<geometry_msgs::msg::PoseArray>(
      "/sic_slam/keyframes", 10);

    RCLCPP_INFO(
      this->get_logger(),
      "sic_slam_node ready: /bluerov2/imu -> /sic_slam/odometry (IMU-only graph).");
  }

protected:
  void imuCallback(const sensor_msgs::msg::Imu::SharedPtr msg)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    rclcpp::Time stamp(msg->header.stamp);
    if (!have_prev_imu_time_) {
      prev_imu_time_ = stamp;
      have_prev_imu_time_ = true;
      return;
    }
    double dt = (stamp - prev_imu_time_).seconds();
    prev_imu_time_ = stamp;
    if (dt <= 0.0 || dt > 1.0) {
      return;
    }

    gtsam::Vector3 accel(
      msg->linear_acceleration.x, msg->linear_acceleration.y, msg->linear_acceleration.z);
    gtsam::Vector3 gyro(
      msg->angular_velocity.x, msg->angular_velocity.y, msg->angular_velocity.z);
    preint_->integrateMeasurement(accel, gyro, dt);

    accumulated_s_ += dt;
    // Keyframe on whichever threshold fires first -- time alone would
    // under-sample a fast, straight run and over-sample a slow, twisty
    // one; distance/rotation catch what a fixed period misses.
    gtsam::NavState delta = preint_->deltaXij();
    double dist = delta.pose().translation().norm();
    double rot = gtsam::Rot3::Logmap(delta.pose().rotation()).norm();
    if (accumulated_s_ >= keyframe_period_s_ ||
      dist >= keyframe_distance_m_ || rot >= keyframe_rotation_rad_)
    {
      addKeyframe(stamp);
      accumulated_s_ = 0.0;
    }
  }

  void sonarCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    latest_scan_ = msg;
  }

  // Extended in Task 5 (sonar BetweenFactor) and Task 6 (CurrentFactor,
  // current random walk, thruster subscriptions). Kept as its own method so
  // later tasks can insert factors between the IMU factor and the
  // isam_.update() call without restructuring the callback.
  virtual void addKeyframe(const rclcpp::Time & stamp)
  {
    std::size_t prev = keyframe_index_;
    std::size_t curr = keyframe_index_ + 1;

    gtsam::CombinedImuFactor imu_factor(
      X(prev), V(prev), X(curr), V(curr), B(prev), B(curr), *preint_);
    graph_.add(imu_factor);

    gtsam::NavState prev_state(last_pose_, last_vel_);
    gtsam::NavState predicted = preint_->predict(prev_state, last_bias_);
    values_.insert(X(curr), predicted.pose());
    values_.insert(V(curr), predicted.velocity());
    values_.insert(B(curr), last_bias_);

    std::vector<cavex_sic_slam::ScanPoint> curr_scan_points;
    if (latest_scan_) {
      // LaserScan stores ranges/intensities as float; registerScans/
      // laserScanToPoints operate on double.
      std::vector<double> ranges(latest_scan_->ranges.begin(), latest_scan_->ranges.end());
      std::vector<double> intensities(
        latest_scan_->intensities.begin(), latest_scan_->intensities.end());
      curr_scan_points = cavex_sic_slam::laserScanToPoints(
        ranges, intensities, latest_scan_->angle_min, latest_scan_->angle_increment);
    }

    if (!curr_scan_points.empty() && !prev_keyframe_scan_.empty()) {
      auto reg = cavex_sic_slam::registerScans(
        curr_scan_points, prev_keyframe_scan_, 0.5, 20);
      if (reg.converged) {
        gtsam::Pose3 odom_delta(
          gtsam::Rot3::Yaw(reg.dyaw), gtsam::Point3(reg.dx, reg.dy, 0.0));
        // roll,pitch,yaw sigma (rad), then x,y,z sigma (m) -- z loose,
        // sonar can't observe depth.
        auto sonar_noise = gtsam::noiseModel::Diagonal::Sigmas(
          (gtsam::Vector(6) << 0.3, 0.3, 0.02, 0.1, 0.1, 0.5).finished());
        graph_.add(gtsam::BetweenFactor<gtsam::Pose3>(
          X(prev), X(curr), odom_delta, sonar_noise));
      }

      // Loop closure: try registering against earlier, non-adjacent
      // keyframes. Gate on match quality so a bad match never enters the
      // graph as a confident constraint.
      for (std::size_t hist_index = 0; hist_index < keyframe_history_.size(); ++hist_index) {
        const auto & [hist_pose, hist_scan] = keyframe_history_[hist_index];
        auto loop_reg = cavex_sic_slam::registerScans(
          curr_scan_points, hist_scan, 0.5, 20);
        if (loop_reg.converged && loop_reg.matched_fraction > 0.7 &&
          loop_reg.mean_residual < 0.15)
        {
          gtsam::Pose3 loop_delta(
            gtsam::Rot3::Yaw(loop_reg.dyaw), gtsam::Point3(loop_reg.dx, loop_reg.dy, 0.0));
          auto loop_noise = gtsam::noiseModel::Diagonal::Sigmas(
            (gtsam::Vector(6) << 0.3, 0.3, 0.05, 0.15, 0.15, 0.6).finished());
          graph_.add(gtsam::BetweenFactor<gtsam::Pose3>(
            X(hist_index), X(curr), loop_delta, loop_noise));
        }
      }
    }

    std::array<double, 6> thrust_snapshot = thrust_n_;
    auto current_factor_noise = gtsam::noiseModel::Isotropic::Sigma(3, 0.15);
    graph_.add(cavex_sic_slam::CurrentFactor(
      X(curr), V(curr), C(curr), thrust_snapshot, thruster_geom_, thruster_drag_,
      current_factor_noise));

    auto current_walk_noise = gtsam::noiseModel::Isotropic::Sigma(3, 0.02);
    graph_.add(gtsam::BetweenFactor<gtsam::Vector3>(
      C(prev), C(curr), gtsam::Vector3::Zero(), current_walk_noise));

    values_.insert(C(curr), last_current_);

    isam_.update(graph_, values_);
    gtsam::Values result = isam_.calculateEstimate();
    last_pose_ = result.at<gtsam::Pose3>(X(curr));
    last_vel_ = result.at<gtsam::Vector3>(V(curr));
    last_bias_ = result.at<gtsam::imuBias::ConstantBias>(B(curr));
    last_current_ = result.at<gtsam::Vector3>(C(curr));
    gtsam::Marginals marginals(isam_.getFactorsUnsafe(), result);
    gtsam::Matrix current_cov = marginals.marginalCovariance(C(curr));
    publishCurrent(stamp, last_current_, current_cov);

    // Sonar map/keyframe publishing (Task 5). Self-contained: only reads
    // curr_scan_points, last_pose_, keyframe_history_ -- safe regardless of
    // where later tasks insert their own post-estimate blocks.
    if (!curr_scan_points.empty()) {
      prev_keyframe_scan_ = curr_scan_points;
      keyframe_history_.emplace_back(last_pose_, curr_scan_points);
      publishMap(stamp, curr_scan_points, last_pose_);
    }
    publishKeyframes(stamp);

    graph_.resize(0);
    values_.clear();
    preint_->resetIntegrationAndSetBias(last_bias_);
    keyframe_index_ = curr;

    publishOdometry(stamp, last_pose_, last_vel_);
  }

  void publishOdometry(
    const rclcpp::Time & stamp, const gtsam::Pose3 & pose, const gtsam::Vector3 & vel)
  {
    nav_msgs::msg::Odometry msg;
    msg.header.stamp = stamp;
    msg.header.frame_id = "map";
    msg.child_frame_id = "bluerov2/base_link";
    const auto & t = pose.translation();
    msg.pose.pose.position.x = t.x();
    msg.pose.pose.position.y = t.y();
    msg.pose.pose.position.z = t.z();
    auto q = pose.rotation().toQuaternion();
    msg.pose.pose.orientation.w = q.w();
    msg.pose.pose.orientation.x = q.x();
    msg.pose.pose.orientation.y = q.y();
    msg.pose.pose.orientation.z = q.z();
    msg.twist.twist.linear.x = vel.x();
    msg.twist.twist.linear.y = vel.y();
    msg.twist.twist.linear.z = vel.z();
    odom_pub_->publish(msg);
  }

  void publishCurrent(
    const rclcpp::Time & stamp, const gtsam::Vector3 & current, const gtsam::Matrix & cov)
  {
    geometry_msgs::msg::TwistWithCovarianceStamped msg;
    msg.header.stamp = stamp;
    msg.header.frame_id = "map";
    msg.twist.twist.linear.x = current.x();
    msg.twist.twist.linear.y = current.y();
    msg.twist.twist.linear.z = current.z();
    for (int r = 0; r < 3; ++r) {
      for (int c = 0; c < 3; ++c) {
        msg.twist.covariance[r * 6 + c] = cov(r, c);
      }
    }
    current_pub_->publish(msg);
  }

  void publishMap(
    const rclcpp::Time & stamp,
    const std::vector<cavex_sic_slam::ScanPoint> & points,
    const gtsam::Pose3 & pose)
  {
    sensor_msgs::msg::PointCloud msg;
    msg.header.stamp = stamp;
    msg.header.frame_id = "map";
    for (const auto & p : points) {
      gtsam::Point3 world_pt = pose.transformFrom(gtsam::Point3(p.x, p.y, 0.0));
      geometry_msgs::msg::Point32 pt32;
      pt32.x = static_cast<float>(world_pt.x());
      pt32.y = static_cast<float>(world_pt.y());
      pt32.z = static_cast<float>(world_pt.z());
      msg.points.push_back(pt32);
    }
    map_pub_->publish(msg);
  }

  void publishKeyframes(const rclcpp::Time & stamp)
  {
    geometry_msgs::msg::PoseArray msg;
    msg.header.stamp = stamp;
    msg.header.frame_id = "map";
    for (const auto & [pose, scan] : keyframe_history_) {
      geometry_msgs::msg::Pose p;
      const auto & t = pose.translation();
      p.position.x = t.x(); p.position.y = t.y(); p.position.z = t.z();
      auto q = pose.rotation().toQuaternion();
      p.orientation.w = q.w(); p.orientation.x = q.x();
      p.orientation.y = q.y(); p.orientation.z = q.z();
      msg.poses.push_back(p);
    }
    keyframes_pub_->publish(msg);
  }

  std::mutex mutex_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr sonar_sub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud>::SharedPtr map_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr keyframes_pub_;
  std::array<rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr, 6> thruster_subs_;
  std::array<double, 6> thrust_n_;
  cavex_sic_slam::ThrusterGeometry thruster_geom_;
  cavex_sic_slam::DragCoefficients thruster_drag_;
  rclcpp::Publisher<geometry_msgs::msg::TwistWithCovarianceStamped>::SharedPtr current_pub_;

  sensor_msgs::msg::LaserScan::SharedPtr latest_scan_;
  std::vector<cavex_sic_slam::ScanPoint> prev_keyframe_scan_;
  std::vector<std::pair<gtsam::Pose3, std::vector<cavex_sic_slam::ScanPoint>>> keyframe_history_;

  boost::shared_ptr<gtsam::PreintegratedCombinedMeasurements::Params> imu_params_;
  std::shared_ptr<gtsam::PreintegratedCombinedMeasurements> preint_;
  gtsam::ISAM2 isam_;
  gtsam::NonlinearFactorGraph graph_;
  gtsam::Values values_;

  std::size_t keyframe_index_;
  double keyframe_period_s_;
  double keyframe_distance_m_;
  double keyframe_rotation_rad_;
  double accumulated_s_ = 0.0;
  bool have_prev_imu_time_;
  rclcpp::Time prev_imu_time_;

  gtsam::Pose3 last_pose_;
  gtsam::Vector3 last_vel_;
  gtsam::imuBias::ConstantBias last_bias_;
  gtsam::Vector3 last_current_;
};

}  // namespace cavex_sic_slam

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<cavex_sic_slam::SicSlamNode>());
  rclcpp::shutdown();
  return 0;
}
