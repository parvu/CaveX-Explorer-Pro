#include <memory>
#include <mutex>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <nav_msgs/msg/odometry.hpp>

#include <gtsam/navigation/CombinedImuFactor.h>
#include <gtsam/nonlinear/ISAM2.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/geometry/Pose3.h>

using gtsam::symbol_shorthand::X;
using gtsam::symbol_shorthand::V;
using gtsam::symbol_shorthand::B;

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

    isam_.update(graph_, values_);
    graph_.resize(0);
    values_.clear();
    last_pose_ = prior_pose;
    last_vel_ = prior_vel;
    last_bias_ = prior_bias;

    imu_sub_ = this->create_subscription<sensor_msgs::msg::Imu>(
      "/bluerov2/imu", rclcpp::SensorDataQoS(),
      std::bind(&SicSlamNode::imuCallback, this, std::placeholders::_1));
    odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>("/sic_slam/odometry", 10);

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

    isam_.update(graph_, values_);
    gtsam::Values result = isam_.calculateEstimate();
    last_pose_ = result.at<gtsam::Pose3>(X(curr));
    last_vel_ = result.at<gtsam::Vector3>(V(curr));
    last_bias_ = result.at<gtsam::imuBias::ConstantBias>(B(curr));

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

  std::mutex mutex_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;

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
};

}  // namespace cavex_sic_slam

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<cavex_sic_slam::SicSlamNode>());
  rclcpp::shutdown();
  return 0;
}
