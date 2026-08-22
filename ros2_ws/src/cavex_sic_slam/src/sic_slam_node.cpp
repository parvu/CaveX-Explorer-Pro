#include <limits>
#include <memory>
#include <mutex>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <sensor_msgs/msg/point_cloud.hpp>
#include <geometry_msgs/msg/pose_array.hpp>
#include <geometry_msgs/msg/point32.hpp>
#include <geometry_msgs/msg/twist_with_covariance_stamped.hpp>
#include <geometry_msgs/msg/vector3_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <std_msgs/msg/float64.hpp>

#include <gtsam/navigation/CombinedImuFactor.h>
#include <gtsam/nonlinear/ISAM2.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/nonlinear/Marginals.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/linear/LossFunctions.h>

#include "cavex_sic_slam/scan_registration.hpp"
#include "cavex_sic_slam/current_factor.hpp"
#include "cavex_sic_slam/continuous_current_estimator.hpp"

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
    // Ablation switch: false runs the identical IMU+sonar iSAM2 graph with
    // no C(i) variable, no CurrentFactor, no current random walk at all --
    // not "CurrentFactor present but unconstrained". Same pipeline, same
    // keyframe triggers, same live-sim/ATE methodology; only the current
    // observability mechanism is removed, for an apples-to-apples baseline.
    enable_current_factor_ = this->declare_parameter<bool>("enable_current_factor", true);
    // Additive continuous-time smoothing of the existing discrete C(i)
    // estimates -- see .superpowers/plans/2026-08-23-continuous-current-
    // field-integration.md. Does NOT touch pose (X/V) estimation and does
    // NOT independently re-derive current; only meaningful when
    // enable_current_factor_ is also true. Default off.
    enable_continuous_current_field_ = this->declare_parameter<bool>(
      "enable_continuous_current_field", false);
    // Dynamic Covariance Scaling (real request, 2026-08-22): tests whether
    // a proper robust M-estimator on the actual outlier-prone geometric
    // constraints (sonar odometry + loop-closure BetweenFactors -- a bad
    // scan match is a real outlier, unlike CurrentFactor's role which is
    // absorbing a smoothly-varying disturbance, not rejecting bad
    // measurements) achieves comparable graph stability to CurrentFactor,
    // which would make CurrentFactor's incidental stabilizing effect
    // (found earlier this session: it reduces IndeterminantLinearSystem-
    // Exception crash RATE) redundant with a more targeted fix. Default
    // off -- preserves existing behavior exactly.
    enable_dcs_robust_ = this->declare_parameter<bool>("enable_dcs_robust", false);
    dcs_c_ = this->declare_parameter<double>("dcs_c", 1.0);

    auto imu_params = gtsam::PreintegratedCombinedMeasurements::Params::MakeSharedU(9.81);
    // Roll-180 mount into the ArduPilot body frame: accel/gyro measurements
    // must be rotated into the graph's body frame before preintegration.
    imu_params->setBodyPSensor(gtsam::Pose3(
        gtsam::Rot3::Rx(M_PI), gtsam::Point3(0, 0, 0)));
    imu_params_ = imu_params;
    gtsam::imuBias::ConstantBias zero_bias;
    preint_ = std::make_shared<gtsam::PreintegratedCombinedMeasurements>(
      imu_params_, zero_bias);

    // This was a fully-default ISAM2Params() -- relinearizeSkip defaults to
    // 10 (only every 10th update() call actually relinearizes) and
    // relinearizeThreshold defaults to 0.1. Tightened while chasing a
    // chaotic pose-estimate oscillation under real current (2026-08-22);
    // that oscillation's actual root cause turned out to be an upstream
    // test-harness bug (zero real thrust signal reaching CurrentFactor),
    // not stale ISAM2 linearization, so this alone did not fix it either.
    // Left in regardless -- forcing full relinearization every update is a
    // real robustness improvement on its own merits (trades more CPU per
    // update for never letting a variable go stale), independent of that
    // investigation.
    gtsam::ISAM2Params isam_params;
    isam_params.relinearizeThreshold = 0.01;
    isam_params.relinearizeSkip = 1;
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
    if (enable_current_factor_) {
      values_.insert(C(0), prior_current);
      // Loose: 0.05 actively fought convergence to a real nonzero current --
      // it should say "no information yet", not "confidently zero". See the
      // matching current_walk_noise change below for the full reasoning.
      auto current_noise = gtsam::noiseModel::Isotropic::Sigma(3, 0.5);
      graph_.addPrior(C(0), prior_current, current_noise);
    }

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
    continuous_current_pub_ = this->create_publisher<geometry_msgs::msg::Vector3Stamped>(
      "/sic_slam/current_estimate_continuous", 10);

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
      "sic_slam_node ready: /bluerov2/imu + /bluerov2/sonar -> /sic_slam/odometry (%s).",
      enable_current_factor_ ? "IMU+sonar+CurrentFactor" : "IMU+sonar only, no CurrentFactor");
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

  // Dynamic Covariance Scaling (real request, 2026-08-22): wraps a base
  // noise model in GTSAM's built-in DCS M-estimator (Agarwal et al.,
  // "Robust Map Optimization using Dynamic Covariance Scaling") when
  // enabled, otherwise returns it unchanged. Applied only to the
  // sonar/loop-closure BetweenFactors -- the actual outlier-prone
  // geometric constraints (a bad scan match is a real measurement
  // outlier) -- not to CombinedImuFactor or CurrentFactor, whose role is
  // absorbing a smoothly-varying quantity, not rejecting bad data.
  gtsam::SharedNoiseModel wrapDcsIfEnabled(const gtsam::SharedNoiseModel & base) const
  {
    if (!enable_dcs_robust_) {
      return base;
    }
    return gtsam::noiseModel::Robust::Create(
      gtsam::noiseModel::mEstimator::DCS::Create(dcs_c_), base);
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

    // TEMP diagnostic, 2026-08-22: investigating a recurring
    // gtsam::IndeterminantLinearSystemException. Real hypothesis: when a
    // keyframe's sonar scan has zero valid returns (real range/detection
    // limits, not just turbidity), or registration fails to converge, the
    // whole sonar-constraint block below is skipped and X(curr)/V(curr)
    // are constrained by IMU (+CurrentFactor) alone -- if that happens
    // for several consecutive keyframes, the chain could become
    // ill-conditioned. Remove once confirmed/fixed.
    RCLCPP_INFO(
      this->get_logger(),
      "kf %zu: curr_scan=%zu prev_scan=%zu", curr, curr_scan_points.size(),
      prev_keyframe_scan_.size());
    if (!curr_scan_points.empty() && !prev_keyframe_scan_.empty()) {
      auto reg = cavex_sic_slam::registerScans(
        curr_scan_points, prev_keyframe_scan_, 0.5, 20);
      RCLCPP_INFO(
        this->get_logger(), "kf %zu: reg.converged=%d matched_fraction=%.3f",
        curr, reg.converged, reg.matched_fraction);
      if (reg.converged) {
        gtsam::Pose3 odom_delta(
          gtsam::Rot3::Yaw(reg.dyaw), gtsam::Point3(reg.dx, reg.dy, 0.0));
        // roll,pitch,yaw sigma (rad), then x,y,z sigma (m) -- z loose,
        // sonar can't observe depth.
        gtsam::SharedNoiseModel sonar_noise = gtsam::noiseModel::Diagonal::Sigmas(
          (gtsam::Vector(6) << 0.3, 0.3, 0.1, 0.1, 0.1, 0.5).finished());
        graph_.add(gtsam::BetweenFactor<gtsam::Pose3>(
          X(prev), X(curr), odom_delta, wrapDcsIfEnabled(sonar_noise)));
      }

      // Loop closure: try registering against earlier, non-adjacent
      // keyframes. Gate on match quality so a bad match never enters the
      // graph as a confident constraint.
      for (const auto & [hist_key, hist_pose, hist_scan] : keyframe_history_) {
        if (hist_key == prev) {
          // Already covered by the odometry BetweenFactor above -- the
          // most recent history entry is the same scan pair as (prev, curr).
          continue;
        }
        auto loop_reg = cavex_sic_slam::registerScans(
          curr_scan_points, hist_scan, 0.5, 20);
        if (loop_reg.converged && loop_reg.matched_fraction > 0.7 &&
          loop_reg.mean_residual < 0.15)
        {
          gtsam::Pose3 loop_delta(
            gtsam::Rot3::Yaw(loop_reg.dyaw), gtsam::Point3(loop_reg.dx, loop_reg.dy, 0.0));
          gtsam::SharedNoiseModel loop_noise = gtsam::noiseModel::Diagonal::Sigmas(
            (gtsam::Vector(6) << 0.3, 0.3, 0.15, 0.15, 0.15, 0.6).finished());
          graph_.add(gtsam::BetweenFactor<gtsam::Pose3>(
            X(hist_key), X(curr), loop_delta, wrapDcsIfEnabled(loop_noise)));
        }
      }
    }

    if (enable_current_factor_) {
      std::array<double, 6> thrust_snapshot = thrust_n_;
      auto current_factor_noise = gtsam::noiseModel::Isotropic::Sigma(3, 0.15);
      graph_.add(cavex_sic_slam::CurrentFactor(
        X(curr), V(curr), C(curr), thrust_snapshot, thruster_geom_, thruster_drag_,
        current_factor_noise));

      // Briefly tried tightening this 0.2 -> 0.02 while chasing a chaotic
      // pose-estimate oscillation under real current (2026-08-22) --
      // reverted back to 0.2. Root cause of that oscillation turned out to
      // be upstream: the test harness driving the vehicle via
      // ApplyLinkWrench never fed real thrust into `thrust_n_`, so
      // CurrentFactor's `v_pred_` was zero for the whole test, making its
      // residual the maximally-degenerate `V - C` -- no noise-sigma choice
      // fixes an upstream zero-signal input. The original 0.2 rationale
      // (see above: 0.02 needs ~5600 keyframes to track a real changing
      // current) is still valid and untouched by this.
      auto current_walk_noise = gtsam::noiseModel::Isotropic::Sigma(3, 0.2);
      graph_.add(gtsam::BetweenFactor<gtsam::Vector3>(
        C(prev), C(curr), gtsam::Vector3::Zero(), current_walk_noise));

      values_.insert(C(curr), last_current_);
    }

    isam_.update(graph_, values_);
    gtsam::Values result = isam_.calculateEstimate();
    last_pose_ = result.at<gtsam::Pose3>(X(curr));
    last_vel_ = result.at<gtsam::Vector3>(V(curr));
    last_bias_ = result.at<gtsam::imuBias::ConstantBias>(B(curr));

    // TEMP diagnostic, 2026-08-22: sonar registration health was ruled
    // out (matched_fraction stayed ~1.0 right through observed
    // divergence events, both in a 1000+-keyframe run and a
    // ~10-15-keyframe-old fresh graph). Real next hypothesis: the graph
    // is briefly near-singular, catchable via the condition number
    // (max/min eigenvalue) of X(curr)/V(curr)'s marginal covariance --
    // a large, spiking condition number right before a divergence event
    // would confirm this. Computed unconditionally (not just when
    // CurrentFactor is on) so both ablation legs get the same diagnostic.
    // Remove once confirmed/fixed -- Marginals computation is real cost
    // added to every keyframe.
    {
      gtsam::Marginals diag_marginals(isam_.getFactorsUnsafe(), result);
      auto log_condition = [&](const char * name, const gtsam::Matrix & cov) {
          Eigen::SelfAdjointEigenSolver<gtsam::Matrix> es(cov);
          double min_eig = es.eigenvalues().minCoeff();
          double max_eig = es.eigenvalues().maxCoeff();
          double cond = (std::abs(min_eig) > 1e-15) ? max_eig / min_eig :
            std::numeric_limits<double>::infinity();
          RCLCPP_INFO(
            this->get_logger(), "kf %zu: %s cond=%.3e min_eig=%.3e max_eig=%.3e",
            curr, name, cond, min_eig, max_eig);
        };
      log_condition("X", diag_marginals.marginalCovariance(X(curr)));
      log_condition("V", diag_marginals.marginalCovariance(V(curr)));
      if (enable_current_factor_) {
        log_condition("C", diag_marginals.marginalCovariance(C(curr)));
      }
    }

    if (enable_current_factor_) {
      last_current_ = result.at<gtsam::Vector3>(C(curr));
      gtsam::Marginals marginals(isam_.getFactorsUnsafe(), result);
      gtsam::Matrix current_cov = marginals.marginalCovariance(C(curr));
      publishCurrent(stamp, last_current_, current_cov);
    }

    if (enable_current_factor_ && enable_continuous_current_field_) {
      double t_seconds = rclcpp::Time(stamp).seconds();
      continuous_current_estimator_.addSample(t_seconds, last_current_);
      if (++continuous_current_refit_counter_ % 5 == 0) {
        continuous_current_estimator_.refit();
      }
      auto smoothed = continuous_current_estimator_.evaluate(t_seconds);
      if (smoothed) {
        geometry_msgs::msg::Vector3Stamped msg;
        msg.header.stamp = stamp;
        msg.header.frame_id = "map";
        msg.vector.x = (*smoothed)(0);
        msg.vector.y = (*smoothed)(1);
        msg.vector.z = (*smoothed)(2);
        continuous_current_pub_->publish(msg);
      }
    }

    // Sonar map/keyframe publishing (Task 5). Self-contained: only reads
    // curr_scan_points, last_pose_, keyframe_history_ -- safe regardless of
    // where later tasks insert their own post-estimate blocks.
    if (!curr_scan_points.empty()) {
      prev_keyframe_scan_ = curr_scan_points;
      keyframe_history_.emplace_back(curr, last_pose_, curr_scan_points);
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
    // nav_msgs/Odometry requires twist to be expressed in child_frame_id
    // (the body frame), but `vel` is the graph's map/world-frame V(i).
    gtsam::Vector3 body_vel = pose.rotation().unrotate(vel);
    msg.twist.twist.linear.x = body_vel.x();
    msg.twist.twist.linear.y = body_vel.y();
    msg.twist.twist.linear.z = body_vel.z();
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
    for (const auto & [key, pose, scan] : keyframe_history_) {
      (void)key;
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
  rclcpp::Publisher<geometry_msgs::msg::Vector3Stamped>::SharedPtr continuous_current_pub_;

  sensor_msgs::msg::LaserScan::SharedPtr latest_scan_;
  std::vector<cavex_sic_slam::ScanPoint> prev_keyframe_scan_;
  std::vector<std::tuple<std::size_t, gtsam::Pose3,
    std::vector<cavex_sic_slam::ScanPoint>>> keyframe_history_;

  boost::shared_ptr<gtsam::PreintegratedCombinedMeasurements::Params> imu_params_;
  std::shared_ptr<gtsam::PreintegratedCombinedMeasurements> preint_;
  gtsam::ISAM2 isam_;
  gtsam::NonlinearFactorGraph graph_;
  gtsam::Values values_;

  std::size_t keyframe_index_;
  bool enable_current_factor_;
  bool enable_continuous_current_field_;
  cavex_sic_slam::ContinuousCurrentEstimator continuous_current_estimator_{90.0, 6};
  size_t continuous_current_refit_counter_ = 0;
  bool enable_dcs_robust_;
  double dcs_c_;
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
