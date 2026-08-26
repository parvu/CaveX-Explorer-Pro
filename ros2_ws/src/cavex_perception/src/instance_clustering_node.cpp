#include <cstring>
#include <iostream>
#include <memory>
#include <mutex>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/image_encodings.hpp>
#include <vision_msgs/msg/detection3_d_array.hpp>
#include <cv_bridge/cv_bridge.hpp>
#include <image_geometry/pinhole_camera_model.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_eigen/tf2_eigen.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/passthrough.h>

#include "instance_clustering.hpp"

using cavex_perception::Instance;
using cavex_perception::clusterPoints;
using cavex_perception::clustersToInstances;
using cavex_perception::matchAndAssignIds;

namespace
{

pcl::PointCloud<pcl::PointXYZRGB>::Ptr makeBlob(
  float cx, float cy, float cz, int n_points, float spread)
{
  auto cloud = pcl::make_shared<pcl::PointCloud<pcl::PointXYZRGB>>();
  for (int i = 0; i < n_points; ++i) {
    pcl::PointXYZRGB p;
    float t = static_cast<float>(i) / static_cast<float>(n_points);
    p.x = cx + spread * (t - 0.5f);
    p.y = cy + spread * (t - 0.5f);
    p.z = cz;
    p.r = p.g = p.b = 200;
    cloud->push_back(p);
  }
  return cloud;
}

void selfCheck()
{
  // Two well-separated blobs (5m apart) should cluster into 2 instances.
  auto two_blobs = pcl::make_shared<pcl::PointCloud<pcl::PointXYZRGB>>();
  *two_blobs += *makeBlob(0.0f, 0.0f, 0.0f, 20, 0.3f);
  *two_blobs += *makeBlob(5.0f, 0.0f, 0.0f, 20, 0.3f);
  auto clusters = clusterPoints(two_blobs, 0.5, 5, 1000);
  if (clusters.size() != 2) {
    std::cerr << "FAIL: expected 2 clusters for two well-separated blobs, got "
              << clusters.size() << "\n";
    std::exit(1);
  }

  // A single merged blob (points spread continuously, gaps < tolerance)
  // should cluster into exactly 1 instance.
  auto merged = makeBlob(0.0f, 0.0f, 0.0f, 50, 2.0f);
  auto merged_clusters = clusterPoints(merged, 0.5, 5, 1000);
  if (merged_clusters.size() != 1) {
    std::cerr << "FAIL: expected 1 cluster for a continuous blob, got "
              << merged_clusters.size() << "\n";
    std::exit(1);
  }

  // Centroid of a symmetric blob at (5,0,0) should land near (5,0,0).
  auto instances = clustersToInstances(*two_blobs, clusters);
  bool found_near_five = false;
  for (const auto & inst : instances) {
    if (std::abs(inst.centroid.x() - 5.0f) < 0.3f) {
      found_near_five = true;
    }
  }
  if (!found_near_five) {
    std::cerr << "FAIL: expected one cluster centroid near x=5.0\n";
    std::exit(1);
  }

  // ID tracking: a repeat frame with a small centroid shift (0.1m, well
  // within match_distance_m=1.0) should keep the same ids; a frame with
  // only one of the two blobs should keep that one's id and not reuse the
  // other's id for a new, unrelated instance.
  int next_id = 0;
  auto frame1 = matchAndAssignIds(instances, {}, next_id, 1.0);
  if (frame1.size() != 2 || frame1[0].id == frame1[1].id) {
    std::cerr << "FAIL: expected 2 distinct fresh ids on first frame\n";
    std::exit(1);
  }

  std::vector<Instance> shifted = frame1;
  shifted[0].centroid.x() += 0.1f;
  shifted[1].centroid.x() += 0.1f;
  auto frame2 = matchAndAssignIds(shifted, frame1, next_id, 1.0);
  if (frame2[0].id != frame1[0].id || frame2[1].id != frame1[1].id) {
    std::cerr << "FAIL: expected ids to persist across a small centroid shift\n";
    std::exit(1);
  }

  std::vector<Instance> only_second = {frame1[1]};
  only_second[0].centroid.x() += 0.1f;
  auto frame3 = matchAndAssignIds(only_second, frame1, next_id, 1.0);
  if (frame3[0].id != frame1[1].id) {
    std::cerr << "FAIL: expected the surviving instance to keep its id\n";
    std::exit(1);
  }

  std::cout << "instance_clustering_node self-check: OK\n";
}

}  // namespace

namespace cavex_perception
{

class InstanceClusteringNode : public rclcpp::Node
{
public:
  InstanceClusteringNode()
  : Node("instance_clustering_node"), next_id_(0)
  {
    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    color_sub_ = create_subscription<sensor_msgs::msg::Image>(
      "/camera/color/image_raw", 10,
      [this](sensor_msgs::msg::Image::ConstSharedPtr msg) {
        std::lock_guard<std::mutex> lock(color_mutex_);
        latest_color_ = msg;
      });
    info_sub_ = create_subscription<sensor_msgs::msg::CameraInfo>(
      "/camera/color/camera_info", 10,
      [this](sensor_msgs::msg::CameraInfo::ConstSharedPtr msg) {
        std::lock_guard<std::mutex> lock(color_mutex_);
        latest_info_ = msg;
      });
    lidar_sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
      "/lidar/points", rclcpp::SensorDataQoS(),
      std::bind(&InstanceClusteringNode::lidarCallback, this, std::placeholders::_1));

    instances_pub_ = create_publisher<vision_msgs::msg::Detection3DArray>(
      "/instance_clustering/instances", 10);
    colored_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>(
      "/instance_clustering/colored_points", 10);

    RCLCPP_INFO(get_logger(), "instance_clustering_node ready: colorizing lidar via camera "
      "projection, clustering into instances, publishing /instance_clustering/instances "
      "and /instance_clustering/colored_points.");
  }

private:
  void lidarCallback(sensor_msgs::msg::PointCloud2::ConstSharedPtr lidar_msg)
  {
    sensor_msgs::msg::Image::ConstSharedPtr color;
    sensor_msgs::msg::CameraInfo::ConstSharedPtr info;
    {
      std::lock_guard<std::mutex> lock(color_mutex_);
      color = latest_color_;
      info = latest_info_;
    }
    if (!color || !info) {
      return;  // no color frame cached yet -- skip this lidar frame
    }

    // Transform: lidar frame -> camera optical frame, for projection.
    geometry_msgs::msg::TransformStamped lidar_to_cam;
    try {
      lidar_to_cam = tf_buffer_->lookupTransform(
        color->header.frame_id, lidar_msg->header.frame_id,
        tf2::TimePointZero);
    } catch (const tf2::TransformException & ex) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
        "TF lookup %s -> %s failed: %s", lidar_msg->header.frame_id.c_str(),
        color->header.frame_id.c_str(), ex.what());
      return;
    }
    // Transform: lidar frame -> map, so published instances/cloud are in
    // the same frame dead_end_backtrack_node's pose/costmap use.
    geometry_msgs::msg::TransformStamped lidar_to_map;
    try {
      lidar_to_map = tf_buffer_->lookupTransform(
        "map", lidar_msg->header.frame_id, tf2::TimePointZero);
    } catch (const tf2::TransformException & ex) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
        "TF lookup %s -> map failed: %s", lidar_msg->header.frame_id.c_str(),
        ex.what());
      return;
    }

    cv_bridge::CvImageConstPtr cv_color;
    try {
      cv_color = cv_bridge::toCvShare(color, sensor_msgs::image_encodings::BGR8);
    } catch (const cv_bridge::Exception & ex) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
        "cv_bridge conversion failed: %s", ex.what());
      return;
    }

    image_geometry::PinholeCameraModel cam_model;
    cam_model.fromCameraInfo(*info);

    auto raw_cloud_unfiltered = pcl::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
    pcl::fromROSMsg(*lidar_msg, *raw_cloud_unfiltered);

    // Important 4 (final whole-branch review): a continuous cave surface
    // (floor+walls+ceiling all connected) forms one giant cluster with no
    // pre-filtering. Crop to a z-band around the lidar's own origin
    // (lidar frame, sensor-relative z) to exclude most floor/ceiling
    // returns while keeping wall/obstacle returns at the vehicle's own
    // level -- a reasonable default band for this ground vehicle's
    // ~0.3m footprint scale (see cluster_tolerance_m_'s own comment).
    pcl::PointCloud<pcl::PointXYZ> raw_cloud;
    pcl::PassThrough<pcl::PointXYZ> z_filter;
    z_filter.setInputCloud(raw_cloud_unfiltered);
    z_filter.setFilterFieldName("z");
    z_filter.setFilterLimits(-1.0, 1.0);
    z_filter.filter(raw_cloud);

    Eigen::Isometry3d lidar_to_cam_eigen = tf2::transformToEigen(lidar_to_cam);
    Eigen::Isometry3d lidar_to_map_eigen = tf2::transformToEigen(lidar_to_map);

    auto colored = pcl::make_shared<pcl::PointCloud<pcl::PointXYZRGB>>();
    colored->reserve(raw_cloud.size());
    for (const auto & p : raw_cloud.points) {
      if (!std::isfinite(p.x) || !std::isfinite(p.y) || !std::isfinite(p.z)) {
        continue;
      }
      Eigen::Vector3d p_lidar(p.x, p.y, p.z);
      Eigen::Vector3d p_cam = lidar_to_cam_eigen * p_lidar;

      pcl::PointXYZRGB cp;
      cp.r = cp.g = cp.b = 128;  // default gray, overwritten below if in view
      if (p_cam.z() > 0.0) {
        cv::Point2d uv = cam_model.project3dToPixel(
          cv::Point3d(p_cam.x(), p_cam.y(), p_cam.z()));
        int u = static_cast<int>(uv.x);
        int v = static_cast<int>(uv.y);
        if (u >= 0 && u < cv_color->image.cols && v >= 0 && v < cv_color->image.rows) {
          cv::Vec3b bgr = cv_color->image.at<cv::Vec3b>(v, u);
          cp.b = bgr[0];
          cp.g = bgr[1];
          cp.r = bgr[2];
        }
      }

      Eigen::Vector3d p_map = lidar_to_map_eigen * p_lidar;
      cp.x = static_cast<float>(p_map.x());
      cp.y = static_cast<float>(p_map.y());
      cp.z = static_cast<float>(p_map.z());
      colored->push_back(cp);
    }

    auto clusters = clusterPoints(colored, cluster_tolerance_m_, min_cluster_size_, max_cluster_size_);
    auto raw_instances = clustersToInstances(*colored, clusters);
    auto instances = matchAndAssignIds(raw_instances, prev_instances_, next_id_, match_distance_m_);
    prev_instances_ = instances;

    publishInstances(instances, lidar_msg->header.stamp);
    publishColoredCloud(*colored, lidar_msg->header.stamp);
  }

  void publishInstances(
    const std::vector<Instance> & instances, const builtin_interfaces::msg::Time & stamp)
  {
    vision_msgs::msg::Detection3DArray msg;
    msg.header.frame_id = "map";
    msg.header.stamp = stamp;
    for (const auto & inst : instances) {
      vision_msgs::msg::Detection3D det;
      det.header = msg.header;
      // Spec (design doc §3): persistent ID goes in
      // results[0].hypothesis.class_id, not det.id.
      vision_msgs::msg::ObjectHypothesisWithPose hyp;
      hyp.hypothesis.class_id = std::to_string(inst.id);
      hyp.hypothesis.score = 1.0;  // geometric clustering has no real confidence to report
      det.results.push_back(hyp);
      det.bbox.center.position.x = inst.centroid.x();
      det.bbox.center.position.y = inst.centroid.y();
      det.bbox.center.position.z = inst.centroid.z();
      det.bbox.center.orientation.w = 1.0;
      det.bbox.size.x = inst.size.x();
      det.bbox.size.y = inst.size.y();
      det.bbox.size.z = inst.size.z();
      msg.detections.push_back(det);
    }
    instances_pub_->publish(msg);
  }

  void publishColoredCloud(
    const pcl::PointCloud<pcl::PointXYZRGB> & cloud, const builtin_interfaces::msg::Time & stamp)
  {
    sensor_msgs::msg::PointCloud2 msg;
    pcl::toROSMsg(cloud, msg);
    msg.header.frame_id = "map";
    msg.header.stamp = stamp;
    colored_pub_->publish(msg);
  }

  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr color_sub_;
  rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr info_sub_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr lidar_sub_;
  rclcpp::Publisher<vision_msgs::msg::Detection3DArray>::SharedPtr instances_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr colored_pub_;

  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  std::mutex color_mutex_;
  sensor_msgs::msg::Image::ConstSharedPtr latest_color_;
  sensor_msgs::msg::CameraInfo::ConstSharedPtr latest_info_;

  std::vector<Instance> prev_instances_;
  int next_id_;

  // Matches dead_end_backtrack_node's OPENING_SCAN_RADIUS_M (2.4m) scale --
  // clusters within 0.5m of each other merge into one instance, a plausible
  // single real object/wall-feature at this vehicle's ~0.3m footprint scale.
  double cluster_tolerance_m_ = 0.5;
  int min_cluster_size_ = 10;
  int max_cluster_size_ = 25000;
  double match_distance_m_ = 1.0;
};

}  // namespace cavex_perception

int main(int argc, char ** argv)
{
  for (int i = 1; i < argc; ++i) {
    if (std::strcmp(argv[i], "--self-check") == 0) {
      selfCheck();
      return 0;
    }
  }
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<cavex_perception::InstanceClusteringNode>());
  rclcpp::shutdown();
  return 0;
}
