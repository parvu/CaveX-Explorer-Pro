#include <cstring>
#include <iostream>

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

  std::cout << "sic_slam_node self-check: OK\n";
}

}  // namespace

int main(int argc, char ** argv)
{
  for (int i = 1; i < argc; ++i) {
    if (std::strcmp(argv[i], "--self-check") == 0) {
      selfCheck();
      return 0;
    }
  }
  std::cerr << "sic_slam_node: full ROS node not yet implemented "
               "(see Task 4). Run with --self-check for now.\n";
  return 1;
}
