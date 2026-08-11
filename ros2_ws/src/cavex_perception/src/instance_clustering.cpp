#include "instance_clustering.hpp"

#include <limits>

#include <pcl/segmentation/extract_clusters.h>
#include <pcl/search/kdtree.h>

namespace cavex_perception
{

std::vector<pcl::PointIndices> clusterPoints(
  const pcl::PointCloud<pcl::PointXYZRGB>::Ptr & cloud,
  double tolerance_m, int min_size, int max_size)
{
  std::vector<pcl::PointIndices> clusters;
  if (cloud->empty()) {
    return clusters;
  }
  auto tree = pcl::make_shared<pcl::search::KdTree<pcl::PointXYZRGB>>();
  tree->setInputCloud(cloud);

  pcl::EuclideanClusterExtraction<pcl::PointXYZRGB> ec;
  ec.setClusterTolerance(tolerance_m);
  ec.setMinClusterSize(min_size);
  ec.setMaxClusterSize(max_size);
  ec.setSearchMethod(tree);
  ec.setInputCloud(cloud);
  ec.extract(clusters);
  return clusters;
}

std::vector<Instance> clustersToInstances(
  const pcl::PointCloud<pcl::PointXYZRGB> & cloud,
  const std::vector<pcl::PointIndices> & clusters)
{
  std::vector<Instance> instances;
  instances.reserve(clusters.size());
  for (const auto & cluster : clusters) {
    Eigen::Vector3f min_pt(
      std::numeric_limits<float>::max(),
      std::numeric_limits<float>::max(),
      std::numeric_limits<float>::max());
    Eigen::Vector3f max_pt(
      std::numeric_limits<float>::lowest(),
      std::numeric_limits<float>::lowest(),
      std::numeric_limits<float>::lowest());
    Eigen::Vector3f sum = Eigen::Vector3f::Zero();

    for (int idx : cluster.indices) {
      const auto & p = cloud.points[idx];
      Eigen::Vector3f v(p.x, p.y, p.z);
      sum += v;
      min_pt = min_pt.cwiseMin(v);
      max_pt = max_pt.cwiseMax(v);
    }

    Instance inst;
    inst.id = -1;
    inst.centroid = sum / static_cast<float>(cluster.indices.size());
    inst.size = max_pt - min_pt;
    instances.push_back(inst);
  }
  return instances;
}

std::vector<Instance> matchAndAssignIds(
  const std::vector<Instance> & new_instances,
  const std::vector<Instance> & prev_instances,
  int & next_id, double match_distance_m)
{
  std::vector<Instance> result = new_instances;
  std::vector<bool> prev_used(prev_instances.size(), false);

  for (auto & inst : result) {
    int best_idx = -1;
    float best_dist = static_cast<float>(match_distance_m);
    for (size_t j = 0; j < prev_instances.size(); ++j) {
      if (prev_used[j]) {
        continue;
      }
      float dist = (inst.centroid - prev_instances[j].centroid).norm();
      if (dist <= best_dist) {
        best_dist = dist;
        best_idx = static_cast<int>(j);
      }
    }
    if (best_idx >= 0) {
      inst.id = prev_instances[best_idx].id;
      prev_used[best_idx] = true;
    } else {
      inst.id = next_id++;
    }
  }
  return result;
}

}  // namespace cavex_perception
