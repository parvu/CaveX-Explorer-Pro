#ifndef CAVEX_PERCEPTION__INSTANCE_CLUSTERING_HPP_
#define CAVEX_PERCEPTION__INSTANCE_CLUSTERING_HPP_

#include <vector>

#include <Eigen/Core>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/PointIndices.h>

namespace cavex_perception
{

struct Instance
{
  int id;
  Eigen::Vector3f centroid;
  Eigen::Vector3f size;
};

// Segments `cloud` into geometric clusters. Points closer together than
// tolerance_m belong to the same cluster; clusters outside [min_size,
// max_size] points are discarded as noise/too-large-to-be-an-instance.
std::vector<pcl::PointIndices> clusterPoints(
  const pcl::PointCloud<pcl::PointXYZRGB>::Ptr & cloud,
  double tolerance_m, int min_size, int max_size);

// Computes the centroid and axis-aligned bounding-box size of each cluster.
// Returned instances have id = -1 (unassigned) -- matchAndAssignIds fills
// in real ids.
std::vector<Instance> clustersToInstances(
  const pcl::PointCloud<pcl::PointXYZRGB> & cloud,
  const std::vector<pcl::PointIndices> & clusters);

// Greedily matches each of new_instances to the closest prev_instances
// centroid within match_distance_m (each previous instance can match at
// most once). Matched instances keep the previous id; unmatched new
// instances get a fresh id from next_id (which is incremented). Returns
// new_instances with ids assigned, in the same order as the input.
std::vector<Instance> matchAndAssignIds(
  const std::vector<Instance> & new_instances,
  const std::vector<Instance> & prev_instances,
  int & next_id, double match_distance_m);

}  // namespace cavex_perception

#endif  // CAVEX_PERCEPTION__INSTANCE_CLUSTERING_HPP_
