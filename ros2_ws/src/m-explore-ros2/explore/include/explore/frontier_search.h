#ifndef FRONTIER_SEARCH_H_
#define FRONTIER_SEARCH_H_

#include "nav2_costmap_2d/costmap_2d_ros.hpp"

namespace frontier_exploration
{
/**
 * @brief Represents a frontier
 *
 */
struct Frontier {
  std::uint32_t size;
  double min_distance;
  double cost;
  geometry_msgs::msg::Point initial;
  geometry_msgs::msg::Point centroid;
  geometry_msgs::msg::Point middle;
  // Real, live-diagnosed problem (2026-08-27): explore.cpp used to navigate to
  // `centroid` (a plain arithmetic average of every member cell) -- fine for a
  // roughly convex blob, but a naive average of an irregularly-shaped or
  // wall-hugging frontier can land in unknown/inflated space with no real,
  // reachable cell nearby, which Nav2's planner then can't terminate a plan
  // at ("Failed to create plan with tolerance..."), confirmed live,
  // repeatedly, across several different real frontiers. `target` is the
  // closest ACTUAL frontier cell (guaranteed real, adjacent to real free
  // space, by definition of being a frontier cell at all) that is also at
  // least robot_exclusion_radius_ away from the robot -- combines the fix
  // for that bug with the earlier robot-adjacency fix (see this struct's
  // `centroid` field's own history in frontier_search.cpp) in one target
  // point. This is what explore.cpp actually navigates to now.
  geometry_msgs::msg::Point target;
  std::vector<geometry_msgs::msg::Point> points;
};

/**
 * @brief Thread-safe implementation of a frontier-search task for an input
 * costmap.
 */
class FrontierSearch
{
public:
  FrontierSearch() : logger_(rclcpp::get_logger("frontier_search")) {} // Default constructor for the logger

  /**
   * @brief Constructor for search task
   * @param costmap Reference to costmap data to search.
   */
  FrontierSearch(nav2_costmap_2d::Costmap2D* costmap, double potential_scale,
                 double gain_scale, double min_frontier_size, rclcpp::Logger logger,
                 double robot_exclusion_radius = 0.0);

  /**
   * @brief Runs search implementation, outward from the start position
   * @param position Initial position to search from
   * @return List of frontiers, if any
   */
  std::vector<Frontier> searchFrom(geometry_msgs::msg::Point position);

protected:
  /**
   * @brief Starting from an initial cell, build a frontier from valid adjacent
   * cells
   * @param initial_cell Index of cell to start frontier building
   * @param reference Reference index to calculate position from
   * @param frontier_flag Flag vector indicating which cells are already marked
   * as frontiers
   * @return new frontier
   */
  Frontier buildNewFrontier(unsigned int initial_cell, unsigned int reference,
                            std::vector<bool>& frontier_flag);

  /**
   * @brief isNewFrontierCell Evaluate if candidate cell is a valid candidate
   * for a new frontier.
   * @param idx Index of candidate cell
   * @param frontier_flag Flag vector indicating which cells are already marked
   * as frontiers
   * @return true if the cell is frontier cell
   */
  bool isNewFrontierCell(unsigned int idx,
                         const std::vector<bool>& frontier_flag);

  /**
   * @brief computes frontier cost
   * @details cost function is defined by potential_scale and gain_scale
   *
   * @param frontier frontier for which compute the cost
   * @return cost of the frontier
   */
  double frontierCost(const Frontier& frontier);

private:
  nav2_costmap_2d::Costmap2D* costmap_;
  unsigned char* map_;
  unsigned int size_x_, size_y_;
  double potential_scale_, gain_scale_;
  double min_frontier_size_;
  double robot_exclusion_radius_;
  rclcpp::Logger logger_;
};
}  // namespace frontier_exploration
#endif
