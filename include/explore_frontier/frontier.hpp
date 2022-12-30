#ifndef FRONTIER_H_
#define FRONTIER_H_

#include <vector>
#include <geometry_msgs/msg/point.hpp>

namespace explore_frontier
{

struct Frontier
  {
    std::uint32_t size;
    double min_distance; // distance to middle
    double cost;
    geometry_msgs::msg::Point initial; // initially identified cell
    geometry_msgs::msg::Point centroid;
    geometry_msgs::msg::Point middle; // closest cell by Eucledian dist or planner
    std::vector<geometry_msgs::msg::Point> points;
    std::vector<geometry_msgs::msg::Point> path; // path to the frontier from current location
  };

}
#endif