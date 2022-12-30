#ifndef ASTAR_H_
#define ASTAR_H_

#include <rclcpp/rclcpp.hpp>
#include <vector>
#include <stack>
#include <float.h>
#include<nav2_costmap_2d/cost_values.hpp>
#include<nav2_costmap_2d/costmap_2d.hpp>
#include "explore_frontier/frontier.hpp"

namespace astar
{

typedef std::pair<unsigned int, unsigned int> Pair;
typedef std::pair<double, std::pair<unsigned int, unsigned int>> pPair;

struct cell
{
    int parent_i, parent_j;
    double f, g, h;
};

class AStarPlanner
{
  public:
  AStarPlanner();
  AStarPlanner(nav2_costmap_2d::Costmap2D* costmap);
  ~AStarPlanner();
  bool isValid(unsigned int row, unsigned int col);
  bool isUnBlocked(unsigned int row, unsigned int col);
  bool isDestination(unsigned int row, unsigned int col, 
                       explore_frontier::Frontier& f);
  double calculateHValue(unsigned int row, unsigned int col, Pair dest); 
  void tracePath(std::vector<std::vector<cell>> &cellDetails, Pair dest, 
                  std::vector<geometry_msgs::msg::Point> &outPath);
  bool searchPath(Pair src, explore_frontier::Frontier& f);

  private:
  nav2_costmap_2d::Costmap2D* costmap_;
};
}

#endif