#include <explore_frontier/frontier_search.hpp>

#include <mutex>

#include <nav2_costmap_2d/cost_values.hpp>
#include <nav2_costmap_2d/costmap_2d.hpp>
#include <geometry_msgs/msg/point.hpp>

#include <explore_frontier/costmap_tools.hpp>

namespace explore_frontier
{

using nav2_costmap_2d::FREE_SPACE;
using nav2_costmap_2d::INSCRIBED_INFLATED_OBSTACLE;
using nav2_costmap_2d::LETHAL_OBSTACLE;
using nav2_costmap_2d::NO_INFORMATION;

FrontierSearch::FrontierSearch(const nav2_util::LifecycleNode::WeakPtr &parent,
                              nav2_costmap_2d::Costmap2D *costmap,
                              double potential_scale, double gain_scale,
                              double min_frontier_size): 
  costmap_(costmap), 
  potential_scale_(potential_scale), 
  gain_scale_(gain_scale), 
  min_frontier_size_(min_frontier_size)
{
  a_star_planner_ = astar::AStarPlanner(costmap);
  auto node = parent.lock();
  logger_ = node->get_logger();
}

std::vector<Frontier> FrontierSearch::searchFrom(geometry_msgs::msg::Point position)
{
  std::vector<Frontier> frontier_list;
  RCLCPP_DEBUG(logger_, "searchFrom start");

  // Sanity check that robot is inside costmap bounds before searching
  unsigned int mx, my;
  if (!costmap_->worldToMap(position.x, position.y, mx, my))
  {
    RCLCPP_ERROR(logger_, "Robot out of costmap bounds, cannot search for frontiers");
    return frontier_list;
  }

  // make sure map is consistent and locked for duration of search
  std::lock_guard<nav2_costmap_2d::Costmap2D::mutex_t> lock(*(costmap_->getMutex()));

  map_ = costmap_->getCharMap();
  size_x_ = costmap_->getSizeInCellsX();
  size_y_ = costmap_->getSizeInCellsY();

  // initialize flag arrays to keep track of visited and frontier cells
  std::vector<bool> frontier_flag(size_x_ * size_y_, false);
  std::vector<bool> visited_flag(size_x_ * size_y_, false);

  // initialize breadth first search
  std::queue<unsigned int> bfs;

  // find closest clear cell to start search
  unsigned int init_cell, clear, pos = costmap_->getIndex(mx, my);
  if (nearestCell(clear, pos, INSCRIBED_INFLATED_OBSTACLE-1, *costmap_))
  {
    // bfs.push(clear);
    init_cell = clear;
  }
  else
  {
    // bfs.push(pos);
    init_cell = pos;
    RCLCPP_WARN(logger_, "Could not find nearby clear cell to start search");
  }
  bfs.push(init_cell);
  visited_flag[bfs.front()] = true;

  while (!bfs.empty())
  {
    unsigned int idx = bfs.front();
    bfs.pop();

    // iterate over 4-connected neighbourhood
    for (unsigned nbr : nhood4(idx, *costmap_))
    {
      // add to queue all free, unvisited cells, use descending search in case
      // initialized on non-free cell
      if (map_[nbr] <= INSCRIBED_INFLATED_OBSTACLE && !visited_flag[nbr])
      {
        visited_flag[nbr] = true;
        bfs.push(nbr);
        // check if cell is new frontier cell (unvisited, NO_INFORMATION, free
        // neighbour)
      }
      else if (isNewFrontierCell(nbr, frontier_flag))
      {
        frontier_flag[nbr] = true;
        Frontier new_frontier = buildNewFrontier(nbr, pos, frontier_flag);
        if (new_frontier.size * costmap_->getResolution() >=
            min_frontier_size_)
        {
          frontier_list.push_back(new_frontier);
        }
      }
    }
  }

  geometry_msgs::msg::Point init_point;
  costmap_->indexToCells(init_cell, mx, my);
  costmap_->mapToWorld(mx, my, init_point.x, init_point.y);
  init_point.z = 0.0;
  // set costs of frontiers
  for (auto &frontier : frontier_list)
  {
    frontier.cost = frontierCost(frontier, init_point);
  }
  std::sort(
      frontier_list.begin(), frontier_list.end(),
      [](const Frontier &f1, const Frontier &f2)
      { return f1.cost < f2.cost; });

  return frontier_list;

}

Frontier FrontierSearch::buildNewFrontier(unsigned int initial_cell,
                                          unsigned int reference,
                                          std::vector<bool> &frontier_flag)
{
  // initialize frontier structure
  Frontier output;
  output.centroid.x = 0;
  output.centroid.y = 0;
  output.size = 1;
  output.min_distance = std::numeric_limits<double>::infinity();

  // record initial contact point for frontier
  unsigned int ix, iy;
  costmap_->indexToCells(initial_cell, ix, iy);
  costmap_->mapToWorld(ix, iy, output.initial.x, output.initial.y);

  // push initial gridcell onto queue
  std::queue<unsigned int> bfs;
  bfs.push(initial_cell);

  // cache reference position in world coords
  unsigned int rx, ry;
  double reference_x, reference_y;
  costmap_->indexToCells(reference, rx, ry);
  costmap_->mapToWorld(rx, ry, reference_x, reference_y);

  while (!bfs.empty())
  {
    unsigned int idx = bfs.front();
    bfs.pop();

    // try adding cells in 8-connected neighborhood to frontier
    for (unsigned int nbr : nhood8(idx, *costmap_))
    {
      // check if neighbour is a potential frontier cell
      if (isNewFrontierCell(nbr, frontier_flag))
      {
        // mark cell as frontier
        frontier_flag[nbr] = true;
        unsigned int mx, my;
        double wx, wy;
        costmap_->indexToCells(nbr, mx, my);
        costmap_->mapToWorld(mx, my, wx, wy);

        geometry_msgs::msg::Point point;
        point.x = wx;
        point.y = wy;
        output.points.push_back(point);

        // update frontier size
        output.size++;

        // update centroid of frontier
        output.centroid.x += wx;
        output.centroid.y += wy;

        // determine frontier's distance from robot, going by closest gridcell
        // to robot
        // double distance = sqrt(pow((double(reference_x) - double(wx)), 2.0) +
        //                        pow((double(reference_y) - double(wy)), 2.0));
        // if (distance < output.min_distance)
        // {
        //   output.min_distance = distance;
        //   output.middle.x = wx;
        //   output.middle.y = wy;
        // }

        // add to queue for breadth first search
        bfs.push(nbr);
      }
    }
  }
  // average out frontier centroid
  output.centroid.x /= output.size;
  output.centroid.y /= output.size;
  return output;
}

bool FrontierSearch::isNewFrontierCell(unsigned int idx,
                                        const std::vector<bool> &frontier_flag)
{
  // check that cell is unknown and not already marked as frontier
  if (map_[idx] != NO_INFORMATION || frontier_flag[idx])
  {
    return false;
  }

  // frontier cells should have at least one cell in 4-connected neighbourhood
  // that is free
  for (unsigned int nbr : nhood4(idx, *costmap_))
  {
    if (map_[nbr] < INSCRIBED_INFLATED_OBSTACLE)
    {
      return true;
    }
  }

  return false;
}

double FrontierSearch::frontierCost(Frontier &frontier, 
                                    geometry_msgs::msg::Point position)
{
  astar::Pair src;
  costmap_->worldToMap(position.x, position.y, src.second, src.first);
  bool path_found = a_star_planner_.searchPath(src, frontier);
  if (!path_found)
  {
    RCLCPP_DEBUG(logger_, "A* planner: no path to the frontier");
    frontier.min_distance = std::numeric_limits<double>::infinity();
    frontier.middle = frontier.centroid;
  }

  return (potential_scale_ * frontier.min_distance *
          costmap_->getResolution()) -
          (gain_scale_ * frontier.size * costmap_->getResolution());
}

} // end of the namespace