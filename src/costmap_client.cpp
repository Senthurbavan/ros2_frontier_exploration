/*********************************************************************
 *
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2015-2016, Jiri Horner.
 *  Copyright (c) 2022-2023, Senthurbavan Kirubaharan.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the Jiri Horner nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 *********************************************************************/

#include "explore_frontier/costmap_client.hpp"
#include <mutex>
#include <string>

namespace explore_frontier
{

// static translation table to speed things up
std::array<unsigned char, 256> init_translation_table();

static const std::array<unsigned char, 256> cost_translation_table__ =
  init_translation_table();

Costmap2DClient::Costmap2DClient(
  const nav2_util::LifecycleNode::WeakPtr &parent)
{
  auto node = parent.lock();
  logger_ = node->get_logger();

  node->declare_parameter("costmap_topic", rclcpp::ParameterValue("costmap"));
  node->get_parameter("costmap_topic", costmap_topic_);

  node->declare_parameter("costmap_updates_topic", 
                          rclcpp::ParameterValue("costmap_updates"));
  node->get_parameter("costmap_updates_topic", costmap_updates_topic_);

  node->declare_parameter("robot_base_frame", rclcpp::ParameterValue("base_footprint"));
  node->get_parameter("robot_base_frame", robot_base_frame_);

  node->declare_parameter("transform_tolerance", rclcpp::ParameterValue(0.2));
  node->get_parameter("transform_tolerance", transform_tolerance_);

  costmap_sub_ = node->create_subscription<nav_msgs::msg::OccupancyGrid>(
    costmap_topic_, 
    10, 
    std::bind(&Costmap2DClient::updateFullMap, this, std::placeholders::_1));

  //wait for costmap to become available

  costmap_updates_sub_ = node->create_subscription<map_msgs::msg::OccupancyGridUpdate>(
    costmap_updates_topic_, 
    10, 
    std::bind(&Costmap2DClient::updatePartialMap, this, std::placeholders::_1));

  // wait for tf to become available

}

void Costmap2DClient::updateFullMap(const nav_msgs::msg::OccupancyGrid::ConstSharedPtr &msg)
{
  global_frame_ = msg->header.frame_id;

  unsigned int size_in_cells_x = msg->info.width;
  unsigned int size_in_cells_y = msg->info.height;
  double resolution = msg->info.resolution;
  double origin_x = msg->info.origin.position.x;
  double origin_y = msg->info.origin.position.y;

  RCLCPP_DEBUG(logger_, "received full new map, resizing to: %d, %d", size_in_cells_x,
            size_in_cells_y);
  costmap_.resizeMap(size_in_cells_x, size_in_cells_y, resolution, origin_x,
                      origin_y);

  // lock as we are accessing raw underlying map
  auto *mutex = costmap_.getMutex();
  std::lock_guard<nav2_costmap_2d::Costmap2D::mutex_t> lock(*mutex);

  // fill map with data
  unsigned char *costmap_data = costmap_.getCharMap();
  size_t costmap_size = costmap_.getSizeInCellsX() * costmap_.getSizeInCellsY();
  RCLCPP_DEBUG(logger_, "full map update, %lu values", costmap_size);
  for (size_t i = 0; i < costmap_size && i < msg->data.size(); ++i)
  {
    unsigned char cell_cost = static_cast<unsigned char>(msg->data[i]);
    costmap_data[i] = cost_translation_table__[cell_cost];
  }
  RCLCPP_DEBUG(logger_, "map updated, written %lu values", costmap_size);
}

void Costmap2DClient::updatePartialMap(
      const map_msgs::msg::OccupancyGridUpdate::ConstSharedPtr &msg)
{
  RCLCPP_DEBUG(logger_, "received partial map update");
  global_frame_ = msg->header.frame_id;

  if (msg->x < 0 || msg->y < 0)
  {
    RCLCPP_ERROR(logger_, "negative coordinates, invalid update. x: %d, y: %d", msg->x,
              msg->y);
    return;
  }

  size_t x0 = static_cast<size_t>(msg->x);
  size_t y0 = static_cast<size_t>(msg->y);
  size_t xn = msg->width + x0;
  size_t yn = msg->height + y0;

  // lock as we are accessing raw underlying map
  auto *mutex = costmap_.getMutex();
  std::lock_guard<nav2_costmap_2d::Costmap2D::mutex_t> lock(*mutex);

  size_t costmap_xn = costmap_.getSizeInCellsX();
  size_t costmap_yn = costmap_.getSizeInCellsY();

  if (xn > costmap_xn || x0 > costmap_xn || yn > costmap_yn ||
      y0 > costmap_yn)
  {
    RCLCPP_WARN(logger_, "received update doesn't fully fit into existing map, "
              "only part will be copied. received: [%lu, %lu], [%lu, %lu] "
              "map is: [0, %lu], [0, %lu]",
              x0, xn, y0, yn, costmap_xn, costmap_yn);
  }

  // update map with data
  unsigned char *costmap_data = costmap_.getCharMap();
  size_t i = 0;
  for (size_t y = y0; y < yn && y < costmap_yn; ++y)
  {
    for (size_t x = x0; x < xn && x < costmap_xn; ++x)
    {
      size_t idx = costmap_.getIndex(x, y);
      unsigned char cell_cost = static_cast<unsigned char>(msg->data[i]);
      costmap_data[idx] = cost_translation_table__[cell_cost];
      ++i;
    }
  }
}


bool Costmap2DClient::getRobotPose(geometry_msgs::msg::PoseStamped global_pose) const
{
  return nav2_util::getCurrentPose(
    global_pose, *tf_buffer_, global_frame_, robot_base_frame_, transform_tolerance_);  
}


std::array<unsigned char, 256> init_translation_table()
{
  std::array<unsigned char, 256> cost_translation_table;

  // lineary mapped from [0..100] to [0..255]
  for (size_t i = 0; i < 256; ++i)
  {
    cost_translation_table[i] =
        static_cast<unsigned char>(1 + (251 * (i - 1)) / 97);
  }

  // special values:
  cost_translation_table[0] = 0;                                // NO obstacle
  cost_translation_table[99] = 253;                             // INSCRIBED obstacle
  cost_translation_table[100] = 254;                            // LETHAL obstacle
  cost_translation_table[static_cast<unsigned char>(-1)] = 255; // UNKNOWN

  return cost_translation_table;
}

}