#include "explore_frontier/explore_frontier.hpp"

namespace explore_frontier
{

ExploreFrontier::ExploreFrontier(const rclcpp::NodeOptions &options)
  : nav2_util::LifecycleNode("explore_frontier", "", options)//,
    // costmap_client_(shared_from_this()
{
  RCLCPP_INFO(get_logger(), "\n -- Creating -- \n");
  declare_parameter("timer_duration", 1);
}


ExploreFrontier::~ExploreFrontier()
{
}


nav2_util::CallbackReturn ExploreFrontier::on_configure(
  const rclcpp_lifecycle::State & /*state*/)
{
  RCLCPP_INFO(get_logger(), "\n -- Configuring --\n");

  clock_ = this->get_clock();

  this->timer_dur = get_parameter("timer_duration").as_int();
  RCLCPP_INFO(get_logger(), "timer_duration is: %ld", this->timer_dur);

  terminate_timeout_ = std::make_shared<rclcpp::Duration>(
    rclcpp::Duration::from_seconds(30));

  progress_timeout_ = std::make_shared<rclcpp::Duration>(
    rclcpp::Duration::from_seconds(30));

  marker_array_publisher_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(
    "frontiers", 10);
  f_path_publisher_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(
    "frontier_paths", 10);

  this->nav_to_pose_client_ = rclcpp_action::create_client<NavigateToPose>(
    get_node_base_interface(),
    get_node_graph_interface(),
    get_node_logging_interface(),
    get_node_waitables_interface(),
    "navigate_to_pose"/*, callback_group_*/
  );
  RCLCPP_INFO(get_logger(), "costmap_client_ not set non-empty: %s", costmap_client_?"T":"F");
  costmap_client_ = std::make_unique<Costmap2DClient>(shared_from_this());
  RCLCPP_INFO(get_logger(), "costmap_client_ set non-empty: %s", costmap_client_?"T":"F");
  search_ = std::make_unique<FrontierSearch>(shared_from_this(),
    costmap_client_->getCostmap(), 1.0, 1.0, 0.75);
  RCLCPP_INFO(get_logger(), "FrontierSearch ready");

  return nav2_util::CallbackReturn::SUCCESS;
}


nav2_util::CallbackReturn ExploreFrontier::on_activate(
  const rclcpp_lifecycle::State & /*state*/)
{
  RCLCPP_INFO(get_logger(), "\n -- Activating -- \n");

  createBond();

  this->marker_array_publisher_->on_activate();
  this->f_path_publisher_->on_activate();

  this->client_timer_ = create_wall_timer(
    std::chrono::seconds(this->timer_dur), 
    std::bind(&ExploreFrontier::makePlan, this)/*, timer_callback_group_*/
  );

  return nav2_util::CallbackReturn::SUCCESS;
}


nav2_util::CallbackReturn ExploreFrontier::on_deactivate(
  const rclcpp_lifecycle::State & /*state*/)
{
  RCLCPP_INFO(get_logger(), "\n -- Deactivating -- \n");

  this->client_timer_->cancel();
  if(oneshot_)
  {
    this->oneshot_->cancel();
  }
  this->marker_array_publisher_->on_deactivate();
  this->f_path_publisher_->on_deactivate();

  destroyBond();
  return nav2_util::CallbackReturn::SUCCESS;
}


nav2_util::CallbackReturn ExploreFrontier::on_cleanup(
  const rclcpp_lifecycle::State & /*state*/)
{
  RCLCPP_INFO(get_logger(), "\n -- Cleaning up -- \n");

  this->nav_to_pose_client_.reset();
  this->marker_array_publisher_.reset();
  this->f_path_publisher_.reset();

  return nav2_util::CallbackReturn::SUCCESS;
}


nav2_util::CallbackReturn ExploreFrontier::on_shutdown(
  const rclcpp_lifecycle::State & /*state*/)
{
  RCLCPP_INFO(get_logger(), "\n -- Shutting down -- \n");
  return nav2_util::CallbackReturn::SUCCESS;
}


void ExploreFrontier::resultCallback(
  const GoalHandleNavigateToPose::WrappedResult & result,
  const geometry_msgs::msg::Point& frontier_goal)
{
  RCLCPP_INFO(get_logger(), "Result callback");

  rclcpp_action::ResultCode code = result.code;
  std::string code_str;

  if (code == rclcpp_action::ResultCode::SUCCEEDED)
  {
    code_str = "SUCCEEDED";
  } else if (code == rclcpp_action::ResultCode::CANCELED)
  {
    code_str = "CANCELED";
  }else if (code == rclcpp_action::ResultCode::ABORTED)
  {
    code_str = "ABORTED";
  }else
  {
    code_str = "UNKNOWN";
  }
  RCLCPP_INFO(get_logger(), "navigate_to_pose result received : %s", code_str.c_str());

  if (code == rclcpp_action::ResultCode::ABORTED)
  {
    frontier_blacklist_.push_back(frontier_goal);
    RCLCPP_DEBUG(get_logger(), "Adding current goal to black list");
  }

  oneshot_ = this->create_wall_timer(
    std::chrono::seconds(0), 
    std::bind(&ExploreFrontier::makePlan, this)/*, timer_callback_group_*/
  );
}


void ExploreFrontier::goalResponseCallback(
  const GoalHandleNavigateToPose::SharedPtr & goal_handle)
{
  RCLCPP_INFO(get_logger(), "Goal Response Callback");

  if(!goal_handle)
  {
    RCLCPP_ERROR(get_logger(), 
                  "navigate_to_pose action client failed to send goal to server.");
  }
}


void ExploreFrontier::visualizeFrontiers(
  const std::vector<explore_frontier::Frontier>& frontiers,
  explore_frontier::Frontier &goal_frontier)
{
  std_msgs::msg::ColorRGBA blue;
  blue.r = 0;
  blue.g = 0;
  blue.b = 1.0;
  blue.a = 1.0;
  std_msgs::msg::ColorRGBA red;
  red.r = 1.0;
  red.g = 0;
  red.b = 0;
  red.a = 1.0;
  std_msgs::msg::ColorRGBA green;
  green.r = 0;
  green.g = 1.0;
  green.b = 0;
  green.a = 1.0;

  RCLCPP_DEBUG(get_logger(), "visualising %lu frontiers", frontiers.size());
  visualization_msgs::msg::MarkerArray markers_msg;
  std::vector<visualization_msgs::msg::Marker> &markers = markers_msg.markers;
  visualization_msgs::msg::Marker m;

  m.header.frame_id = costmap_client_->getGlobalFrameID();
  m.header.stamp = clock_->now();
  m.ns = "frontiers";
  m.scale.x = 1.0;
  m.scale.y = 1.0;
  m.scale.z = 1.0;
  m.color.r = 0;
  m.color.g = 0;
  m.color.b = 255;
  m.color.a = 255;
  m.lifetime = rclcpp::Duration(0,0);
  m.frame_locked = true;

  // weighted frontiers are always sorted
  double min_cost = frontiers.empty() ? 0. : frontiers.front().cost;

  m.action = visualization_msgs::msg::Marker::ADD;
  int id = 0;

  for(const explore_frontier::Frontier &frontier : frontiers)
  {
    m.type = visualization_msgs::msg::Marker::POINTS;
    m.id = id;
    // m.pose.position = {}; // HOW TO HANDLE THIS?
    m.scale.x = 0.1;
    m.scale.y = 0.1;
    m.scale.z = 0.1;
    m.points = frontier.points;
    if (goalOnBlacklist(frontier.middle))
    {
      m.color = red;
    }
    else if(&frontier == &goal_frontier)
    {
      m.color = green;
    }
    else
    {
      m.color = blue;
    }
    markers.push_back(m);
    ++id;

    m.type = visualization_msgs::msg::Marker::SPHERE;
    m.id = id;
    m.pose.position = frontier.middle;
    // scale frontier according to its cost (costier frontiers will be smaller)
    double scale = std::min(std::abs(min_cost * 0.4 / frontier.cost), 0.5);
    m.scale.x = scale;
    m.scale.y = scale;
    m.scale.z = scale;
    m.points = {};
    if(&frontier == &goal_frontier)
    {
      m.color = red;
    }else
    {
      m.color = green;
    }
    markers.push_back(m);
    ++id;
  }

  size_t current_markers_count = markers.size();
  // delete previous markers, which are now unused
  m.action = visualization_msgs::msg::Marker::DELETE;
  for (; id < (int)last_markers_count_; ++id)
  {
    m.id = id;
    markers.push_back(m);
  }

  last_markers_count_ = current_markers_count;
  marker_array_publisher_->publish(std::move(markers_msg));
}


void ExploreFrontier::visualizeFrontierPaths(
                        const std::vector<explore_frontier::Frontier>& frontiers)
{
  std_msgs::msg::ColorRGBA blue;
  blue.r = 0;
  blue.g = 0;
  blue.b = 1.0;
  blue.a = 1.0;
  std_msgs::msg::ColorRGBA red;
  red.r = 1.0;
  red.g = 0;
  red.b = 0;
  red.a = 1.0;
  std_msgs::msg::ColorRGBA green;
  green.r = 0;
  green.g = 1.0;
  green.b = 0;
  green.a = 1.0;
  std_msgs::msg::ColorRGBA orange;
  orange.r = 1.0;
  orange.g = 1.0;
  orange.b = 0;
  orange.a = 1.0;

  visualization_msgs::msg::MarkerArray markers_msg;
  std::vector<visualization_msgs::msg::Marker> &markers = markers_msg.markers;
  visualization_msgs::msg::Marker m;

  m.header.frame_id = costmap_client_->getGlobalFrameID();
  m.header.stamp = clock_->now();
  m.ns = "frontier_paths";
  m.scale.x = 1.0;
  m.scale.y = 1.0;
  m.scale.z = 1.0;
  m.color.r = 0;
  m.color.g = 0;
  m.color.b = 255;
  m.color.a = 255;
  // lives forever
  m.lifetime = rclcpp::Duration(0,0);
  m.frame_locked = true;

  m.action = visualization_msgs::msg::Marker::ADD;
  int id = 0;

  for(const explore_frontier::Frontier &f: frontiers)
  {
    if (f.min_distance == std::numeric_limits<double>::infinity())
    {
      continue;
    }
    m.type = visualization_msgs::msg::Marker::LINE_STRIP;
    m.id = id;
    // m.pose.position = {};
    m.scale.x = 0.01;
    m.scale.y = 0.01;
    m.scale.z = 0.01;
    m.points = f.path;
    m.color = orange;
    m.pose.orientation.w = 1.0;
    markers.push_back(m);
    ++id;
  }

  size_t current_path_count = markers.size();
  m.action = visualization_msgs::msg::Marker::DELETE;
  for(; id < (int)last_path_count_;id++)
  {
    m.id = id;
    markers.push_back(m);
  }

  last_path_count_ = current_path_count;
  f_path_publisher_->publish(std::move(markers_msg));
}


void ExploreFrontier::makePlan()
{
  RCLCPP_INFO(get_logger(), "inside makePlan");

  // Just cancel the oneshot timer in case it triggered the callback
  if(this->oneshot_)
  {
    this->oneshot_->cancel();
  }
  RCLCPP_INFO(get_logger(), "oneshot canceled");

  std::vector<explore_frontier::Frontier>::iterator frontier;
  geometry_msgs::msg::PoseStamped pose;
  RCLCPP_INFO(get_logger(), "going to get robot pose empty: %s", costmap_client_?"F":"T");
  costmap_client_->getRobotPose(pose);
  RCLCPP_INFO(get_logger(), "got robot pose");

  std::vector<explore_frontier::Frontier> frontiers = 
                                              search_->searchFrom(pose.pose.position);

  RCLCPP_DEBUG(get_logger(), "found %lu frontiers", frontiers.size());
  for (size_t i = 0; i < frontiers.size(); ++i)
  {
    RCLCPP_DEBUG(get_logger(), "frontier %zd cost: %f", i, frontiers[i].cost);
  }

  if (frontiers.empty())
  {
    stop();
    // Erase everything
    if (true)
    {
      visualizeFrontiers(frontiers, *frontier);
      // visualizeFrontierPaths(frontiers);
    }
    // returnToDocking();
    return;
  }

  frontier = std::find_if_not(frontiers.begin(), frontiers.end(),
                                    [this](const explore_frontier::Frontier &f)
                                    {
                                        return (goalOnBlacklist(f.middle) ||
                                        f.min_distance == std::numeric_limits
                                        <double>::infinity() );
                                    });

  if (frontier == frontiers.end())
  {
    if((clock_->now() - last_nonempty_frontier_ > *terminate_timeout_))
    {
      stop();
      frontiers.clear();
      // Erase everything
      if (true)
      {
        visualizeFrontiers(frontiers, *frontier);
        // visualizeFrontierPaths(frontiers);
      }
      // returnToDocking();
      return;
    }else
    {
      // RCLCPP_DEBUG_THROTTLE(get_logger(), *clock_, 1000, "-- Empty Frontiers --");
    }      
  } else
  {
    last_nonempty_frontier_ = clock_->now();
  }

  // publish frontiers as visualization markers
  if (true)
  {
    visualizeFrontiers(frontiers, *frontier);
    // visualizeFrontierPaths(frontiers);
  }

  geometry_msgs::msg::Point target_position = frontier->middle;

  // time out if we are not making any progress
  bool same_goal = prev_goal_ == target_position;
  prev_goal_ = target_position;
  if (!same_goal || prev_distance_ > frontier->min_distance)
  {
    // we have different goal or we made some progress
    last_progress_ = clock_->now();
    prev_distance_ = frontier->min_distance;
  }
  // black list if we've made no progress for a long time
  if (clock_->now() - last_progress_ > *progress_timeout_)
  {
    frontier_blacklist_.push_back(target_position);
    RCLCPP_DEBUG(get_logger(), "Adding current goal to black list");
    makePlan();
    return;
  }

  // we don't need to do anything if we still pursuing the same goal
  if (same_goal)
  {
    return;
  }

  NavigateToPose::Goal client_goal;
  client_goal.pose.header.frame_id = "map";

  client_goal.pose.pose.position.x = target_position.x;
  client_goal.pose.pose.position.y = target_position.y;
  client_goal.pose.pose.position.z = target_position.z;

  client_goal.pose.pose.orientation.w = 1.0;
  client_goal.pose.pose.orientation.x = 0.0;
  client_goal.pose.pose.orientation.y = 0.0;
  client_goal.pose.pose.orientation.z = 0.0;  

  auto send_goal_options = rclcpp_action::Client<NavigateToPose>::SendGoalOptions();
  send_goal_options.result_callback = [this,target_position](
      const GoalHandleNavigateToPose::WrappedResult & result)
      {
        resultCallback(result, target_position);
      };
  send_goal_options.goal_response_callback = std::bind(
    &ExploreFrontier::goalResponseCallback, this, std::placeholders::_1
  );

  future_goal_handle_ = nav_to_pose_client_->async_send_goal(client_goal, 
                                                              send_goal_options);
  RCLCPP_INFO(get_logger(), "makePlan: Goal sent to the server");
}


void ExploreFrontier::clientTimerCallback()
{
  RCLCPP_INFO(get_logger(), "timer callback");
  
  if(oneshot_)
  {
    this->oneshot_->cancel();
  }

  NavigateToPose::Goal client_goal;
  geometry_msgs::msg::Point target_position;

  client_goal.pose.header.frame_id = "map";

  client_goal.pose.pose.position.x = 0.32;
  client_goal.pose.pose.position.y = -0.55;
  client_goal.pose.pose.position.z = 0.0;

  client_goal.pose.pose.orientation.w = 1.0;
  client_goal.pose.pose.orientation.x = 0.0;
  client_goal.pose.pose.orientation.y = 0.0;
  client_goal.pose.pose.orientation.z = 0.0;

  auto send_goal_options = rclcpp_action::Client<NavigateToPose>::SendGoalOptions();
  // send_goal_options.result_callback = std::bind(
  //   &ExploreFrontier::resultCallback, this, std::placeholders::_1
  // );
  send_goal_options.result_callback = [this,target_position](
      const GoalHandleNavigateToPose::WrappedResult & result)
      {
        resultCallback(result, target_position);
      };
  send_goal_options.goal_response_callback = std::bind(
    &ExploreFrontier::goalResponseCallback, this, std::placeholders::_1
  );
  future_goal_handle_ = nav_to_pose_client_->async_send_goal(client_goal, 
                                                              send_goal_options);
  
  RCLCPP_INFO(get_logger(), "Goal sent to the server");
}


bool ExploreFrontier::goalOnBlacklist(const geometry_msgs::msg::Point &goal)
{
  int tolerance = 5;
  nav2_costmap_2d::Costmap2D *costmap2d = costmap_client_->getCostmap();

  for(geometry_msgs::msg::Point& frontier_goal : frontier_blacklist_)
  {
    double x_diff = fabs(goal.x - frontier_goal.x);
    double y_diff = fabs(goal.y - frontier_goal.y);

    if (x_diff < tolerance * costmap2d->getResolution() &&
      y_diff < tolerance * costmap2d->getResolution())
        return true;
  }
  return false;
}

void ExploreFrontier::stop()
{
  // actionClient.cancelAllGoals();
  // exploring_timer_.stop();
  // ROS_INFO("Exploration stopped.");
  this->nav_to_pose_client_->async_cancel_all_goals();
  this->client_timer_->cancel();
  if(this->oneshot_)
  {
    this->oneshot_->cancel();
  }
  RCLCPP_INFO(get_logger(), "Exploration stopped");
}

} // namespace explore_frontier

#include "rclcpp_components/register_node_macro.hpp"

RCLCPP_COMPONENTS_REGISTER_NODE(explore_frontier::ExploreFrontier)
