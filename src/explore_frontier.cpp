#include "explore_frontier/explore_frontier.hpp"

namespace explore_frontier
{

ExploreFrontier::ExploreFrontier(const rclcpp::NodeOptions &options)
  : nav2_util::LifecycleNode("explore_frontier", "", options)
{
  RCLCPP_INFO(get_logger(), "\n -- Creating -- \n");
  declare_parameter("timer_duration", 10);
}

ExploreFrontier::~ExploreFrontier()
{
}

nav2_util::CallbackReturn ExploreFrontier::on_configure(
  const rclcpp_lifecycle::State & /*state*/)
{
  RCLCPP_INFO(get_logger(), "\n -- Configuring --\n");

  this->timer_dur = get_parameter("timer_duration").as_int();

  this->nav_to_pose_client_ = rclcpp_action::create_client<NavigateToPose>(
    get_node_base_interface(),
    get_node_graph_interface(),
    get_node_logging_interface(),
    get_node_waitables_interface(),
    "navigate_to_pose"/*, callback_group_*/
  );

  return nav2_util::CallbackReturn::SUCCESS;
}

nav2_util::CallbackReturn ExploreFrontier::on_activate(
  const rclcpp_lifecycle::State & /*state*/)
{
  RCLCPP_INFO(get_logger(), "\n -- Activating -- \n");
  createBond();

  this->client_timer_ = create_wall_timer(
    std::chrono::seconds(timer_dur), 
    std::bind(&ExploreFrontier::clientTimerCallback, this)/*, timer_callback_group_*/
  );

  return nav2_util::CallbackReturn::SUCCESS;
}

nav2_util::CallbackReturn ExploreFrontier::on_deactivate(
  const rclcpp_lifecycle::State & /*state*/)
{
  RCLCPP_INFO(get_logger(), "\n -- Deactivating -- \n");

  this->client_timer_->cancel();

  destroyBond();
  return nav2_util::CallbackReturn::SUCCESS;
}

nav2_util::CallbackReturn ExploreFrontier::on_cleanup(
  const rclcpp_lifecycle::State & /*state*/)
{
  RCLCPP_INFO(get_logger(), "\n -- Cleaning up -- \n");
  this->nav_to_pose_client_.reset();

  return nav2_util::CallbackReturn::SUCCESS;
}

nav2_util::CallbackReturn ExploreFrontier::on_shutdown(
  const rclcpp_lifecycle::State & /*state*/)
{
  RCLCPP_INFO(get_logger(), "\n -- Shutting down -- \n");
  return nav2_util::CallbackReturn::SUCCESS;
}

void ExploreFrontier::clientTimerCallback()
{
  RCLCPP_INFO(get_logger(), "timer callback");
  
  this->client_timer_->cancel();

  NavigateToPose::Goal client_goal;

  client_goal.pose.header.frame_id = "map";

  client_goal.pose.pose.position.x = 0.32;
  client_goal.pose.pose.position.y = -0.55;
  client_goal.pose.pose.position.z = 0.0;

  client_goal.pose.pose.orientation.w = 1.0;
  client_goal.pose.pose.orientation.x = 0.0;
  client_goal.pose.pose.orientation.y = 0.0;
  client_goal.pose.pose.orientation.z = 0.0;

  auto send_goal_options = rclcpp_action::Client<NavigateToPose>::SendGoalOptions();
  send_goal_options.result_callback = std::bind(
    &ExploreFrontier::resultCallback, this, std::placeholders::_1
  );
  send_goal_options.goal_response_callback = std::bind(
    &ExploreFrontier::goalResponseCallback, this, std::placeholders::_1
  );
  future_goal_handle_ = nav_to_pose_client_->async_send_goal(client_goal, 
                                                              send_goal_options);
  RCLCPP_INFO(get_logger(), "Goal sent to the server");
}

void ExploreFrontier::resultCallback(
  const GoalHandleNavigateToPose::WrappedResult & result)
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

} // namespace explore_frontier

#include "rclcpp_components/register_node_macro.hpp"

RCLCPP_COMPONENTS_REGISTER_NODE(explore_frontier::ExploreFrontier)
