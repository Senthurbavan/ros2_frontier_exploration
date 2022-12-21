#ifndef TEST_EXPLORE_EXPLORE_FRONTIER_TEST_HPP
#define TEST_EXPLORE_EXPLORE_FRONTIER_TEST_HPP

#include "nav2_util/lifecycle_node.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "nav2_util/node_utils.hpp"


namespace test_explore
{

  class ExploreFrontierTest : public nav2_util::LifecycleNode
  {
  public:

    using NavigateToPose = nav2_msgs::action::NavigateToPose;
    using GoalHandleNavigateToPose = rclcpp_action::ClientGoalHandle<NavigateToPose>;

    explicit ExploreFrontierTest(
      const rclcpp::NodeOptions &options = rclcpp::NodeOptions());
    ~ExploreFrontierTest();

  protected:
    nav2_util::CallbackReturn on_configure(
      const rclcpp_lifecycle::State &state) override;
    nav2_util::CallbackReturn on_activate(
      const rclcpp_lifecycle::State &state) override;
    nav2_util::CallbackReturn on_deactivate(
      const rclcpp_lifecycle::State &state) override;
    nav2_util::CallbackReturn on_cleanup(
      const rclcpp_lifecycle::State &state) override;
    nav2_util::CallbackReturn on_shutdown(
      const rclcpp_lifecycle::State &state) override;

    void clientTimerCallback();
    void resultCallback( const GoalHandleNavigateToPose::WrappedResult & result);
    void goalResponseCallback(const GoalHandleNavigateToPose::SharedPtr & goal_handle);


    /* data */
    int64_t timer_dur;
    rclcpp::TimerBase::SharedPtr client_timer_;

    // Action client
    rclcpp::CallbackGroup::SharedPtr callback_group_, timer_callback_group_;
    rclcpp::executors::SingleThreadedExecutor callback_group_executor_, 
      timer_callback_group_executor_;
    rclcpp_action::Client<NavigateToPose>::SharedPtr nav_to_pose_client_;
    std::shared_future<rclcpp_action::ClientGoalHandle<NavigateToPose>::SharedPtr> 
      future_goal_handle_;

  };

} // namespace
#endif