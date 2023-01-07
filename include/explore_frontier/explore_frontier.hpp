#ifndef TEST_EXPLORE_EXPLORE_FRONTIER_TEST_HPP
#define TEST_EXPLORE_EXPLORE_FRONTIER_TEST_HPP

#include <rclcpp/rclcpp.hpp>
#include "nav2_util/lifecycle_node.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "nav2_util/node_utils.hpp"
#include <visualization_msgs/msg/marker_array.hpp>

#include <explore_frontier/costmap_client.hpp>
#include <explore_frontier/frontier.hpp>
#include <explore_frontier/frontier_search.hpp>
#include <explore_frontier/astar.hpp>


namespace explore_frontier
{

  class ExploreFrontier : public nav2_util::LifecycleNode
  {
  public:

    using NavigateToPose = nav2_msgs::action::NavigateToPose;
    using GoalHandleNavigateToPose = rclcpp_action::ClientGoalHandle<NavigateToPose>;

    explicit ExploreFrontier(
      const rclcpp::NodeOptions &options = rclcpp::NodeOptions());
    ~ExploreFrontier();

    void start();
    void stop();

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
    void resultCallback( const GoalHandleNavigateToPose::WrappedResult & result,
                        const geometry_msgs::msg::Point& frontier_goal);
    void goalResponseCallback(const GoalHandleNavigateToPose::SharedPtr & goal_handle);

    // Exploration
    /**
     * @brief  Make a global plan
     */
    void makePlan();

    /**
     * @brief  Publish a frontiers as markers
     */
    void visualizeFrontiers(
        const std::vector<explore_frontier::Frontier>& frontiers,
        explore_frontier::Frontier &goal_frontier);

    void visualizeFrontierPaths(
        const std::vector<explore_frontier::Frontier>& frontiers);

    // void reachedGoal(const actionlib::SimpleClientGoalState& status,
    //                 const xavier_msgs::XavierMoveBaseResultConstPtr& result,
    //                 const geometry_msgs::Point& frontier_goal);

    bool goalOnBlacklist(const geometry_msgs::msg::Point& goal);

    void returnToDocking();



    /* data */
    int64_t timer_dur;
    rclcpp::TimerBase::SharedPtr client_timer_, oneshot_;
    rclcpp::Clock::SharedPtr clock_;

    // Publishers
    rclcpp_lifecycle::LifecyclePublisher<visualization_msgs::msg::MarkerArray>
      ::SharedPtr marker_array_publisher_;
    rclcpp_lifecycle::LifecyclePublisher<visualization_msgs::msg::MarkerArray>
      ::SharedPtr f_path_publisher_;

    // Action client
    rclcpp::CallbackGroup::SharedPtr callback_group_, timer_callback_group_;
    rclcpp::executors::SingleThreadedExecutor callback_group_executor_, 
      timer_callback_group_executor_;
    rclcpp_action::Client<NavigateToPose>::SharedPtr nav_to_pose_client_;
    std::shared_future<rclcpp_action::ClientGoalHandle<NavigateToPose>::SharedPtr> 
      future_goal_handle_;

    // Costmap2DClient costmap_client_;
    // explore_frontier::FrontierSearch search_;

    std::unique_ptr<Costmap2DClient> costmap_client_{nullptr};
    std::unique_ptr<FrontierSearch> search_;

    std::vector<geometry_msgs::msg::Point> frontier_blacklist_;
    geometry_msgs::msg::Point prev_goal_;
    double prev_distance_;

    // Visualization
    size_t last_markers_count_, last_path_count_;

    std::shared_ptr<rclcpp::Duration> terminate_timeout_, progress_timeout_;
    rclcpp::Time last_nonempty_frontier_, last_progress_;

  };

} // namespace
#endif