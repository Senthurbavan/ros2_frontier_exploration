#ifndef TEST_EXPLORE_EXPLORE_FRONTIER_TEST_HPP
#define TEST_EXPLORE_EXPLORE_FRONTIER_TEST_HPP

#include <rclcpp/rclcpp.hpp>
#include "nav2_util/lifecycle_node.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "nav2_util/node_utils.hpp"
#include <visualization_msgs/msg/marker_array.hpp>
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include "tf2/time.h"
#include "tf2_ros/create_timer_ros.h"
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

    bool goalOnBlacklist(const geometry_msgs::msg::Point& goal);

    void returnToDocking();

    std::unique_ptr<Costmap2DClient> costmap_client_{nullptr};
    std::unique_ptr<FrontierSearch> search_;

    // Transform listener
    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

    // Publishers
    rclcpp_lifecycle::LifecyclePublisher<visualization_msgs::msg::MarkerArray>
      ::SharedPtr marker_array_publisher_;
    rclcpp_lifecycle::LifecyclePublisher<visualization_msgs::msg::MarkerArray>
      ::SharedPtr f_path_publisher_;

    // Action client
    rclcpp::CallbackGroup::SharedPtr callback_group_;
    rclcpp_action::Client<NavigateToPose>::SharedPtr nav_to_pose_client_;
    std::shared_future<rclcpp_action::ClientGoalHandle<NavigateToPose>::SharedPtr> 
      future_goal_handle_;
    
    // Data
    std::string global_frame_;      ///< @brief The global frame for the costmap
    std::string robot_base_frame_;  ///< @brief The frame_id of the robot base
    int64_t explore_period_;
    rclcpp::TimerBase::SharedPtr client_timer_, oneshot_;
    rclcpp::Clock::SharedPtr clock_;
    std::shared_ptr<rclcpp::Duration> terminate_timeout_, progress_timeout_;
    rclcpp::Time last_nonempty_frontier_, last_progress_;
    double docking_x_, docking_y_;
    std::vector<geometry_msgs::msg::Point> frontier_blacklist_;
    geometry_msgs::msg::Point prev_goal_;
    double prev_distance_;

    // Visualization
    size_t last_markers_count_, last_path_count_;
    bool visualize_;

  };

} // namespace
#endif