#ifndef TEST_EXPLORE_EXPLORE_FRONTIER_TEST_HPP
#define TEST_EXPLORE_EXPLORE_FRONTIER_TEST_HPP

#include "nav2_util/lifecycle_node.hpp"

#include "nav2_util/node_utils.hpp"

namespace test_explore
{

class ExploreFrontierTest : public nav2_util::LifecycleNode
{
public:
    explicit ExploreFrontierTest(
        const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
    ~ExploreFrontierTest();

protected:
    /* data */
    nav2_util::CallbackReturn on_configure(const rclcpp_lifecycle::State & state) override;
    nav2_util::CallbackReturn on_activate(const rclcpp_lifecycle::State & state) override;
    nav2_util::CallbackReturn on_deactivate(const rclcpp_lifecycle::State & state) override;
    nav2_util::CallbackReturn on_cleanup(const rclcpp_lifecycle::State & state) override;
    nav2_util::CallbackReturn on_shutdown(const rclcpp_lifecycle::State & state) override;
};


} // namespace
#endif