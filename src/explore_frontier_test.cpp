#include "test_explore/explore_frontier_test.hpp"


namespace test_explore
{

ExploreFrontierTest::ExploreFrontierTest(const rclcpp::NodeOptions & options)
: nav2_util::LifecycleNode("explore_frontier_test", "", options)
{
    RCLCPP_INFO(get_logger(), "\n -- Creating -- \n");
}

ExploreFrontierTest::~ExploreFrontierTest()
{
}

nav2_util::CallbackReturn ExploreFrontierTest::on_configure(
    const rclcpp_lifecycle::State & /*state*/)
{
    RCLCPP_INFO(get_logger(), "\n -- Configuring --\n");
    return nav2_util::CallbackReturn::SUCCESS;
}

nav2_util::CallbackReturn ExploreFrontierTest::on_activate(
    const rclcpp_lifecycle::State & /*state*/)
{
    RCLCPP_INFO(get_logger(), "\n -- Activating -- \n");
    createBond();
    return nav2_util::CallbackReturn::SUCCESS;
}

nav2_util::CallbackReturn ExploreFrontierTest::on_deactivate(
    const rclcpp_lifecycle::State & /*state*/)
{
    RCLCPP_INFO(get_logger(), "\n -- Deactivating -- \n");
    destroyBond();
    return nav2_util::CallbackReturn::SUCCESS;
}

nav2_util::CallbackReturn ExploreFrontierTest::on_cleanup(
    const rclcpp_lifecycle::State & /*state*/)
{
    RCLCPP_INFO(get_logger(), "\n -- Cleaning up -- \n");
    return nav2_util::CallbackReturn::SUCCESS;
}

nav2_util::CallbackReturn ExploreFrontierTest::on_shutdown(
    const rclcpp_lifecycle::State & /*state*/)
{
    RCLCPP_INFO(get_logger(), "\n -- Shutting down -- \n");
    return nav2_util::CallbackReturn::SUCCESS;
}

} // namespace test_explore

#include "rclcpp_components/register_node_macro.hpp"

RCLCPP_COMPONENTS_REGISTER_NODE(test_explore::ExploreFrontierTest)
