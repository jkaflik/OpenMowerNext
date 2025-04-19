#include "docking_helper/docking_helper_node.hpp"
#include <rclcpp/rclcpp.hpp>

using open_mower_next::docking_helper::DockingHelperNode;

int main(int argc, char* argv[])
{
  rclcpp::init(argc, argv);

  auto options = rclcpp::NodeOptions();
  auto node = std::make_shared<DockingHelperNode>(options);

  RCLCPP_INFO(node->get_logger(), "Starting docking helper node");

  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
