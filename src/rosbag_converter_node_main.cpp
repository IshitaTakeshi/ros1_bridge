#include <memory>
#include <exception>
#include "ros1_bridge/rosbag_converter_node.h"

int main(int argc, char * argv[])
{
  auto args = rclcpp::init_and_remove_ros_arguments(argc, argv);
  rclcpp::NodeOptions node_options;
  node_options.arguments(args);

  const auto rosbag_converter_node_ptr = std::make_shared<RosbagConverterNode>(node_options);

  // It won't subscribe to anything, no reason to spin.
//  rclcpp::spin(rosbag_converter_node_ptr);

  if (!rclcpp::shutdown()) {
    throw std::runtime_error{"Shutdown failed."};
  }

  return 0;
}
