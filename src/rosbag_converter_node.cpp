#include "ros1_bridge/rosbag_converter_node.h"


RosbagConverterNode::RosbagConverterNode(const rclcpp::NodeOptions & options)
    : Node("rosbag_converter_node", options),
      pub_ptr_string_test_{this->create_publisher<std_msgs::msg::String>(
          "/rosbag_converter_test",
          10)}
{
  auto path_in = static_cast<std::string>(
      rclcpp::Node::declare_parameter(
          "path_in_ros1_bag_file").get<std::string>());

  std::cout << "path_in: " <<path_in << std::endl;

}
