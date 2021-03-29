#include "ros1_bridge/rosbag_converter_node.h"
#include "rclcpp/logger.hpp"
#include <map>
#include <vector>
#include <sstream>


RosbagConverterNode::RosbagConverterNode(const rclcpp::NodeOptions & options)
: Node("rosbag_converter_node", options),
  pub_ptr_string_test_{this->create_publisher<std_msgs::msg::String>(
      "/rosbag_converter_test",
      10)}
{
  auto path_in = static_cast<std::string>(
    rclcpp::Node::declare_parameter(
      "path_in_ros1_bag_file").get<std::string>());
  auto path_out = static_cast<std::string>(
    rclcpp::Node::declare_parameter(
      "path_out_ros2_bag_file").get<std::string>());

  RCLCPP_INFO_STREAM(this->get_logger(), "path_in_ros1_bag_file: " << path_in);
  RCLCPP_INFO_STREAM(this->get_logger(), "path_out_ros2_bag_file: " << path_out);

  rosbag::Bag bag_in;
  bag_in.open(path_in, rosbag::bagmode::Read);


  rosbag::View view(bag_in);

  std::vector<const rosbag::ConnectionInfo *> connection_infos = view.getConnections();
  std::set<std::string> topics;
  std::map<std::string, std::string> map_topic_names_to_types;

  for (const rosbag::ConnectionInfo * info: connection_infos) {
    bool doesnt_contain = map_topic_names_to_types.find(info->topic) ==
      map_topic_names_to_types.end();
    if (doesnt_contain) {
      map_topic_names_to_types.insert(std::make_pair(info->topic, info->datatype));
    }
  }

  std::stringstream ss_topics;
  for (const auto & pair_key_value :map_topic_names_to_types) {
    ss_topics << pair_key_value.first << " : " << pair_key_value.second << std::endl;
  }
  RCLCPP_INFO_STREAM(this->get_logger(), ss_topics.str());


//  for (rosbag::MessageInstance const & m: view) {
//    std_msgs::Int32::ConstPtr i = m.instantiate<std_msgs::Int32>();
//    if (i != nullptr) {
//      std::cout << i->data << std::endl;
//    }
//  }

  bag_in.close();


}
