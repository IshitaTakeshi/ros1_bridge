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

  bool print_type_correspondences_1_to_2 = static_cast<bool>(
    rclcpp::Node::declare_parameter(
      "print_type_correspondences_1_to_2").get<bool>());

  RCLCPP_INFO_STREAM(this->get_logger(), "path_in_ros1_bag_file: " << path_in);
  RCLCPP_INFO_STREAM(this->get_logger(), "path_out_ros2_bag_file: " << path_out);

  rosbag::Bag bag_in;
  bag_in.open(path_in, rosbag::bagmode::Read);


  rosbag::View view(bag_in);

  std::vector<const rosbag::ConnectionInfo *> connection_infos = view.getConnections();

  // Keys: (string) topic names
  // Values: (string) topic type names
  std::map<std::string, std::string> map_topic_names_to_types;

  // Keys: (string) topic type names
  // Values: (bool) the type has ros2 counterpart (false by default)
  std::map<std::string, bool> map_type_has_ros2_version;

  for (const rosbag::ConnectionInfo * info: connection_infos) {
    bool doesnt_contain = map_topic_names_to_types.find(info->topic) ==
      map_topic_names_to_types.end();
    if (doesnt_contain) {
      map_topic_names_to_types.insert(std::make_pair(info->topic, info->datatype));
      // Now add type name to map_type_has_ros2_version if it doesn't contain already
      bool doesnt_contain_map2 = map_type_has_ros2_version.find(info->datatype) ==
        map_type_has_ros2_version.end();
      if (doesnt_contain_map2) {
        map_type_has_ros2_version.insert(std::make_pair(info->datatype, false));
      }
    }
  }

  {
    std::stringstream ss_topics;
    for (const auto & pair_key_value :map_topic_names_to_types) {
      ss_topics << pair_key_value.first << " : " << pair_key_value.second << std::endl;
    }
    RCLCPP_INFO_STREAM(this->get_logger(), ss_topics.str());
  }

  // check if there are any mappings at all

  const auto & mappings_2to1 = ros1_bridge::get_all_message_mappings_2to1();
  if (mappings_2to1.empty()) {
    RCLCPP_ERROR_STREAM(this->get_logger(), "No message type conversion pairs supported.");
    return;
  } else if (print_type_correspondences_1_to_2) {
    std::stringstream ss_topic_mapping;
    ss_topic_mapping << "Supported ROS 2 <=> ROS 1 message type conversion pairs:" << std::endl;
    for (auto & pair : mappings_2to1) {
      ss_topic_mapping << "  - " <<
        pair.first << " (ROS 2) <=> " <<
        pair.second.c_str() << " (ROS 1)" << std::endl;
    }
    RCLCPP_INFO_STREAM(this->get_logger(), ss_topic_mapping.str());
  }


  // for each of the topic types, check if it has ros2 counterpart

  std::map<std::string, std::string> map_type_ros1_to_type_ros2;

  for (auto & pair_key_value :map_type_has_ros2_version) {
    const auto & type_name = pair_key_value.first;
    bool & ros2_counterpart_exists = pair_key_value.second;
    std::string type_name_ros2;
    ros2_counterpart_exists = ros1_bridge::get_1to2_mapping(type_name, type_name_ros2);
    if (!ros2_counterpart_exists) {
      continue;
    }
    map_type_ros1_to_type_ros2.insert(std::make_pair(pair_key_value.first, type_name_ros2));
  }
  if (map_type_ros1_to_type_ros2.empty()) {
    RCLCPP_ERROR_STREAM(
      this->get_logger(), "None of the types in the bag file are convertible to ROS2.");
  }

  {
    // Print compatible topic name pairs
    std::stringstream ss_convertible_info;
    for (const auto & pair_key_value :map_topic_names_to_types) {
      const auto & topic_name = pair_key_value.first;
      const auto & topic_type_ros1 = pair_key_value.second;
      const auto & ros2_counterpart_exists = map_type_has_ros2_version.at(topic_type_ros1);
      if (!ros2_counterpart_exists) {
        continue;
      }
      const auto & topic_type_ros2 = map_type_ros1_to_type_ros2.at(topic_type_ros1);

      ss_convertible_info <<
        topic_name << " : (ROS1) " <<
        topic_type_ros1 << " <=> " <<
        topic_type_ros2 << " (ROS2)" << std::endl;
    }
    RCLCPP_INFO_STREAM(this->get_logger(), ss_convertible_info.str());
  }

//  for (rosbag::MessageInstance const & m: view) {
//    std_msgs::Int32::ConstPtr i = m.instantiate<std_msgs::Int32>();
//    if (i != nullptr) {
//      std::cout << i->data << std::endl;
//    }
//  }

  bag_in.close();


}
