#include "ros1_bridge/rosbag_converter_node.h"
#include "rclcpp/logger.hpp"
#include <map>
#include <vector>
#include <sstream>
#include "ros1_bridge/factory_interface.hpp"
#include "sensor_msgs_factories.hpp"

#include <rosbag/bag.h>
#include <rosbag/view.h>

#include "rclcpp/serialization.hpp"
#include "rclcpp/serialized_message.hpp"
#include "rcpputils/filesystem_helper.hpp"
#include "ros1_bridge/proto_rosbag2.pb.h"
#include <iostream>
#include <fstream>

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
      "path_out_ros2_serialized_binary").get<std::string>());
  bool print_type_correspondences_1_to_2 = static_cast<bool>(
    rclcpp::Node::declare_parameter(
      "print_type_correspondences_1_to_2").get<bool>());

  std::stringstream ss_input_args;
  ss_input_args << "Input args: " << std::endl;
  ss_input_args << "path_in_ros1_bag_file: " << path_in << std::endl;
  ss_input_args << "path_out_ros2_serialized_binary: " << path_out << std::endl;
  ss_input_args << "print_type_correspondences_1_to_2: " << print_type_correspondences_1_to_2 <<
    std::endl;
  ss_input_args << std::endl;
  RCLCPP_INFO_STREAM(this->get_logger(), ss_input_args.str());

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
    ss_topics << "Topic name-type pairs in the bag file:" << std::endl;
    for (const auto & pair_key_value :map_topic_names_to_types) {
      ss_topics << pair_key_value.first << " : " << pair_key_value.second << std::endl;
    }
    ss_topics << std::endl;
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
    ss_topic_mapping << std::endl;
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
    ss_convertible_info << "Convertible topic name/type mapping information: " << std::endl;
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

  // Now we know supported topic names, their ROS1 types and ROS2 counterpart type names.

  // Let's create the factory instances for these topics.

  // Key: topic name
  // Value: factory instance
  using FactoryPtr = std::shared_ptr<ros1_bridge::FactoryInterface>;
  std::map<std::string, FactoryPtr> map_topic_name_to_factory;

  for (const auto & pair_key_value :map_topic_names_to_types) {
    const auto & topic_name = pair_key_value.first;
    const auto & topic_type_ros1 = pair_key_value.second;
    const auto & ros2_counterpart_exists = map_type_has_ros2_version.at(topic_type_ros1);
    if (!ros2_counterpart_exists) {
      continue;
    }
    const auto & topic_type_ros2 = map_type_ros1_to_type_ros2.at(topic_type_ros1);

    FactoryPtr factory = ros1_bridge::get_factory(topic_type_ros1, topic_type_ros2);
    map_topic_name_to_factory.insert(std::make_pair(topic_name, factory));
  }


  GOOGLE_PROTOBUF_VERIFY_VERSION;
  rosbag_converter_proto::ProtoRosBag2 proto_rosbag2;
  for (rosbag::MessageInstance const & m: view) {

    const auto & topic_name = m.getTopic();
    const auto & topic_type_ros1 = map_topic_names_to_types.at(topic_name);
    const auto & ros2_counterpart_exists = map_type_has_ros2_version.at(topic_type_ros1);
    if (!ros2_counterpart_exists) {
      continue;
    }
    const auto & topic_type_ros2 = map_type_ros1_to_type_ros2.at(topic_type_ros1);
    FactoryPtr & factory = map_topic_name_to_factory.at(topic_name);

    if (topic_type_ros1 == "sensor_msgs/PointCloud2") {
      sensor_msgs::PointCloud2::ConstPtr cloud_r1 = m.instantiate<sensor_msgs::PointCloud2>();
      sensor_msgs::msg::PointCloud2 cloud_r2;
      if (cloud_r1 == nullptr) {
        continue;
      }
      factory->convert_1_to_2(cloud_r1.get(), &cloud_r2);
      rclcpp::SerializedMessage serialized_msg;
      rclcpp::Serialization<sensor_msgs::msg::PointCloud2> serialization;
      serialization.serialize_message(&cloud_r2, &serialized_msg);

      auto proto_message_ptr = proto_rosbag2.add_messages();
      proto_message_ptr->set_topic_name(topic_name);
      proto_message_ptr->set_topic_type_name(topic_type_ros2);
      proto_message_ptr->set_time_stamp(m.getTime().toNSec());
      proto_message_ptr->set_serialized_data(
        reinterpret_cast<char *>(serialized_msg.
        get_rcl_serialized_message().buffer), serialized_msg.size());

    }

  }

  bag_in.close();

  std::fstream output(path_out, std::ios::out | std::ios::trunc | std::ios::binary);
  if (!proto_rosbag2.SerializeToOstream(&output)) {
    RCLCPP_ERROR_STREAM(this->get_logger(), "Failed to write serialized bag to disk.");
  }


}
