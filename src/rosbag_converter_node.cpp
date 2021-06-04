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
    rclcpp::Node::declare_parameter("path_in_ros1_bag_file").get<std::string>());
  auto path_out = static_cast<std::string>(
    rclcpp::Node::declare_parameter("path_out_ros2_serialized_binary").get<std::string>());
  bool print_type_correspondences_1_to_2 = static_cast<bool>(
    rclcpp::Node::declare_parameter("print_type_correspondences_1_to_2").get<bool>());

  std::stringstream ss_input_args;
  ss_input_args << "Input args: " << std::endl;
  ss_input_args << "path_in_ros1_bag_file: " << path_in << std::endl;
  ss_input_args << "path_out_ros2_serialized_binary: " << path_out << std::endl;
  ss_input_args << "print_type_correspondences_1_to_2: "
                << print_type_correspondences_1_to_2 << std::endl;
  ss_input_args << std::endl;
  RCLCPP_INFO_STREAM(this->get_logger(), ss_input_args.str());

  rosbag::Bag bag_in;
  bag_in.open(path_in, rosbag::bagmode::Read);

  rosbag::View view(bag_in);

  std::vector<const rosbag::ConnectionInfo *> connection_infos = view.getConnections();

  // Keys: (string) topic names
  // Values: (string) topic type names
  std::map<std::string, std::string> topic_name_type_map;

  // Keys: (string) topic type names
  // Values: (bool) the type has ros2 counterpart (false by default)
  std::map<std::string, bool> map_type_has_ros2_version;

  for (const rosbag::ConnectionInfo * info : connection_infos) {
    bool doesnt_contain = topic_name_type_map.find(info->topic) == topic_name_type_map.end();
    if (doesnt_contain) {
      topic_name_type_map.insert(std::make_pair(info->topic, info->datatype));
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
    for (const auto & pair_key_value : topic_name_type_map) {
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

  for (auto & pair_key_value : map_type_has_ros2_version) {
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
    for (const auto & pair_key_value : topic_name_type_map) {
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

  for (const auto & pair_key_value : topic_name_type_map) {
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

  // Simple x^n (integer^integer) exponent function
  auto pow_ul = [](uint64_t x, uint32_t n)
    {
      uint64_t y = 1UL;
      for (uint32_t i = 0; i < n; i++) {
        y *= x;
      }
      return y;
    };

  const size_t proto_max_file_size_bytes = pow_ul(10, 9);
  int count_written_proto_files = 0;

  GOOGLE_PROTOBUF_VERIFY_VERSION;
  rosbag_converter_proto::ProtoRosBag2 proto_rosbag2;

  auto write_to_rosbag2 =
    [&count_written_proto_files,
      &proto_rosbag2,
      path_out,
      this]() {
      std::string str_count_raw = std::to_string(count_written_proto_files);
      std::string str_count_with_leading_zeros = "_" +
        std::string(3 - str_count_raw.length(), '0') + str_count_raw;
      std::string str_file_name = path_out + str_count_with_leading_zeros;
      std::fstream output(str_file_name,
        std::ios::out | std::ios::trunc | std::ios::binary);
      if (!proto_rosbag2.SerializeToOstream(&output)) {
        RCLCPP_ERROR_STREAM(this->get_logger(), "Failed to write serialized bag to disk.");
        return false;
      }
      RCLCPP_INFO_STREAM(this->get_logger(), "File written successfully: " + str_file_name);
      count_written_proto_files++;
      proto_rosbag2.Clear();
      return true;
    };

  for (rosbag::MessageInstance const & m : view) {
    const auto & topic_name = m.getTopic();
    const auto & topic_type_ros1 = topic_name_type_map.at(topic_name);
    const auto & ros2_counterpart_exists = map_type_has_ros2_version.at(topic_type_ros1);
    if (!ros2_counterpart_exists) {
      continue;
    }
    const auto & topic_type_ros2 = map_type_ros1_to_type_ros2.at(topic_type_ros1);
    FactoryPtr & factory = map_topic_name_to_factory.at(topic_name);
    rclcpp::SerializedMessage serialized_msg;
    bool conversion_is_successful = factory->ros1_message_instance_to_ros2_serialized_message(
      m,
      serialized_msg);

    if (!conversion_is_successful) {
      RCLCPP_ERROR_STREAM(
        this->get_logger(),
        "Type conversion failed: " << topic_type_ros1 << " to " << topic_type_ros2);
      continue;
    }

    auto proto_message_ptr = proto_rosbag2.add_messages();
    proto_message_ptr->set_topic_name(topic_name);
    proto_message_ptr->set_topic_type_name(topic_type_ros2);
    proto_message_ptr->set_time_stamp(m.getTime().toNSec());
    proto_message_ptr->set_serialized_data(
      reinterpret_cast<char *>(serialized_msg.
      get_rcl_serialized_message().buffer), serialized_msg.size());

    if (proto_rosbag2.ByteSizeLong() > proto_max_file_size_bytes) {
      // Write every 1GB
      // Name them as file_name.proto_rosbag2_000, file_name.proto_rosbag2_001, ...
      if (!write_to_rosbag2()) {
        return;
      }
    }
  }

  if (proto_rosbag2.ByteSizeLong() > 0) {
    // Write residual <1GB part if exists
    // Name them as file_name.proto_rosbag2_000, file_name.proto_rosbag2_001, ...
    if (!write_to_rosbag2()) {
      return;
    }
  }

  bag_in.close();
}
