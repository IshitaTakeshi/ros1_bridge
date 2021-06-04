#include "ros1_bridge/rosbag_converter_node.h"
#include "rclcpp/logger.hpp"
#include <cmath>
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

std::string makeProtobufPath(const std::string& path_out, const int count_written_proto_files) {
  std::string str_count_raw = std::to_string(count_written_proto_files);
  std::string str_count_with_leading_zeros = "_" + std::string(3 - str_count_raw.length(), '0') + str_count_raw;
  return path_out + str_count_with_leading_zeros;
}

char* messageToData(const rclcpp::SerializedMessage& serialized_msg) {
  return reinterpret_cast<char *>(
      serialized_msg.get_rcl_serialized_message().buffer
  );
}

void addProtobufMessage(rosbag_converter_proto::ProtoRosBag2& proto_rosbag2,
                        const std::string& topic_name,
                        const std::string& ros2_type,
                        const double time_second,
                        const rclcpp::SerializedMessage& message) {
  auto proto_message_ptr = proto_rosbag2.add_messages();
  proto_message_ptr->set_topic_name(topic_name);
  proto_message_ptr->set_topic_type_name(ros2_type);
  proto_message_ptr->set_time_stamp(time_second);
  proto_message_ptr->set_serialized_data(messageToData(message), message.size());
}

using Correspondence = std::multimap<std::basic_string<char>,
                                     std::basic_string<char>>;

void printTypeCorrespondences(const rclcpp::Logger& logger,
                              const Correspondence& mappings_2to1) {
  std::stringstream ss_topic_mapping;
  ss_topic_mapping << "Supported ROS 2 <=> ROS 1 message type conversion pairs:" << std::endl;
  for (auto & [ros1_type, ros2_type] : mappings_2to1) {
    ss_topic_mapping << "  - "
        << ros1_type << " (ROS 1) <=> "
        << ros2_type << " (ROS 2)" << std::endl;
  }
  ss_topic_mapping << std::endl;
  RCLCPP_INFO_STREAM(logger, ss_topic_mapping.str());
}

RosbagConverterNode::RosbagConverterNode(const rclcpp::NodeOptions & options)
: Node("rosbag_converter_node", options),
  pub_ptr_string_test_{this->create_publisher<std_msgs::msg::String>(
      "/rosbag_converter_test",
      10)}
{

  const std::string path_in_arg = "path_in_ros1_bag_file";
  const std::string path_out_arg = "path_out_ros2_serialized_binary";
  const std::string path_in = rclcpp::Node::declare_parameter(path_in_arg).get<std::string>();
  const std::string path_out = rclcpp::Node::declare_parameter(path_out_arg).get<std::string>();

  std::stringstream ss_input_args;
  ss_input_args << "Input args: " << std::endl;
  ss_input_args << "path_in_ros1_bag_file: " << path_in << std::endl;
  ss_input_args << "path_out_ros2_serialized_binary: " << path_out << std::endl;
  ss_input_args << std::endl;
  RCLCPP_INFO_STREAM(this->get_logger(), ss_input_args.str());

  const auto & mappings_2to1 = ros1_bridge::get_all_message_mappings_2to1();
  if (mappings_2to1.empty()) {
    RCLCPP_ERROR_STREAM(this->get_logger(), "No message type conversion pairs supported.");
    return;
  }

  printTypeCorrespondences(this->get_logger(), mappings_2to1);

  rosbag::Bag bag_in;
  bag_in.open(path_in, rosbag::bagmode::Read);

  rosbag::View view(bag_in);

  using FactoryPtr = std::shared_ptr<ros1_bridge::FactoryInterface>;

  std::map<std::string, std::string> topic_name_to_type;
  std::map<std::string, std::string> topic_name_to_ros2_type;
  std::map<std::string, FactoryPtr> topic_name_to_factory;

  for (const rosbag::ConnectionInfo * info : view.getConnections()) {
    const std::string topic_name = info->topic;
    if (hasKey(topic_name_to_type, topic_name)) {
      continue;
    }
    const std::string ros1_type = info->datatype;
    topic_name_to_type[topic_name] = ros1_type;

    std::string ros2_type;
    if (!ros1_bridge::get_1to2_mapping(ros1_type, ros2_type)) {
      continue;
    }
    topic_name_to_ros2_type[topic_name] = ros2_type;
    topic_name_to_factory[topic_name] = ros1_bridge::get_factory(ros1_type, ros2_type);
  }

  if (topic_name_to_factory.empty()) {
    RCLCPP_ERROR_STREAM(this->get_logger(),
                        "None of the types in the bag file are convertible to ROS2.");
  }

  int count_written_proto_files = 0;

  GOOGLE_PROTOBUF_VERIFY_VERSION;
  rosbag_converter_proto::ProtoRosBag2 proto_rosbag2;

  auto write_to_rosbag2 = [&count_written_proto_files, &proto_rosbag2, path_out, this]() {
    const std::string path = makeProtobufPath(path_out, count_written_proto_files);
    std::fstream output(path, std::ios::out | std::ios::trunc | std::ios::binary);
    if (!proto_rosbag2.SerializeToOstream(&output)) {
      RCLCPP_ERROR_STREAM(this->get_logger(), "Failed to write serialized bag to disk.");
      return false;
    }
    RCLCPP_INFO_STREAM(this->get_logger(), "File written successfully: " + path);
    count_written_proto_files++;
    proto_rosbag2.Clear();
    return true;
  };

  for (rosbag::MessageInstance const & m : view) {
    const auto& topic_name = m.getTopic();
    if (!hasKey(topic_name_to_factory, topic_name)) {
      continue;
    }
    FactoryPtr& factory = topic_name_to_factory.at(topic_name);
    rclcpp::SerializedMessage serialized_msg;
    bool is_converted = factory->ros1_message_instance_to_ros2_serialized_message(
      m, serialized_msg);

    if (!is_converted) {
      RCLCPP_ERROR_STREAM(this->get_logger(),
                          "Type conversion failed: " << topic_name);
      continue;
    }

    const auto& ros2_type = topic_name_to_ros2_type.at(topic_name);
    addProtobufMessage(proto_rosbag2, topic_name, ros2_type,
                       m.getTime().toNSec(), serialized_msg);

    if (proto_rosbag2.ByteSizeLong() > std::pow(10, 9)) {
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
