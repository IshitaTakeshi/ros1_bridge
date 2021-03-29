#pragma once

#include <cstring>
#include <map>
#include <memory>
#include <set>
#include <string>
#include <utility>
#include <vector>

// include ROS 1
#ifdef __clang__
# pragma clang diagnostic push
# pragma clang diagnostic ignored "-Wunused-parameter"
#endif
#include "ros/ros.h"
#ifdef __clang__
# pragma clang diagnostic pop
#endif
#include "ros/this_node.h"
#include "ros/header.h"
#include "ros/service_manager.h"
#include "ros/transport/transport_tcp.h"
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <std_msgs_factories.hpp>

// include ROS 2
#include "rclcpp/rclcpp.hpp"
#include "rclcpp/scope_exit.hpp"

#include "rcutils/get_env.h"

#include "ros1_bridge/bridge.hpp"

#include "rosbag2_cpp/writer.hpp"
#include "rosbag2_cpp/writers/sequential_writer.hpp"
#include "std_msgs/msg/string.hpp"


class RosbagConverterNode: public rclcpp::Node
{
public:
  explicit RosbagConverterNode(const rclcpp::NodeOptions & options);
  rclcpp::Publisher < std_msgs::msg::String > ::SharedPtr pub_ptr_string_test_;

private:

};
