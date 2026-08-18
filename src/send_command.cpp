// rsf_node2 - sends one command number to the running driver node.
//
// SPDX-License-Identifier: Apache-2.0
//
//   ros2 run rsf_node2 send_command 1
//
// The command numbers are those of table 3-2 in C-42-04636:
//   1 START_STREAMING, 2 STOP_STREAMING, 3 START_RSF, 4 STOP_RSF, 5 RESET_RSF
//
// The topic is latched (transient local), so the driver picks the command up
// even if it subscribes slightly later.

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/u_int8.hpp>

#include <chrono>
#include <cstdio>
#include <cstdlib>
#include <string>
#include <vector>

#include "rsf/protocol.hpp"

namespace {

void printUsage(const char* program) {
  std::printf("Usage: %s <command number> [--topic TOPIC]\n\n", program);
  for (std::uint8_t value = 1; value <= 5; ++value) {
    std::printf("  %u  %s\n", static_cast<unsigned>(value),
                rsf::toString(static_cast<rsf::CommandType>(value)));
  }
}

}  // namespace

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);

  const std::vector<std::string> arguments = rclcpp::remove_ros_arguments(argc, argv);
  if (arguments.size() < 2) {
    printUsage(argv[0]);
    rclcpp::shutdown();
    return 1;
  }

  const int value = std::atoi(arguments[1].c_str());
  if (value < static_cast<int>(rsf::CommandType::kStartStreaming) ||
      value > static_cast<int>(rsf::CommandType::kResetRsf)) {
    std::fprintf(stderr, "command number must be in the range 1..5\n\n");
    printUsage(argv[0]);
    rclcpp::shutdown();
    return 1;
  }

  auto node = std::make_shared<rclcpp::Node>("rsf_send_command");
  const std::string topic =
      node->declare_parameter<std::string>("topic", "/rsf/cmd_to_spel");

  rclcpp::QoS qos(rclcpp::KeepLast(10));
  qos.reliable();
  qos.transient_local();
  auto publisher = node->create_publisher<std_msgs::msg::UInt8>(topic, qos);

  // Give the discovery a moment to match the driver's subscription.
  rclcpp::sleep_for(std::chrono::milliseconds(200));

  std_msgs::msg::UInt8 message;
  message.data = static_cast<std::uint8_t>(value);
  publisher->publish(message);
  rclcpp::spin_some(node);
  rclcpp::sleep_for(std::chrono::milliseconds(200));

  RCLCPP_INFO(node->get_logger(), "sent %s (%u) on %s",
              rsf::toString(static_cast<rsf::CommandType>(value)),
              static_cast<unsigned>(message.data), topic.c_str());

  rclcpp::shutdown();
  return 0;
}
