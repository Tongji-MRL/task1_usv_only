#include <rclcpp/rclcpp.hpp>
#include <tf2_msgs/msg/tf_message.hpp>
#include <chrono>
#include <iomanip>
#include <iostream>
#include <map>
#include <set>
#include <sstream>
#include <thread>

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>("validation_tf_authority");
  std::map<std::string, std::set<std::string>> owners;
  auto callback = [&](tf2_msgs::msg::TFMessage::ConstSharedPtr message,
                      const rclcpp::MessageInfo & info) {
    std::ostringstream gid;
    for (auto byte : info.get_rmw_message_info().publisher_gid.data) {
      gid << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(byte);
    }
    for (const auto & transform : message->transforms) {
      owners[transform.header.frame_id + " -> " + transform.child_frame_id].insert(gid.str());
    }
  };
  auto dynamic = node->create_subscription<tf2_msgs::msg::TFMessage>("/tf", 100, callback);
  auto fixed = node->create_subscription<tf2_msgs::msg::TFMessage>(
    "/tf_static", rclcpp::QoS(100).transient_local(), callback);
  const auto end = std::chrono::steady_clock::now() + std::chrono::seconds(10);
  while (rclcpp::ok() && std::chrono::steady_clock::now() < end) {
    rclcpp::spin_some(node);
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }
  std::cout << "{\n";
  bool first = true;
  bool valid = !owners.empty();
  for (const auto & edge : owners) {
    if (!first) {std::cout << ",\n";}
    first = false;
    std::cout << "  \"" << edge.first << "\": " << edge.second.size();
    valid = valid && edge.second.size() == 1;
  }
  std::cout << "\n}\n";
  rclcpp::shutdown();
  return valid ? 0 : 1;
}
