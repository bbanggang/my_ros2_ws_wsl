#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/int32.hpp"
#include "rclcpp/time_source.hpp"
#include <memory>
#include <chrono>
#include <rclcpp/logging.hpp>
#include <rclcpp/node.hpp>
#include <rclcpp/qos.hpp>
#include <rclcpp/rate.hpp>
#include <rclcpp/utilities.hpp>
using namespace std::chrono_literals;

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>("node_pub1");
  auto qos_profile = rclcpp::QoS(rclcpp::KeepLast(10));
  auto mypub = node->create_publisher<std_msgs::msg::Int32>("topic_pub1", qos_profile);
  std_msgs::msg::Int32 msg;
  static int num = 0;
  rclcpp::WallRate loop_rate(50ms);
  while(rclcpp::ok()){
    msg.data = num;
    RCLCPP_INFO(node->get_logger(), "Publish: %d",msg.data);
    mypub->publish(msg);
    num++;
    loop_rate.sleep();
  }
  rclcpp::shutdown();
  return 0;
}
