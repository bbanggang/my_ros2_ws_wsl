#include "psub2_1/pub1.hpp"
#include "std_msgs/msg/int32.hpp"
#include <functional>
#include <rclcpp/logging.hpp>
#include <rclcpp/qos.hpp>

Pub::Pub() : Node("mypub"), num(0), count_(0){
  auto qos = rclcpp::QoS(rclcpp::KeepLast(10));
  pub_ = this->create_publisher<std_msgs::msg::Int32>("mytopic", qos);
  timer_ = this->create_wall_timer(50ms, std::bind(&Pub::publish_msg, this));
}

void Pub::publish_msg(){
  auto msg = std_msgs::msg::Int32();
  msg.data = this->num;
  RCLCPP_INFO(this->get_logger(), "Publisted message: %d",msg.data);
  pub_->publish(msg);
  num++;
}