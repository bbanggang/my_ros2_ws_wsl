#include "psub2_2/pub2.hpp"
#include "geometry_msgs/msg/vector3.hpp"
#include <functional>
#include <rclcpp/logging.hpp>
#include <rclcpp/qos.hpp>

Pub::Pub() : Node("mypub"), x(0), y(0), z(0), count_(0){
  auto qos = rclcpp::QoS(rclcpp::KeepLast(10));
  pub_ = this->create_publisher<geometry_msgs::msg::Vector3>("mytopic", qos);
  timer_ = this->create_wall_timer(1s, std::bind(&Pub::publish_msg, this));
}

void Pub::publish_msg(){
  auto msg = geometry_msgs::msg::Vector3();
  msg.x = x; msg.y = y; msg.z = z;
  RCLCPP_INFO(this->get_logger(), "Publisted message: x->%.2f, y->%.2f, z->%.2f", msg.x, msg.y, msg.z);
  pub_->publish(msg);
  x++, y++, z++;
}