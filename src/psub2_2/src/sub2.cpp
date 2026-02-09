#include "psub2_2/sub2.hpp"
#include <functional>
#include <rclcpp/logging.hpp>
#include <rclcpp/qos.hpp>

void Sub::subscribe_msg(const geometry_msgs::msg::Vector3::SharedPtr msg) const{
    RCLCPP_INFO(this->get_logger(), "Received message :  x->%.2f, y->%.2f, z->%.2f",msg->x, msg->y, msg->z);
}
Sub::Sub() : Node("mysub"){
    auto qos_profile = rclcpp::QoS(rclcpp::KeepLast(10));
    sub_ = this->create_subscription<geometry_msgs::msg::Vector3>("mytopic", qos_profile, std::bind(&Sub::subscribe_msg, this, std::placeholders::_1));
}
