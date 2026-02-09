#include "psub2_1/sub1.hpp"
#include "std_msgs/msg/int32.hpp"
#include <functional>
#include <rclcpp/logging.hpp>
#include <rclcpp/qos.hpp>

void Sub::subscribe_msg(const std_msgs::msg::Int32::SharedPtr msg) const{
    RCLCPP_INFO(this->get_logger(), "Received message : %d",msg->data);
}

Sub::Sub() : Node("mysub"){
    auto qos_profile = rclcpp::QoS(rclcpp::KeepLast(10));
    sub_ = this->create_subscription<std_msgs::msg::Int32>("mytopic", qos_profile, std::bind(&Sub::subscribe_msg, this, std::placeholders::_1));
}
