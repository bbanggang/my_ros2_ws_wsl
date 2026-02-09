#ifndef _SUM_HPP_
#define _SUM_HPP_
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include <functional>
#include <memory>
#include <rclcpp/node.hpp>
#include <rclcpp/timer.hpp>
using std::placeholders::_1;

class Sub : public rclcpp::Node{
private:
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr sub_;
    void subscribe_msg(const std_msgs::msg::String::SharedPtr msg) const;
public:
    Sub();    
};

#endif // !_SUM_HPP_
