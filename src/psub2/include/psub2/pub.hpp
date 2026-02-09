#ifndef _PUB_HPP_
#define _PUB_HPP_
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include <cstddef>
#include <memory>
#include <chrono>
#include <rclcpp/node.hpp>
#include <rclcpp/publisher.hpp>
#include <rclcpp/timer.hpp>
using namespace std::chrono_literals;
class Pub : public rclcpp::Node{
private:
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_;
    size_t count_;
    void publish_msg();
public:
    Pub();    
};

#endif // !_PUB_HPP_
