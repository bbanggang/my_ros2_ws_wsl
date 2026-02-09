#ifndef SUB_HPP_
#define SUB_HPP_

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/vector3.hpp"
#include "dxl1_1/dxl.hpp"
#include <memory>

class DxlSubscriber : public rclcpp::Node
{
public:
    DxlSubscriber();
    virtual ~DxlSubscriber();

private:
    // 토픽 수신 시 실행될 콜백 함수
    void topic_callback(const geometry_msgs::msg::Vector3::SharedPtr msg);

    // 멤버 변수
    Dxl dxl_;
    rclcpp::Subscription<geometry_msgs::msg::Vector3>::SharedPtr subscription_;
};

#endif // SUB_HPP_