#ifndef PUB_HPP_
#define PUB_HPP_

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/vector3.hpp"
#include <termios.h>
#include <fcntl.h>
#include <memory>

class DxlPublisher : public rclcpp::Node
{
public:
    DxlPublisher();
    virtual ~DxlPublisher() = default;

private:
    // 주기적으로 실행될 타이머 콜백
    void timer_callback();

    // 키보드 입력을 위한 유틸리티 함수
    int getch();
    bool kbhit();

    // ROS 관련 멤버 변수
    rclcpp::Publisher<geometry_msgs::msg::Vector3>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;

    // 속도 및 제어 변수
    int vel1_, vel2_;
    int goal1_, goal2_;
};

#endif // PUB_HPP_