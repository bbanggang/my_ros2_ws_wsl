#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/int32.hpp"
#include <memory>
#include <chrono>
#include <rclcpp/executors.hpp>
#include <rclcpp/logging.hpp>
#include <rclcpp/rate.hpp>
#include <rclcpp/utilities.hpp>
using namespace std::chrono_literals;

int main(int argc, char * argv[]){
    rclcpp::init(argc, argv);
    auto node = std::make_shared<rclcpp::Node>("node_pub1_1");
    auto qos_profile = rclcpp::QoS(rclcpp::KeepLast(10));
    auto mypub = node->create_publisher<std_msgs::msg::Int32>("topic_pub1_1", qos_profile);
    std_msgs::msg::Int32 num;
    num.data = 0;
    rclcpp::WallRate loop_rate(1s); //반복 주파수를 저장하는 객체로 단위는 Hz
    while(rclcpp::ok()){ // 종료 조건이 발생하면 false 리턴 아니면 true 리턴
        RCLCPP_INFO(node->get_logger(), "Publish: %d",num.data); //로그메시지를 출력하는 매크로 함수
        mypub->publish(num); //msg에 저장된 메시지를 publish
        num.data++;
        rclcpp::spin_some(node); //발생한 이벤트 없으면 바로 리턴
        loop_rate.sleep(); // 반복 주파수에서 남은 시간 만큼 sleep
    }
    rclcpp::shutdown(); 
    return 0;
}