#include "dxl1_1/sub.hpp"

DxlSubscriber::DxlSubscriber() : Node("node_dxlsub")
{
    // Dynamixel 포트 열기
    if (!dxl_.open()) {
        RCLCPP_ERROR(this->get_logger(), "Dynamixel open error! Check connection.");
        // 노드 실행 중 치명적 오류 발생 시 종료 유도 (필요에 따라 선택)
        // rclcpp::shutdown(); 
    } else {
        RCLCPP_INFO(this->get_logger(), "Dynamixel port opened successfully.");
    }

    // QoS 설정
    auto qos_profile = rclcpp::QoS(rclcpp::KeepLast(10));

    // 구독자 생성: "topic_dxlpub" 토픽을 구독
    subscription_ = this->create_subscription<geometry_msgs::msg::Vector3>(
        "topic_dxlpub",
        qos_profile,
        std::bind(&DxlSubscriber::topic_callback, this, std::placeholders::_1)
    );
}

DxlSubscriber::~DxlSubscriber()
{
    // 소멸자에서 장치 닫기
    dxl_.close();
    RCLCPP_INFO(this->get_logger(), "Dynamixel port closed.");
}

void DxlSubscriber::topic_callback(const geometry_msgs::msg::Vector3::SharedPtr msg)
{
    // 수신 데이터 로그 출력
    RCLCPP_INFO(this->get_logger(), "Received message: x=%lf, y=%lf", msg->x, msg->y);

    // Dynamixel 속도 설정 (x: 왼쪽 모터, y: 오른쪽 모터 등으로 매핑)
    dxl_.setVelocity((int)msg->x, (int)msg->y);
}