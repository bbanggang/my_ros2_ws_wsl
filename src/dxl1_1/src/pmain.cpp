#include "dxl1_1/pub.hpp"

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    
    // 노드 인스턴스화 및 실행
    auto node = std::make_shared<DxlPublisher>();
    
    try {
        rclcpp::spin(node);
    } catch (const std::exception &e) {
        RCLCPP_ERROR(node->get_logger(), "노드 중단: %s", e.what());
    }

    rclcpp::shutdown();
    return 0;
}