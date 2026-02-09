#include "dxl1_1/sub.hpp"

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    
    // DxlSubscriber 노드 생성 및 실행
    auto node = std::make_shared<DxlSubscriber>();
    
    try {
        rclcpp::spin(node);
    } catch (const std::exception &e) {
        RCLCPP_ERROR(node->get_logger(), "Exception in spin: %s", e.what());
    }

    rclcpp::shutdown();
    return 0;
}