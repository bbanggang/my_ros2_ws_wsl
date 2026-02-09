#include "camera2_3/sub.hpp"

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    
    // ImageSubscriber 노드 실행
    auto node = std::make_shared<ImageSubscriber>();
    
    rclcpp::spin(node);
    
    rclcpp::shutdown();
    return 0;
}