#include "camera2_3/pub.hpp"

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    
    auto node = std::make_shared<ImagePublisher>();
    
    rclcpp::spin(node);
    
    rclcpp::shutdown();
    return 0;
}