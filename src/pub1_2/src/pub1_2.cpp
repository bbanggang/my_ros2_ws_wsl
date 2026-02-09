#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/vector3.hpp" 
#include <iostream>

class VectorPublisher : public rclcpp::Node
{
public:
  VectorPublisher() : Node("vector_publisher_node")
  {
    publisher_ = this->create_publisher<geometry_msgs::msg::Vector3>("topic_vector", 10);
  }

  void run_input_loop()
  {
    geometry_msgs::msg::Vector3 message;
    double x, y, z;

    while (rclcpp::ok())
    {
      std::cout << "x, y, z 입력: ";
      if (!(std::cin >> x >> y >> z)) {
        std::cout << "\n입력 오류 또는 종료 신호 감지. 노드를 종료합니다." << std::endl;
        break;
      }

      message.x = x;
      message.y = y;
      message.z = z;

      RCLCPP_INFO(this->get_logger(), "Publishing Vector3: x=%.2f, y=%.2f, z=%.2f", message.x, message.y, message.z);
      publisher_->publish(message);

      // ROS 2 이벤트 처리를 위해 잠시 spin 
      rclcpp::spin_some(this->get_node_base_interface());
    }
  }

private:
  rclcpp::Publisher<geometry_msgs::msg::Vector3>::SharedPtr publisher_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  
  // 노드 객체 생성
  auto node = std::make_shared<VectorPublisher>();
  
  // 입력 루프 실행 (blocking 방식)
  node->run_input_loop();

  rclcpp::shutdown();
  return 0;
}