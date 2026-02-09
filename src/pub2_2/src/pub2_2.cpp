#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/vector3.hpp" 
#include <iostream>
#include <functional>
#include <rclcpp/executors.hpp>
#include <rclcpp/logging.hpp>
#include <rclcpp/node.hpp>
#include <rclcpp/publisher.hpp>
#include <rclcpp/qos.hpp>
#include <rclcpp/utilities.hpp>
using namespace std::chrono_literals;


void input_callback(rclcpp::Node::SharedPtr node,
rclcpp::Publisher<geometry_msgs::msg::Vector3>::SharedPtr mypub){
  double x, y, z;
  geometry_msgs::msg::Vector3 messages;

  std::cout << "-----3개의 실수를 입력하세요-----" << std::endl;

  if(std::cin >> x >> y >> z){
    messages.x = x;
    messages.y = y;
    messages.z = z;

    RCLCPP_INFO(node->get_logger(), "Publishing : ( x : %.3f, y : %.3f, z : %.3f )", messages.x, messages.y, messages.z);
    mypub->publish(messages);
  }
  else
  {
    RCLCPP_WARN(node->get_logger(), "입력 또는 오류 신호 발생으로 종료합니다.");
    rclcpp::shutdown();
  }
  
}
int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  
  // 노드 객체 생성
  auto node = std::make_shared<rclcpp::Node>("vector_publisher_node");
  auto qos_profile = rclcpp::QoS(rclcpp::KeepLast(10));
  auto pub = node->create_publisher<geometry_msgs::msg::Vector3>("vector_topic", qos_profile);
  
  std::function<void()> fn = std::bind(input_callback, node, pub);

  auto timer = node->create_wall_timer(10ms, fn);

  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}