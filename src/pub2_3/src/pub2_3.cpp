#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include <functional>
#include <memory>
#include <iostream>
#include <rcl/publisher.h>
#include <rclcpp/create_timer.hpp>
#include <rclcpp/executors.hpp>
#include <rclcpp/logging.hpp>
#include <rclcpp/node.hpp>
#include <rclcpp/publisher.hpp>
#include <rclcpp/qos.hpp>
#include <rclcpp/utilities.hpp>
using namespace std::chrono_literals;

void callback(rclcpp::Node::SharedPtr node,
rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr mypub){
  char input_char;
  geometry_msgs::msg::Twist msg;

  std::cout << "-----거북이를 조종하세요!-----" << std::endl;
  std::cout << "f : 전진, b : 후진, l : 좌회전, r : 우회전" << std::endl;
  
  while(rclcpp::ok()){
    std::cout << "명령 입력 + enter" << std::endl;
    std::cin >> input_char;

    if(input_char == 'q'){
      std::cout << "노드 종료!" << std::endl;
      break;
    }

    msg.angular.z = 0.0;
    msg.linear.x = 0.0;
    
    switch(input_char){
      case 'f':
        msg.linear.x = 2.0;
        break;
      case 'b':
        msg.linear.x = -2.0;
        break;
      case 'r':
        msg.angular.z = -2.0;
        break;
      case 'l':
        msg.angular.z = 2.0;
        break;
      case 's': //정지
        break;
      default:
        std::cout << "알 수 없는 명령어입니다. 다시 입력하세요" << std::endl;
        continue;
    }
    RCLCPP_INFO(node->get_logger(), "Send : Linear=%.1f, Angular=%.1f",msg.linear.x, msg.angular.z);
    mypub->publish(msg);

  }
}

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);

  auto node = std::make_shared<rclcpp::Node>("twist_publisher_node");
  auto qos_profile = rclcpp::QoS(rclcpp::KeepLast(10));
  auto pub = node->create_publisher<geometry_msgs::msg::Twist>("turtle1/cmd_vel", qos_profile);

  std::function<void()> fn = std::bind(callback, node, pub);
  auto timer = node->create_wall_timer(10ms, fn);

  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
