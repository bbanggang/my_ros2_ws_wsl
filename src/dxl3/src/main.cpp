#include "rclcpp/rclcpp.hpp"
#include "dxl3/dxl.hpp"
#include <memory>
#include <chrono>
#include <iostream>

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<rclcpp::Node>("dxl_auto_test");

    // [중요] 0.1초마다 실행하기 위해 주파수를 10Hz로 설정 (1초 / 10 = 0.1초)
    rclcpp::WallRate loop_rate(10.0);

    Dxl mx;
    
    // 초기 속도 및 증감량 변수 설정
    int current_vel = 0; // 현재 속도
    int step = 10;       // 변화 속도 (1rpm)

    if(!mx.open())
    {
        RCLCPP_ERROR(node->get_logger(), "dynamixel open error");
        rclcpp::shutdown();
        return -1;
    }

    RCLCPP_INFO(node->get_logger(), "Start Auto Velocity Pattern: 0 -> 100 -> -100 -> 0 ...");

    while(rclcpp::ok())
    {
        // 1. 속도 변경 로직 (삼각파 패턴)
        current_vel += step;

        // 2. 한계 도달 시 방향 전환
        if (current_vel >= 100) 
        {
            current_vel = 100; // 100을 넘지 않도록 고정
            step = -10;        // 감소 모드로 전환
            RCLCPP_INFO(node->get_logger(), "Reached Max (100). Decreasing...");
        }
        else if (current_vel <= -100) 
        {
            current_vel = -100; // -100을 넘지 않도록 고정
            step = 10;          // 증가 모드로 전환
            RCLCPP_INFO(node->get_logger(), "Reached Min (-100). Increasing...");
        }

        // 3. 모터에 속도 명령 전달
        mx.setVelocity(current_vel, -current_vel);

        // 현재 상태 출력
        RCLCPP_INFO(node->get_logger(), "Vel: %d (Step: %d)", current_vel, step);
        
        // 0.1초 대기
        loop_rate.sleep();
    }

    mx.close();
    rclcpp::shutdown();
    return 0;
}