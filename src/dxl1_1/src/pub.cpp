#include "dxl1_1/pub.hpp"

DxlPublisher::DxlPublisher() 
: Node("node_dxlpub"), vel1_(0), vel2_(0), goal1_(0), goal2_(0)
{
    auto qos_profile = rclcpp::QoS(rclcpp::KeepLast(10));
    publisher_ = this->create_publisher<geometry_msgs::msg::Vector3>("topic_dxlpub", qos_profile);

    // 20Hz 주기로 타이머 설정 (50ms 마다 실행)
    timer_ = this->create_wall_timer(
        std::chrono::milliseconds(50),
        std::bind(&DxlPublisher::timer_callback, this)
    );

    RCLCPP_INFO(this->get_logger(), "키보드 제어 노드가 시작되었습니다.");
    RCLCPP_INFO(this->get_logger(), "조작법: f(앞), b(뒤), l(왼쪽), r(오른쪽), s/Space(정지)");
}

void DxlPublisher::timer_callback()
{
    // 1. 키보드 입력 체크
    if (kbhit()) {
        char c = getch();
        switch (c) {
            case 's': case ' ': goal1_ = 0;   goal2_ = 0;   break;
            case 'f':           goal1_ = 50;  goal2_ = -50; break;
            case 'b':           goal1_ = -50; goal2_ = 50;  break;
            case 'l':           goal1_ = -50; goal2_ = -50; break;
            case 'r':           goal1_ = 50;  goal2_ = 50;  break;
            default:            goal1_ = 0;   goal2_ = 0;   break;
        }
    }

    // 2. 가감속 모션 생성 (Generation of accel/decel)
    if (goal1_ > vel1_) vel1_ += 5;
    else if (goal1_ < vel1_) vel1_ -= 5;
    else vel1_ = goal1_;

    if (goal2_ > vel2_) vel2_ += 5;
    else if (goal2_ < vel2_) vel2_ -= 5;
    else vel2_ = goal2_;

    // 3. 메시지 발행
    geometry_msgs::msg::Vector3 vel_msg;
    vel_msg.x = static_cast<double>(vel1_);
    vel_msg.y = static_cast<double>(vel2_);
    vel_msg.z = 0.0;

    RCLCPP_INFO(this->get_logger(), "Publishing: x=%.2f, y=%.2f", vel_msg.x, vel_msg.y);
    publisher_->publish(vel_msg);
}

// --- 터미널 I/O 유틸리티 구현 ---
int DxlPublisher::getch()
{
    struct termios oldt, newt;
    int ch;
    tcgetattr(STDIN_FILENO, &oldt);
    newt = oldt;
    newt.c_lflag &= ~(ICANON | ECHO);
    tcsetattr(STDIN_FILENO, TCSANOW, &newt);
    ch = getchar();
    tcsetattr(STDIN_FILENO, TCSANOW, &oldt);
    return ch;
}

bool DxlPublisher::kbhit()
{
    struct termios oldt, newt;
    int ch;
    int oldf;
    tcgetattr(STDIN_FILENO, &oldt);
    newt = oldt;
    newt.c_lflag &= ~(ICANON | ECHO);
    tcsetattr(STDIN_FILENO, TCSANOW, &newt);
    oldf = fcntl(STDIN_FILENO, F_GETFL, 0);
    fcntl(STDIN_FILENO, F_SETFL, oldf | O_NONBLOCK);
    ch = getchar();
    tcsetattr(STDIN_FILENO, TCSANOW, &oldt);
    fcntl(STDIN_FILENO, F_SETFL, oldf);
    if (ch != EOF) {
        ungetc(ch, stdin);
        return true;
    }
    return false;
}