#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/compressed_image.hpp"
#include "geometry_msgs/msg/vector3.hpp" // 속도 명령용
#include "std_msgs/msg/float64.hpp"
#include "cv_bridge/cv_bridge.hpp"
#include "opencv2/opencv.hpp"
#include <cmath>
#include <chrono>
#include <termios.h>
#include <fcntl.h>

class LineTrackerProcessor : public rclcpp::Node {
public:
    LineTrackerProcessor() : Node("line_tracker_node"), mode_(false), k_(1), base_vel_(100) {
        auto qos_profile = rclcpp::QoS(rclcpp::KeepLast(10)).best_effort();

        raw_sub_ = this->create_subscription<sensor_msgs::msg::CompressedImage>(
            "Image_Topic", qos_profile,
            std::bind(&LineTrackerProcessor::image_callback, this, std::placeholders::_1));

        // Dynamixel 제어 노드로 보낼 속도 명령 퍼블리셔
        vel_pub_ = this->create_publisher<geometry_msgs::msg::Vector3>("vel_cmd_topic", qos_profile);

        last_line_x_ = 320.0;
        last_line_y_ = 45.0;
        
        RCLCPP_INFO(this->get_logger(), "WSL 분석 및 제어 노드 시작. 's': 시작, 'q': 정지");
    }

private:
    // --- 키보드 입력 유틸리티 ---
    int getch() {
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

    bool kbhit() {
        struct termios oldt, newt;
        int ch, oldf;
        tcgetattr(STDIN_FILENO, &oldt);
        newt = oldt;
        newt.c_lflag &= ~(ICANON | ECHO);
        tcsetattr(STDIN_FILENO, TCSANOW, &newt);
        oldf = fcntl(STDIN_FILENO, F_GETFL, 0);
        fcntl(STDIN_FILENO, F_SETFL, oldf | O_NONBLOCK);
        ch = getchar();
        tcsetattr(STDIN_FILENO, TCSANOW, &oldt);
        fcntl(STDIN_FILENO, F_SETFL, oldf);
        if (ch != EOF) { ungetc(ch, stdin); return true; }
        return false;
    }

    void image_callback(const sensor_msgs::msg::CompressedImage::SharedPtr msg) {
        auto startTime = std::chrono::steady_clock::now(); // 처리 시간 측정 시작
        
        // 1. 키보드 입력 체크 (모드 전환)
        if (kbhit()) {
            char ch = getch();
            if (ch == 'q') {
                mode_ = false;
                RCLCPP_WARN(this->get_logger(), "모드: 정지(STOP)");
            } else if (ch == 's') {
                mode_ = true;
                RCLCPP_INFO(this->get_logger(), "모드: 주행(START)");
            }
        }

        // 2. 영상 처리 및 라인 검출
        cv::Mat frame = cv::imdecode(cv::Mat(msg->data), cv::IMREAD_COLOR);
        if (frame.empty()) return;

        // ROI 설정 및 전처리
        cv::Rect roi_rect(0, 270, 640, 90);
        cv::Mat roi = frame(roi_rect);
        cv::Mat gray, bin, display;

        cv::cvtColor(roi, gray, cv::COLOR_BGR2GRAY);
        double avg = cv::mean(gray)[0];
        gray.convertTo(gray, -1, 1, 100 - static_cast<int>(avg));
        cv::threshold(gray, bin, 128, 255, cv::THRESH_BINARY);
        cv::cvtColor(bin, display, cv::COLOR_GRAY2BGR);

        // 레이블링 및 객체 검출
        cv::Mat labels, stats, centroids;
        int n_labels = cv::connectedComponentsWithStats(bin, labels, stats, centroids);

        double min_dist = 1e9;
        int best_idx = -1;

        for (int i = 1; i < n_labels; i++) {
            int area = stats.at<int>(i, cv::CC_STAT_AREA);
            if (area < 100 || area > 15000) continue;


            int left = stats.at<int>(i, cv::CC_STAT_LEFT);
            int top = stats.at<int>(i, cv::CC_STAT_TOP);
            int width = stats.at<int>(i, cv::CC_STAT_WIDTH);
            int height = stats.at<int>(i, cv::CC_STAT_HEIGHT);
            cv::rectangle(display, cv::Rect(left, top, width, height), cv::Scalar(255, 0, 0), 2); // Blue Box

            double cx = centroids.at<double>(i, 0);
            double cy = centroids.at<double>(i, 1);
            double dist = std::sqrt(std::pow(cx - last_line_x_, 2) + std::pow(cy - last_line_y_, 2));

            if (dist < min_dist && dist < 150.0) {
                min_dist = dist;
                best_idx = i;
            }
        }
        // 라인 위치 업데이트 및 시각화
        if (best_idx != -1) {
            last_line_x_ = centroids.at<double>(best_idx, 0);
            last_line_y_ = centroids.at<double>(best_idx, 1);

            cv::rectangle(display, cv::Rect(stats.at<int>(best_idx, 0), stats.at<int>(best_idx, 1), 
                          stats.at<int>(best_idx, 2), stats.at<int>(best_idx, 3)), cv::Scalar(0, 0, 255), 3);
            cv::circle(display, cv::Point(static_cast<int>(last_line_x_), static_cast<int>(last_line_y_)), 5, cv::Scalar(0, 0, 255), -1);
        }

        // 3. 오차 계산 및 속도 명령 생성
        double error = (bin.cols / 2.0) - last_line_x_;
        double abs_error = std::abs(error);
        int steer = 0;
        geometry_msgs::msg::Vector3 vel_msg;

        if (mode_) {
            // 주행 모드: P 제어 적용
            if (abs_error > 200.0) {
                k_ = 0.2;  
            } 
            else if (abs_error > 100.0) {
                k_ = 0.5;
            } 
            else {
                k_ = 0.8; 
            }
            steer = static_cast<int>(error * k_);
            vel_msg.x = base_vel_ - steer;       // 왼쪽 바퀴 속도
            vel_msg.y = -(base_vel_ + steer);    // 오른쪽 바퀴 속도 (반대 방향 가정)
        } else {
            // 정지 모드
            vel_msg.x = 0;
            vel_msg.y = 0;
            steer = 0; 
        }

        // 4. 오차 계산 및 출력
        auto endTime = std::chrono::steady_clock::now();
        float totalTime = std::chrono::duration<float, std::milli>(endTime - startTime).count();
        RCLCPP_INFO(this->get_logger(), "err: %d, lvel: %lf, rvel: %lf, time: %lf",steer, vel_msg.x, vel_msg.y, totalTime);
        
        vel_pub_->publish(vel_msg);

 
        // 5. 결과 윈도우 띄우기
        cv::imshow("1. Raw Video", frame);
        cv::imshow("2. Binary Debug", display);
        cv::waitKey(1);
    }

    // 멤버 변수
    double last_line_x_, last_line_y_;
    bool mode_;
    double k_;
    int base_vel_;

    rclcpp::Subscription<sensor_msgs::msg::CompressedImage>::SharedPtr raw_sub_;
    rclcpp::Publisher<geometry_msgs::msg::Vector3>::SharedPtr vel_pub_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<LineTrackerProcessor>());
    rclcpp::shutdown();
    return 0;
}