#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/compressed_image.hpp"
#include "geometry_msgs/msg/vector3.hpp"
#include "std_msgs/msg/float64.hpp"
#include "cv_bridge/cv_bridge.hpp"
#include "opencv2/opencv.hpp"
#include <cmath>
#include <chrono>
#include <termios.h>
#include <fcntl.h>

class LineTrackerProcessor : public rclcpp::Node {
public:
    LineTrackerProcessor() : Node("lane_tracker_node"), mode_(false), k_(1.0), base_vel_(100) {
        auto qos_profile = rclcpp::QoS(rclcpp::KeepLast(10)).best_effort();

        raw_sub_ = this->create_subscription<sensor_msgs::msg::CompressedImage>(
            "Image_Topic", qos_profile,
            std::bind(&LineTrackerProcessor::image_callback, this, std::placeholders::_1));

        vel_pub_ = this->create_publisher<geometry_msgs::msg::Vector3>("vel_cmd_topic", qos_profile);

        // 초기 차선 위치 설정 (좌측: 160, 우측: 480 부근 가정)
        last_left_x_ = 100.0;
        last_left_y_ = 30.0;  
        last_right_x_ = 550.0;
        last_right_y_ = 30.0; 
        
        RCLCPP_INFO(this->get_logger(), "WSL 차선(Lane) 추적 노드 시작. 's': 주행, 'q': 정지");
    }

private:
    // 키보드 입력 유틸리티 (getch, kbhit는 기존과 동일)
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
        auto startTime = std::chrono::steady_clock::now();
        
        if (kbhit()) {
            char ch = getch();
            if (ch == 'q') { mode_ = false; RCLCPP_WARN(this->get_logger(), "STOP"); }
            else if (ch == 's') { mode_ = true; RCLCPP_INFO(this->get_logger(), "START"); }
        }

        cv::Mat frame = cv::imdecode(cv::Mat(msg->data), cv::IMREAD_COLOR);
        if (frame.empty()) return;

        // 1. ROI 및 전처리
        cv::Rect roi_rect(0, 270, 640, 90);
        cv::Mat roi = frame(roi_rect);
        cv::Mat gray, bin, display;

        cv::cvtColor(roi, gray, cv::COLOR_BGR2GRAY);
        double avg = cv::mean(gray)[0];
        gray.convertTo(gray, -1, 1, 100 - static_cast<int>(avg));
        cv::threshold(gray, bin, 150, 255, cv::THRESH_BINARY);
        cv::cvtColor(bin, display, cv::COLOR_GRAY2BGR);

        // 2. 레이블링
        cv::Mat labels, stats, centroids;
        int n_labels = cv::connectedComponentsWithStats(bin, labels, stats, centroids);

        // 좌/우 독립적 검출 변수
        double left_min_dist = 1e9, right_min_dist = 1e9;
        int left_best_idx = -1, right_best_idx = -1;
        const double mid_line = bin.cols / 2.0; // 320.0

        // 최적의 좌/우 차선 인덱스 찾기
        for (int i = 1; i < n_labels; i++) {
            int area = stats.at<int>(i, cv::CC_STAT_AREA);
            if (area < 750 || area > 15000) continue;

            double cx = centroids.at<double>(i, 0);
            double cy = centroids.at<double>(i, 1); // y좌표 획득

            // ROI를 분할하여 탐색
            if (cx < mid_line) { // 좌측 영역
                double dist = std::sqrt(std::pow(cx - last_left_x_, 2) + std::pow(cy - last_left_y_, 2));
                if (dist < left_min_dist && dist < 65.0) {
                    left_min_dist = dist;
                    left_best_idx = i;
                }
            } else { // 우측 영역
                double dist = std::sqrt(std::pow(cx - last_right_x_, 2) + std::pow(cy - last_right_y_, 2));
                if (dist < right_min_dist && dist < 65.0) {
                    right_min_dist = dist;
                    right_best_idx = i;
                }
            }
        }
        // 시각화 처리
        for (int i = 1; i < n_labels; i++) {
            int area = stats.at<int>(i, cv::CC_STAT_AREA);
            if (area < 750 || area > 15000) continue;

            int left = stats.at<int>(i, cv::CC_STAT_LEFT);
            int top = stats.at<int>(i, cv::CC_STAT_TOP);
            int width = stats.at<int>(i, cv::CC_STAT_WIDTH);
            int height = stats.at<int>(i, cv::CC_STAT_HEIGHT);

            if (i == left_best_idx || i == right_best_idx) {
                // 검출된 진짜 라인: 빨간색 
                cv::rectangle(display, cv::Rect(left, top, width, height), cv::Scalar(0, 0, 255), 1);
            } else {
                // 그 외 후보 객체: 파란색
                cv::rectangle(display, cv::Rect(left, top, width, height), cv::Scalar(255, 0, 0), 1);
            }
        }

        // 3. 데이터 업데이트
        if (left_best_idx != -1) {
            last_left_x_ = centroids.at<double>(left_best_idx, 0);
            last_left_y_ = centroids.at<double>(left_best_idx, 1);
        }
        if (right_best_idx != -1) {
            last_right_x_ = centroids.at<double>(right_best_idx, 0);
            last_right_y_ = centroids.at<double>(right_best_idx, 1);
        }

        // 검출 여부와 상관없이 이전/현재 위치에 빨간 점 찍기
        cv::circle(display, cv::Point(static_cast<int>(last_left_x_), static_cast<int>(last_left_y_)), 3, cv::Scalar(0, 0, 255), -1);
        cv::circle(display, cv::Point(static_cast<int>(last_right_x_), static_cast<int>(last_right_y_)), 3, cv::Scalar(0, 0, 255), -1);

        // 4. 오차 계산 (Lane Midpoint 기반)
        // 수식: Image Center - (Left + Right) / 2
        double lane_midpoint = (last_left_x_ + last_right_x_) / 2.0;
        double error = mid_line - lane_midpoint;
        double abs_error = std::abs(error);

        // 제어 게인 및 속도 명령 생성
        geometry_msgs::msg::Vector3 vel_msg;
        if (mode_) {
            if (abs_error > 150.0) k_ = 0.2;
            else if (abs_error > 70.0) k_ = 0.5;
            else k_ = 0.8;

            int steer = static_cast<int>(error * k_);
            vel_msg.x = base_vel_ - steer;
            vel_msg.y = -(base_vel_ + steer);
        } else {
            vel_msg.x = 0; vel_msg.y = 0;
        }

        auto endTime = std::chrono::steady_clock::now();
        float totalTime = std::chrono::duration<float, std::milli>(endTime - startTime).count();
        RCLCPP_INFO(this->get_logger(), "err: %lf, lvel: %lf, rvel: %lf, time: %lf",error, vel_msg.x, vel_msg.y, totalTime);

        vel_pub_->publish(vel_msg);
        cv::imshow("raw video", frame);
        cv::imshow("Lane Detection Debug", display);
        cv::waitKey(1);
    }

    double last_left_x_, last_left_y_;
    double last_right_x_, last_right_y_;
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