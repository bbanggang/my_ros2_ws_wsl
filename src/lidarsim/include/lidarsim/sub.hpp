#ifndef LIDARDRIVE_HPP_
#define LIDARDRIVE_HPP_

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/vector3.hpp"
#include "opencv2/opencv.hpp"
#include <termios.h>
#include <fcntl.h>
#include <chrono>

class LineDetector : public rclcpp::Node {
public:
    LineDetector();
    ~LineDetector();

private:
    void timer_callback();
    cv::Mat preprocess_image(const cv::Mat& result);
    std::pair<int, int> find_target_line(const cv::Mat& roi, const cv::Mat& stats, const cv::Mat& centroids);
    void draw_result(cv::Mat& result, const cv::Mat& stats, const cv::Mat& centroids, int left_idx, int right_idx);

    int getch();
    bool kbhit();

    rclcpp::Publisher<geometry_msgs::msg::Vector3>::SharedPtr pub_;
    rclcpp::TimerBase::SharedPtr timer_;
    cv::VideoCapture cap_;
    cv::VideoWriter video_writer;
    
    cv::Point tmp_pt_l, tmp_pt_r;
    geometry_msgs::msg::Vector3 vel;
    bool mode;
    double k = 1.5;
    cv::Mat labels;
};

#endif