#ifndef _PUB_HPP_
#define _PUB_HPP_

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/compressed_image.hpp"
#include "cv_bridge/cv_bridge.hpp"
#include "opencv2/opencv.hpp"
#include <memory>
#include <chrono>
#include <opencv2/videoio.hpp>
#include <rclcpp/node.hpp>
#include <rclcpp/publisher.hpp>
#include <rclcpp/timer.hpp>

class ImagePublisher : public rclcpp::Node{
public:
    ImagePublisher();
    virtual ~ImagePublisher();
private:
    void timer_callback();

    rclcpp::Publisher<sensor_msgs::msg::CompressedImage>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
    cv::VideoCapture cap_;
    std::string pipeline_;
};

#endif