#ifndef SUB_HPP_
#define SUB_HPP_

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/compressed_image.hpp"
#include "opencv2/opencv.hpp"
#include <memory>

class ImageSubscriber : public rclcpp::Node
{
public:
    ImageSubscriber();
    virtual ~ImageSubscriber();

private:
    // 영상 수신 시 실행될 콜백 함수
    void image_callback(const sensor_msgs::msg::CompressedImage::SharedPtr msg);

    rclcpp::Subscription<sensor_msgs::msg::CompressedImage>::SharedPtr subscription_;
};

#endif // SUB_HPP_