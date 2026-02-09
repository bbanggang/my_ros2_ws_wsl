#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/compressed_image.hpp"
#include "cv_bridge/cv_bridge.hpp"
#include "opencv2/opencv.hpp"
#include <memory>

// Raspberry Pi 5 + IMX219 GStreamer 파이프라인
std::string src = "libcamerasrc ! \
    video/x-raw,format=YUY2,width=640,height=480,framerate=30/1 ! \
    queue ! videoconvert ! videoflip method=rotate-180 ! \
    videoconvert ! video/x-raw, format=(string)BGR ! appsink";

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<rclcpp::Node>("image_publisher");
    
    auto qos_profile = rclcpp::QoS(rclcpp::KeepLast(10)).best_effort();
    auto pub = node->create_publisher<sensor_msgs::msg::CompressedImage>("image/raw/compressed", qos_profile);

    cv::VideoCapture cap(src, cv::CAP_GSTREAMER);
    if (!cap.isOpened()) {
        RCLCPP_ERROR(node->get_logger(), "카메라를 열 수 없습니다.");
        return -1;
    }

    cv::Mat frame;
    rclcpp::WallRate loop_rate(30.0);

    RCLCPP_INFO(node->get_logger(), "영상 송신을 시작합니다.");

    while (rclcpp::ok()) {
        cap >> frame;
        if (frame.empty()) break;

        std_msgs::msg::Header hdr;
        hdr.stamp = node->get_clock()->now();
        
        // 원본 영상 압축 및 메시지 변환
        auto msg = cv_bridge::CvImage(hdr, "bgr8", frame).toCompressedImageMsg();
        pub->publish(*msg);

        loop_rate.sleep();
    }

    rclcpp::shutdown();
    return 0;
}