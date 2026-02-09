#include "camera2_3/sub.hpp"

ImageSubscriber::ImageSubscriber() : Node("camsub_wsl")
{
    auto qos_profile = rclcpp::QoS(rclcpp::KeepLast(10)).best_effort();
    
    subscription_ = this->create_subscription<sensor_msgs::msg::CompressedImage>(
        "image/compressed",
        qos_profile,
        std::bind(&ImageSubscriber::image_callback, this, std::placeholders::_1)
    );

    RCLCPP_INFO(this->get_logger(), "이미지 수신 노드가 시작되었습니다.");
}

ImageSubscriber::~ImageSubscriber()
{
    cv::destroyAllWindows();
}

void ImageSubscriber::image_callback(const sensor_msgs::msg::CompressedImage::SharedPtr msg)
{
    // 압축된 데이터를 OpenCV Mat 형식으로 디코딩
    cv::Mat frame = cv::imdecode(cv::Mat(msg->data), cv::IMREAD_COLOR);

    if (frame.empty()) {
        RCLCPP_ERROR(this->get_logger(), "디코딩된 프레임이 비어 있습니다.");
        return;
    }

    cv::imshow("wsl", frame);
    
    cv::waitKey(1);

    // 수신 정보 로그 출력
    RCLCPP_INFO(this->get_logger(), "Received Image : %s, %d x %d", 
                msg->format.c_str(), frame.rows, frame.cols);
}