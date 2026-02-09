#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/compressed_image.hpp"
#include "opencv2/opencv.hpp"
#include <memory>
#include <functional>
#include <iostream>
#include <opencv2/imgcodecs.hpp>
  

void mysub_callback(const std::string& win_name, const sensor_msgs::msg::CompressedImage::SharedPtr msg)
{
    cv::Mat frame = cv::imdecode(cv::Mat(msg->data), cv::IMREAD_UNCHANGED);
    if(!frame.empty()){
        cv::imshow(win_name,frame);
        cv::waitKey(1);
    }
    
}
int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<rclcpp::Node>("camsub_wsl");
    // auto qos_profile = rclcpp::QoS(rclcpp::KeepLast(10)); //TCP
    auto qos_profile = rclcpp::QoS(rclcpp::KeepLast(10)).best_effort(); //UDP

    // 람다 함수를 이용한 각 토픽별 구독 설정
    // 람다 함수를 사용했기 때문에 bind로 std::function<void(MessageT) 형으로 만들 필요 없음
    auto sub_raw = node->create_subscription<sensor_msgs::msg::CompressedImage>(
        "image/raw/compressed", qos_profile, 
        [](const sensor_msgs::msg::CompressedImage::SharedPtr msg) {
            mysub_callback("Original Video", msg);
        });

    auto sub_gray = node->create_subscription<sensor_msgs::msg::CompressedImage>(
        "image/gray/compressed", qos_profile, 
        [](const sensor_msgs::msg::CompressedImage::SharedPtr msg) {
            mysub_callback("Grayscale Video", msg);
        });

    auto sub_bin = node->create_subscription<sensor_msgs::msg::CompressedImage>(
        "image/binary/compressed", qos_profile, 
        [](const sensor_msgs::msg::CompressedImage::SharedPtr msg) {
            mysub_callback("Binary Video", msg);
        });

    rclcpp::spin(node);
    cv::destroyAllWindows();
    rclcpp::shutdown();
    return 0;
}