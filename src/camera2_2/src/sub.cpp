#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/compressed_image.hpp"
#include "opencv2/opencv.hpp"
#include <memory>

class ImageSubSaver : public rclcpp::Node
{
public:
    ImageSubSaver() : Node("image_sub_saver")
    {
        auto qos_profile = rclcpp::QoS(rclcpp::KeepLast(10)).best_effort();
        
        sub_ = this->create_subscription<sensor_msgs::msg::CompressedImage>(
            "image/raw/compressed", qos_profile,
            std::bind(&ImageSubSaver::callback, this, std::placeholders::_1));
        
        RCLCPP_INFO(this->get_logger(), "수신 및 저장 노드가 시작되었습니다.");
    }

    ~ImageSubSaver() {
        if (writer_.isOpened()) {
            writer_.release();
            RCLCPP_INFO(this->get_logger(), "영상이 안전하게 저장되었습니다.");
        }
    }

private:
    void callback(const sensor_msgs::msg::CompressedImage::SharedPtr msg)
    {
        cv::Mat frame = cv::imdecode(cv::Mat(msg->data), cv::IMREAD_COLOR);
        if (frame.empty()) return;

        // 첫 프레임이 들어올 때 VideoWriter 초기화
        if (!writer_.isOpened()) {
            int fourcc = cv::VideoWriter::fourcc('m', 'p', '4', 'v'); // mp4 코덱
            double fps = 30.0;
            writer_.open("/home/linux/ros2_ws/output.mp4", fourcc, fps, frame.size(), true);
            RCLCPP_INFO(this->get_logger(), "녹화를 시작합니다: output.mp4");
        }

        // 영상 저장 및 화면 출력
        writer_.write(frame);
        cv::imshow("Received Video", frame);
        cv::waitKey(1);
    }

    rclcpp::Subscription<sensor_msgs::msg::CompressedImage>::SharedPtr sub_;
    cv::VideoWriter writer_;
};

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ImageSubSaver>();
    
    rclcpp::spin(node); // Ctrl+C를 누르면 spin이 종료됨
    
    cv::destroyAllWindows();
    rclcpp::shutdown();
    return 0;
}