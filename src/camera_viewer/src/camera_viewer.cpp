#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "cv_bridge/cv_bridge.hpp"
#include <opencv2/opencv.hpp>

static const std::string WINDOW_NAME = "Gazebo Camera";
static const std::string VIDEO_PATH  = "/home/linux/ros2_ws/video/gazebo_camera.mp4";
static const std::string TOPIC       = "/camera/image_raw";

class CameraViewer : public rclcpp::Node
{
public:
  CameraViewer() : Node("camera_viewer")
  {
    // Gazebo는 BEST_EFFORT로 퍼블리시 → SensorDataQoS 사용
    sub_ = create_subscription<sensor_msgs::msg::Image>(
      TOPIC, rclcpp::SensorDataQoS(),
      std::bind(&CameraViewer::image_cb, this, std::placeholders::_1));

    cv::namedWindow(WINDOW_NAME, cv::WINDOW_AUTOSIZE);
    RCLCPP_INFO(get_logger(), "CameraViewer started. topic: %s", TOPIC.c_str());
  }

  ~CameraViewer()
  {
    if (writer_.isOpened())
      writer_.release();
    cv::destroyAllWindows();
  }

private:
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr sub_;
  cv::VideoWriter writer_;
  bool video_init_ = false;

  void image_cb(const sensor_msgs::msg::Image::ConstSharedPtr &msg)
  {
    cv_bridge::CvImagePtr cv_ptr;
    try {
      cv_ptr = cv_bridge::toCvCopy(msg, "bgr8");
    } catch (const cv_bridge::Exception &e) {
      RCLCPP_ERROR(get_logger(), "cv_bridge exception: %s", e.what());
      return;
    }

    cv::Mat &frame = cv_ptr->image;

    // HUD: 해상도 + 타임스탬프 오버레이
    std::string info = std::to_string(frame.cols) + "x" + std::to_string(frame.rows)
                       + "  t:" + std::to_string(msg->header.stamp.sec % 10000);
    cv::putText(frame, info, {6, frame.rows - 8},
                cv::FONT_HERSHEY_PLAIN, 1.0, cv::Scalar(0, 255, 0), 1);

    cv::imshow(WINDOW_NAME, frame);
    cv::waitKey(1);

    // 첫 프레임에서 VideoWriter 초기화
    if (!video_init_) {
      writer_.open(VIDEO_PATH,
                   cv::VideoWriter::fourcc('M', 'J', 'P', 'G'),
                   30, cv::Size(frame.cols, frame.rows), true);
      video_init_ = true;
      if (writer_.isOpened())
        RCLCPP_INFO(get_logger(), "Recording: %s", VIDEO_PATH.c_str());
      else
        RCLCPP_WARN(get_logger(), "Video writer failed to open");
    }

    if (writer_.isOpened())
      writer_.write(frame);
  }
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CameraViewer>());
  rclcpp::shutdown();
  return 0;
}
