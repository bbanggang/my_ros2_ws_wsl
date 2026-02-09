#include "camera2_3/pub.hpp"
#include "sensor_msgs/msg/compressed_image.hpp"
#include <chrono>
#include <opencv2/videoio.hpp>
#include <rclcpp/logging.hpp>
#include <rclcpp/qos.hpp>
#include <rclcpp/utilities.hpp>

ImagePublisher::ImagePublisher():Node("image_publisher"){
  auto qos_profile = rclcpp::QoS(rclcpp::KeepLast(10)).best_effort();
  publisher_ = this->create_publisher<sensor_msgs::msg::CompressedImage>("image/compressed", qos_profile);
  
  pipeline_ = "libcamerasrc ! \
    video/x-raw,format=YUY2,width=640,height=480,framerate=30/1 ! \
      queue ! videoconvert ! videoflip method=rotate-180 ! \
      videoconvert ! video/x-raw, format=(string)BGR ! appsink";

  cap_.open(pipeline_, cv::CAP_GSTREAMER);
  if(!cap_.isOpened()){
    RCLCPP_ERROR(this->get_logger(), "Could not open video via GStreamer!") ;
    rclcpp::shutdown();
  }

  timer_ = this->create_wall_timer(std::chrono::milliseconds(33), 
  std::bind(&ImagePublisher::timer_callback, this)
  );
}

ImagePublisher::~ImagePublisher()
{
    if (cap_.isOpened()) {
        cap_.release();
    }
}

void ImagePublisher::timer_callback()
{
    cv::Mat frame;
    cap_ >> frame;

    if (frame.empty()) {
        RCLCPP_ERROR(this->get_logger(), "Captured frame is empty!");
        return;
    }

    std_msgs::msg::Header hdr;
    hdr.stamp = this->get_clock()->now();
    
    auto msg = cv_bridge::CvImage(hdr, "bgr8", frame).toCompressedImageMsg();
    publisher_->publish(*msg);
}

