#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include <opencv2/opencv.hpp>
#include <cmath>

// 이미지 크기 및 스케일 설정
static constexpr int    IMG_SIZE  = 600;
static constexpr int    CENTER    = IMG_SIZE / 2;
static constexpr double SCALE     = 80.0;   // 1m = 80px  (최대 ~3.75m 표시)
static constexpr double MAX_RANGE = 3.5;    // TurtleBot3 LiDAR 최대 유효 거리 (m)

static const std::string VIDEO_PATH = "/home/linux/ros2_ws/video/gazebo_lidar.mp4";

class LidarViewer : public rclcpp::Node
{
public:
  LidarViewer() : Node("lidar_viewer")
  {
    // Gazebo는 BEST_EFFORT로 퍼블리시 → SensorDataQoS 사용
    sub_ = create_subscription<sensor_msgs::msg::LaserScan>(
      "scan", rclcpp::SensorDataQoS(),
      std::bind(&LidarViewer::scan_cb, this, std::placeholders::_1));

    RCLCPP_INFO(get_logger(), "LidarViewer started. Subscribing to /scan");
  }

private:
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr sub_;
  cv::VideoWriter writer_;
  bool video_init_ = false;

  // 거리 참조 원, 십자선, 방향 레이블을 배경으로 그린다
  void draw_background(cv::Mat &img)
  {
    img = cv::Mat(IMG_SIZE, IMG_SIZE, CV_8UC3, cv::Scalar(30, 30, 30));

    // 1m / 2m / 3m 참조 원
    for (int r_m = 1; r_m <= 3; ++r_m) {
      int r_px = static_cast<int>(r_m * SCALE);
      cv::circle(img, {CENTER, CENTER}, r_px, cv::Scalar(70, 70, 70), 1);
      cv::putText(img, std::to_string(r_m) + "m",
                  {CENTER + r_px + 3, CENTER - 4},
                  cv::FONT_HERSHEY_PLAIN, 0.9, cv::Scalar(100, 100, 100), 1);
    }

    // 십자선
    cv::line(img, {CENTER, 0},       {CENTER, IMG_SIZE}, cv::Scalar(60, 60, 60), 1);
    cv::line(img, {0,      CENTER},  {IMG_SIZE, CENTER}, cv::Scalar(60, 60, 60), 1);

    // 방향 레이블 (로봇 기준: F=앞, B=뒤, L=좌, R=우)
    cv::putText(img, "F", {CENTER - 7, 18},            cv::FONT_HERSHEY_PLAIN, 1.2, cv::Scalar(0, 220, 0), 2);
    cv::putText(img, "B", {CENTER - 7, IMG_SIZE - 6},  cv::FONT_HERSHEY_PLAIN, 1.2, cv::Scalar(0, 220, 0), 2);
    cv::putText(img, "L", {6,          CENTER + 6},    cv::FONT_HERSHEY_PLAIN, 1.2, cv::Scalar(0, 220, 0), 2);
    cv::putText(img, "R", {IMG_SIZE - 18, CENTER + 6}, cv::FONT_HERSHEY_PLAIN, 1.2, cv::Scalar(0, 220, 0), 2);

    // 로봇 중심 점
    cv::circle(img, {CENTER, CENTER}, 5, cv::Scalar(0, 255, 255), -1);
  }

  void scan_cb(const sensor_msgs::msg::LaserScan::SharedPtr scan)
  {
    cv::Mat img;
    draw_background(img);

    int count = static_cast<int>(scan->ranges.size());

    for (int i = 0; i < count; ++i) {
      float dist  = scan->ranges[i];
      float angle = scan->angle_min + scan->angle_increment * i;

      if (!std::isfinite(dist) || dist <= scan->range_min || dist > MAX_RANGE)
        continue;

      // ROS 로봇 프레임 → 화면 좌표
      //   로봇 앞(angle=0)  → 화면 위  (sy 감소)
      //   로봇 좌(angle>0)  → 화면 왼쪽 (sx 감소)
      int sx = CENTER - static_cast<int>(dist * SCALE * std::sin(angle));
      int sy = CENTER - static_cast<int>(dist * SCALE * std::cos(angle));

      if (sx < 0 || sx >= IMG_SIZE || sy < 0 || sy >= IMG_SIZE)
        continue;

      // 가까울수록 빨강, 멀수록 파랑으로 색상 그라디언트
      float ratio = dist / MAX_RANGE;
      int   b     = static_cast<int>(ratio * 255);
      int   r     = static_cast<int>((1.0f - ratio) * 255);
      cv::circle(img, {sx, sy}, 2, cv::Scalar(b, 50, r), -1);
    }

    // HUD: 포인트 수 표시
    std::string hud = "points: " + std::to_string(count);
    cv::putText(img, hud, {6, IMG_SIZE - 8},
                cv::FONT_HERSHEY_PLAIN, 1.0, cv::Scalar(180, 180, 180), 1);

    cv::imshow("Gazebo LiDAR", img);
    cv::waitKey(1);

    // 첫 프레임에서 VideoWriter 초기화
    if (!video_init_) {
      writer_.open(VIDEO_PATH,
                   cv::VideoWriter::fourcc('M', 'J', 'P', 'G'),
                   10, cv::Size(IMG_SIZE, IMG_SIZE), true);
      video_init_ = true;
      if (writer_.isOpened())
        RCLCPP_INFO(get_logger(), "Recording: %s", VIDEO_PATH.c_str());
      else
        RCLCPP_WARN(get_logger(), "Video writer failed to open");
    }

    if (writer_.isOpened())
      writer_.write(img);
  }
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<LidarViewer>());
  rclcpp::shutdown();
  return 0;
}
