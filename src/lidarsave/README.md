# lidarsave 패키지 설명

## 전체 프로그램 동작 설명

`lidarsave.cpp`는 ROS2 기반 LIDAR 스캔 데이터 시각화 및 녹화 노드이다. SLLIDAR가 발행하는 `sensor_msgs::msg::LaserScan` 메시지를 구독하여, 각 스캔 포인트를 2D 평면도 위에 점으로 찍어 실시간 시각화하고, 해당 영상을 MP4 파일로 저장한다.

lidarplot 패키지와 동일한 구조이며, scale 값이 달라 표시 범위가 다르다.
- lidarplot: `scale = 50.0` → 1m = 50px, 최대 5m 표시
- lidarsave: `scale = 100.0` → 1m = 100px, 최대 2.5m 표시 (더 좁은 범위를 크게 표시)

처리 흐름:

1. **스캔 수신** — `scan` 토픽에서 `LaserScan` 메시지를 수신하여 거리·각도 데이터 배열을 얻음
2. **좌표 변환** — 극좌표(거리, 각도)를 직교좌표(x, y 픽셀)로 변환하여 500×500 이미지 위에 매핑
3. **시각화** — 유효한 스캔 포인트를 빨간 점으로 표시하고, 중심(로봇 위치)을 십자로 표시
4. **녹화** — 첫 프레임 수신 시 VideoWriter를 초기화하고, 이후 매 프레임을 MP4 파일에 기록

---

## 함수별 설명

### 1. 전역 변수 및 매크로

```cpp
#define RAD2DEG(x) ((x)*180./M_PI)

static cv::VideoWriter video_writer;
static bool is_video_init = false;  // 첫 프레임에서만 VideoWriter 초기화하기 위한 플래그
```

---

### 2. `scanCb()` — 스캔 데이터 수신 콜백

```cpp
static void scanCb(sensor_msgs::msg::LaserScan::SharedPtr scan) {
    int count = scan->scan_time / scan->time_increment;

    // ranges 배열의 실제 크기로 count 재설정 (불일치 방지)
    if(scan->ranges.size() > 0) count = scan->ranges.size();

    // 500×500 흰색 배경 이미지 생성
    cv::Mat image(500, 500, CV_8UC3, cv::Scalar(255, 255, 255));
    cv::Point center(250, 250);

    // 중심에 십자 표시 — 로봇 위치 기준점
    cv::line(image, cv::Point(250, 245), cv::Point(250, 255), cv::Scalar(0, 0, 0), 1);
    cv::line(image, cv::Point(245, 250), cv::Point(255, 250), cv::Scalar(0, 0, 0), 1);

    // 거리-픽셀 환산 비율: 1m = 100px
    // 이미지 반경 250px ÷ 100px/m = 최대 표시 거리 2.5m
    double scale = 100.0;

    for (int i = 0; i < count; i++) {
        float distance = scan->ranges[i];
        float angle_rad = scan->angle_min + scan->angle_increment * i;

        if (std::isfinite(distance) && distance > 0) {
            int x = 250 + (int)(distance * scale * sin(angle_rad));
            int y = 250 + (int)(distance * scale * cos(angle_rad));

            if (x >= 0 && x < 500 && y >= 0 && y < 500) {
                cv::circle(image, cv::Point(x, y), 2, cv::Scalar(0, 0, 255), -1);
            }
        }
    }

    cv::imshow("LIDAR Scan", image);
    cv::waitKey(1);

    // 첫 프레임에서만 VideoWriter 초기화
    if (!is_video_init) {
        video_writer.open("/home/linux/ros2_ws/video/ros2_basic_30_test1.mp4",
                          cv::VideoWriter::fourcc('M', 'J', 'P', 'G'),
                          10, cv::Size(500, 500), true);
        is_video_init = true;
    }

    if (video_writer.isOpened()) {
        video_writer.write(image);
    }
}
```

---

### 3. `main()` — 진입점

```cpp
int main(int argc, char **argv) {
    rclcpp::init(argc, argv);

    auto node = rclcpp::Node::make_shared("sllidar_client");

    // SensorDataQoS: LIDAR 드라이버 노드와 동일한 QoS로 구독
    auto lidar_info_sub = node->create_subscription<sensor_msgs::msg::LaserScan>(
                          "scan", rclcpp::SensorDataQoS(), scanCb);

    rclcpp::spin(node);
    rclcpp::shutdown();

    // 종료 시 VideoWriter를 닫아 MP4 파일 정상 마무리
    if (video_writer.isOpened()) {
        video_writer.release();
    }

    return 0;
}
```
