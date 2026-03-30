# 29_ROS2_라이다

## 실습 과제 1

### WSL2에서 라이다 측정데이터를 이용하여 스캔영상을 그려주는 패키지 lidarplot를 완성하시오.
-> /scan 토픽 구독 → (각도, 거리)로 환산 → 스캔 영상 그리기 순서로 처리할 것  
-> 스캔 영상 그리기는 opencv의 회전변환 또는 삼각함수를 이용  
→ LIDAR의 좌표축과 OpenCV의 윈도우의 좌표축이 서로 다르므로 아래 코드와 같이 변환  


```cpp
	// 거리 환산 비율 설정: 5m가 500px이므로, 반경 1m가 100px에 해당.
  double scale = 100.0; 

  for (int i = 0; i < count; i++) {
    // float degree = RAD2DEG(scan->angle_min + scan->angle_increment * i);
    
    // 거리 데이터 (m 단위)
    float distance = scan->ranges[i];
    float angle_rad = scan->angle_min + scan->angle_increment * i;

    // 유효한 거리 데이터인지 확인
    if (std::isfinite(distance) && distance > 0) {
        int x = 250 + (int)(distance * scale * sin(angle_rad));
        int y = 250 + (int)(distance * scale * cos(angle_rad));

        // 이미지 범위 내에 있는지 확인 후 그리기
        if (x >= 0 && x < 500 && y >= 0 && y < 500) {
            cv::circle(image, cv::Point(x, y), 2, cv::Scalar(0, 0, 255), -1);
        }
    }
```
-> 스캔 영상을 모니터에 출력하고 동시에 동영상(mp4)으로 저장할 것  

```cpp
// 화면에 영상 출력
  cv::imshow("LIDAR Scan", image);
  cv::waitKey(1); // 1ms 대기 

  // 동영상 저장 코드
  if (!is_video_init) {
      video_writer.open("~/ros2_ws/video/scan_video.avi", cv::VideoWriter::fourcc('M', 'J', 'P', 'G'), 10, cv::Size(500, 500), true);
      is_video_init = true;
      printf("[SLLIDAR INFO]: Video recording started.\n");
  }
```

-> RPi5 보드에서 sllidar_node를 실행하고 lidarplot 패키지는 wsl2에서 작성하고 실행  

![image.png](https://github.com/user-attachments/assets/b8475edb-5f89-43c5-a9ce-7401220ff808)

**소스 코드**

→ sllidar_node.cpp

https://github.com/Slamtec/sllidar_ros2/tree/main/src

→ lidarplot

https://github.com/bbanggang/my_ros2_ws_wsl/tree/main/src/lidarplot

**실행 결과**
    
    [녹음 2026-02-12 144035.mp4](https://github.com/user-attachments/assets/2372629a-4945-49f1-91f6-d1112da0d3ab)
    

→ 동영상 저장

![image.png](https://github.com/user-attachments/assets/e40ccf1d-5f14-44e8-8fc3-239a0ce6aca2)

## 실습 과제 2

### 다음 값을 조사하라. 명령어 또는 테스트 코드 이용

- Rplidar C1의 좌표축을 설명

→ LIDAR 방향이 x축이으로 우수계를 사용

![image.png](https://github.com/user-attachments/assets/3c37b625-7d9f-43fb-abb4-16af2a751541)  



- Rplidar C1은 1초에 몇 번 토픽메시지를 전송하는가? 10번

→ 송신(scan frequency : 10.0 Hz)

![image.png](https://github.com/user-attachments/assets/4dc07549-a850-4bf0-ab35-303820576a01)

→ 수신 (약 10번)

![image.png](https://github.com/user-attachments/assets/f8047836-9975-4d6f-b318-ecb31b373b95)  



- 토픽메시지의 크기(KB)는 얼마인가? 5.82KB

![image.png](https://github.com/user-attachments/assets/646d9200-9f84-4649-9ed7-4d8351cd4250)  



- 메시지 1개당 몇 개의 거리 측정값이 포함되어 있는가? **약** **720번**

int count = scan->scan_time / scan->time_increment;

```cpp
count = scan->ranges.size();

printf("[SLLIDAR INFO]: I heard a laser scan %s[%d]:\n",
scan->header.frame_id.c_str(), **count**);
```

![image.png](https://github.com/user-attachments/assets/e33ab243-d0c8-4d6d-a512-2c923bd91a01)  



- 1회전에 몇 번 거리를 측정하는가?

$$
1회전\space 측정\space 횟수=  \frac{angle\space max -angle\space min}{angle\space increment} = \frac{3.141592 +3.141592}{0.00873} \\ \approx 720( 719.7232531500573)
$$



- angle_min, angle_increment 값은 얼마인가?

angle_min : -3.141592…, angle_increment : 0.008738…

![image.png](https://github.com/user-attachments/assets/50d62f1c-cfea-4ba4-92f9-abf24d1d9d52)
