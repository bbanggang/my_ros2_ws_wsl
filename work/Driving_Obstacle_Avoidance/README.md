# 30_ROS2_장애물회피주행

## 실습 과제 1

### 키보드로 로봇을 원격제어하는 dxl  패키지를 이용하여 강의실 앞쪽에서 책상 사이를 통과하여 뒤쪽으로 주행하고 다시 원위치로 돌아오도록 제어하고 lidarplot 패키지의 스캔 영상(2m x 2m 영역)을 동영상 파일(mp4)로 저장하는 패키지 lidarsave를 작성

- 라이다는 초당 10번 토픽 전송하므로 동영상은 10fps로 저장

![image.png](https://github.com/user-attachments/assets/b694b9a5-599b-4f23-8f58-90976d02b8c0)

**소스 코드**

→ sllidar_node

[src/sllidar_ros2/src](../../src/sllidar_ros2/src)

→ dxl_nano (node_dxlsub)

https://github.com/bbanggang/my_ros2_ws/tree/main/src/dxl_nano

→ lidarsave

[src/lidarsave](../../src/lidarsave)

→ dxl_wsl (node_dxlpub)

[src/dxl_wsl](../../src/dxl_wsl)


**실행 결과**

[video](https://github.com/user-attachments/assets/a6c6184c-cda9-4e1e-ac36-c3eea237ea5b)


## 실습 과제 2

### 장애물 회피 알고리즘을 작성하고 lidarsave 패키지에서 저장한 동영상 파일을 이용하여 시뮬레이션을 수행하는 lidarsim 패키지를 작성

- 스캔 영상에서 에러 계산 → 영상 처리 결과를 동영상으로 저장 → 에러를 이용하여 속도명령을 전송 → 다이내믹셀 구동

![image.png](https://github.com/user-attachments/assets/908a0b95-0369-41ca-baef-398eab910d53)

**소스 코드**

→ lidarsim

[src/lidarsim](../../src/lidarsim)

→ dxl_nano (node_dxlsub)

https://github.com/bbanggang/my_ros2_ws/tree/main/src/dxl_nano


**실행 결과**

[video](https://github.com/user-attachments/assets/93ce2e94-8f77-42c6-ad41-b0ea7787e145)

## 실습 과제 3

### 장애물 회피 알고리즘을 구현하여 강의실 앞공간에서 책상사이를 통과하여 뒷공간에 도착하도록 제어하는 lidardrive 패키지 작성

![image.png](https://github.com/user-attachments/assets/e8e7283f-5f91-4b0d-b5f4-55c9a6fd1466)

**소스 코드**

→ sllidar_node

[src/sllidar_ros2/src](../../src/sllidar_ros2/src)

→ lidardrive

[src/lidardrive](../../src/lidardrive)

→ dxl_nano (node_dxlsub)

https://github.com/bbanggang/my_ros2_ws/tree/main/src/dxl_nano


**실행 결과**

→ Terminal Video

[video](https://github.com/user-attachments/assets/44690444-04f3-4ae1-b4a0-f4e53207c532)

→ Robot_View and Draw LiDAR Data for OpenCV
 
[robot_view](https://github.com/user-attachments/assets/330b7746-32f1-4ec0-97ad-69e61695613f)

→ Human_View

[human_view](https://github.com/user-attachments/assets/4ffc8c2b-4a06-4cf9-bb65-56af8d2e6edd)


