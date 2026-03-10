# 25_ROS2_라인검출

## 실습 과제 1

### 패키지명 -> linedetect_rapi5(pub.cpp), linedetect_wsl(sub.cpp)

linedetect_wsl 패키지에 라인 검출 알고리즘을 구현하고 2개의 동영상(5_lt_cw_100rpm_out.mp4, 7_lt_ccw_100rpm_in.mp4)을 이용하여 시뮬레이션을 수행하고 결과를 동영상으로 저장

![image.png](https://github.com/user-attachments/assets/f8557bca-01fd-498f-bac7-03edbf4d0cff)

Publisher 노드
- Jestson nano 보드에서 동영상을 입력 받아 영상 토픽을 발행

- camera_ros2 패키지의 pub.cpp에서 카메라대신 동영상에서 입력 받아 발행하는 것으로 수정

Subscriber 노드
- WSL2에서 영상을 구독하여 라인을 검출하는 노드, 영상처리 결과를 모니터에 출력
- camera_ros2 패키지의 sub.cpp에서 콜백함수안에 라인 검출코드 추가

**소스 코드**

https://github.com/bbanggang/my_ros2_ws/tree/main/src/linedetect_rapi5

https://github.com/bbanggang/my_ros2_ws_wsl/tree/main/src/linedetect_wsl

**실행 결과 1 (5_lt_cw_100rpm_out.mp4 - simulation)**

[https://youtu.be/yAaxHHeXtpc](https://youtu.be/yAaxHHeXtpc)

**실행결과 2  (7_lt_ccw_100rpm_in.mp4 - simulation)**

[https://youtu.be/0QZvbcTrFS8](https://youtu.be/0QZvbcTrFS8)