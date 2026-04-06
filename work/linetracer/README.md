# 27_ROS2_라인트레이서

## 실습 과제 1

### linetracer_sim 패키지를 이용하여 트랙에서 실험 수행

![image.png](https://github.com/user-attachments/assets/7519137b-1730-4b1a-a9a3-fe04af1a32e9)

### linedetect_nano 패키지 대신 camera_ros2 패키지 사용
(토픽 주기 30hz, 노드 3개, 토픽 2개)

- 게인(gain)을 이용한 속도 명령

좌측휠 속도 명령 = 직진속도 - k * error

우측휠 속도명령 = -(직진속도 + k * error)

- 게인(gain) 튜닝 방법

게인이 크면 진동이 심하고 회전이 빨라지고, 작으면 느려서 커브 구간에서 라인을 쫒아가지 못하고 벗어나므로 실험적으로 적당한 값을 선정해야함

- 소스 코드

→ camera_ros2

[src/camera_ros2](../../src/camera_ros2)

→ linetracer_sim

[src/linetracer_sim](../../src/linetracer_sim)

→ dxl_nano

https://github.com/bbanggang/my_ros2_ws/tree/main/src/dxl_nano

- 실행 결과

→ robot view

[https://www.youtube.com/watch?v=kfpHmtKOW8w](https://www.youtube.com/watch?v=kfpHmtKOW8w)

→ human view

[https://youtube.com/shorts/jXAMHPxfjnk](https://youtube.com/shorts/jXAMHPxfjnk)
