# 26_ROS2_모션제어

## 실습 과제 1

### 다음 블럭도를 참고하여 동영상을 이용하여 시뮬레이션을 수행하는 linetracer_sim 패키지를 구현하라

![image.png](https://github.com/user-attachments/assets/eb9dba7c-317b-4278-84aa-08fd8c2885d2)

### linetracer_sim 패키지 세부 기능-> 노드3개, 토픽 2개

- 라인검출 패키지 코드를 이용하여 위치오차 계산 -> 좌,우 속도명령 계산 -> 좌,우 속도명령 퍼블리시( 1개의 노드 안에 subscriber와 publisher 모두 필요)
- 라인 검출은 항상 실행되어야 하고 속도명령 전송은 s키를 누르면 전송하고 q를 누르면 정지명령(0rpm)을 전송
- 터미널에는 에러, 좌, 우속도명령, 처리시간을 출력
- 직진속도는 100rpm으로 하고 게인 k=1로 놓고 실험

### 동영상 5_lt_cw_100rpm_out.mp4에 대하여 시뮬레이션을 실행하고
다음 그림처럼 결과 동영상을 만들어서 제출할 것

- 터미널에 위치오차, 좌,우속도명령, 처리시간을 출력
- 실제 터미널에 출력값과 바퀴의 실제 속도가 맞는지 확인할 것

- 소스 코드

→linedetect_rapi5

https://github.com/bbanggang/my_ros2_ws/tree/main/src/linedetect_rapi5

→ linetracer_sim

https://github.com/bbanggang/my_ros2_ws_wsl/tree/main/src/linetracer_sim

→ dxl_nano

https://github.com/bbanggang/my_ros2_ws/tree/main/src/dxl_nano

![image.png](https://github.com/user-attachments/assets/846ac632-9060-400d-b262-c4e0ed8f97fe)


- 실행 결과 1

→ 5_lt_cw_100rpm_out.mp4

[https://youtu.be/t8zn24B6_0Q](https://youtu.be/t8zn24B6_0Q)

- 실행 결과 2

→ 7_lt_ccw_100rpm_in.mp4

[https://www.youtube.com/watch?v=FwXG0Z9sXUA](https://www.youtube.com/watch?v=FwXG0Z9sXUA)
