# Dynamixel Workbench 설치 마이그레이션 보고서

## Jetson Nano → Raspberry Pi 5 / ROS2 Jazzy 환경 전환 시 발생한 차이점 및 해결 방법

---

## 1. 환경 비교

| 항목 | 원본 환경 (기존 가이드) | 마이그레이션 환경 |
| --- | --- | --- |
| 보드 | Jetson Nano | Raspberry Pi 5 |
| 아키텍처 | x86_64 (aarch64) | aarch64 (ARM64) |
| ROS2 버전 | Foxy/Humble  | Jazzy |
| Dynamixel SDK 설치 방식 | 소스 빌드 (/usr/local/include) | apt 설치 (ros-jazzy-dynamixel-sdk) |
| SDK 헤더 경로 | /usr/local/include/dynamixel_sdk | /opt/ros/jazzy/include/dynamixel_sdk |
| SDK 라이브러리 | dxl_x64_cpp | dynamixel_sdk (ament 패키지) |
| 사용자 홈 경로 | /home/ncslab | /home/rapi5 |
| 모터 모델 | MX 시리즈 | XC430-W150-T |

---

## 2. 단계별 차이점 상세

### 2.1 Dynamixel C++ SDK 설치

### 원본 방식

소스코드를 직접 빌드하여 `/usr/local/include/dynamixel_sdk` 경로에 헤더 파일을 설치하고, `dxl_x64_cpp` 라이브러리를 직접 링크하는 방식이었다.

### 마이그레이션 방식

apt 패키지 매니저를 통해 `ros-jazzy-dynamixel-sdk`를 설치하였다. 

이 경우 헤더 파일은 `/opt/ros/jazzy/include/dynamixel_sdk`에 위치하며, CMake에서 `find_package(dynamixel_sdk REQUIRED)`로 자동 인식된다.

![image.png](https://github.com/user-attachments/assets/c85d9a22-3a4b-4684-a304-084163f68efc)

### 영향

이후 모든 패키지의 CMakeLists.txt에서 SDK 경로 및 링크 방식을 수정해야 했다. 

---

### 2.2 dynamixel_workbench_msgs 설치

### 차이점

`git clone -b ros2` 명령으로 ROBOTIS 공식 repo의 ros2 브랜치를 클론하여 빌드하는 절차는 동일하게 적용 가능했다. Jazzy 환경에서도 빌드 에러 없이 정상적으로 msg/srv 파일이 생성되었다.

### 추가 고려사항

빌드 후 환경변수 영속화를 위해 `.bashrc`에 `source ~/ros2_ws/install/local_setup.bash`가 추가되어 있는지 확인이 필요했다. 

```bash
# ./bashrc의 ROS 2 Jazzy 환경 로드 코드
source /opt/ros/jazzy/setup.bash
source ~/ros2_ws/install/local_setup.bash
```

`ros2 interface list | grep dynamixel` 명령으로 메시지 타입이 ROS2 환경에서 정상 인식되는지 검증하는 과정을 추가로 수행했다.

![image.png](https://github.com/user-attachments/assets/f0aabd7f-6e0b-4a02-9de8-c3a6850cbfc8)

---

### 2.3 dynamixel_workbench_operators 설치

### 문제 1: C++ 표준 버전 불일치 (빌드 에러)

ROS2 Jazzy의 `rclcpp` 헤더는 C++17 기능을 사용하지만, 원본 CMakeLists.txt는 C++14로 설정되어 있어 대량의 컴파일 에러가 발생했다.

**해결 방법 — CMakeLists.txt 수정 (2곳):**

```
# 변경 전
set(CMAKE_CXX_STANDARD 14)
add_compile_options(-std=c++14)

# 변경 후
set(CMAKE_CXX_STANDARD 17)
add_compile_options(-std=c++17)
```

### 문제 2: SDK 라이브러리 링크 방식 불일치 (빌드 에러)

RPi5는 aarch64 아키텍처이므로 `dxl_x64_cpp`를 사용할 수 없다.

**해결 방법 — CMakeLists.txt 수정:**

```
# 변경 전
target_link_libraries(dynamixel_workbench_operators dxl_x64_cpp)
target_include_directories(dynamixel_workbench_operators PRIVATE /usr/local/include/dynamixel_sdk)

# 변경 후 — 위 두 줄을 삭제하고 아래와 같이 수정
find_package(dynamixel_sdk REQUIRED)
ament_target_dependencies(dynamixel_workbench_operators rclcpp geometry_msgs dynamixel_sdk)
```

`ament_target_dependencies`에 `dynamixel_sdk`를 추가하면 아키텍처에 맞는 라이브러리와 헤더 경로가 자동으로 설정된다.

![image.png](https://github.com/user-attachments/assets/39314654-3059-43bf-8ae4-def0cfd03693)

---

### 2.4 yaml-cpp 패키지 설치

### 차이점

`sudo apt install libyaml-cpp-dev` 명령은 RPi5 Ubuntu에서도 동일하게 작동하며 별도의 수정 없이 적용 가능했다.

![image.png](https://github.com/user-attachments/assets/d7fb2d0b-d1a0-4b48-b850-d1227105c9e5)

---

### 2.5 U2D2 장치명 고정 (udev rules)

### 차이점

udev rules 설정 절차 자체는 동일하지만, 장치별 **serial 번호**가 보드마다 다르므로 RPi5에서 직접 확인하여 수정해야 했다.

**확인 명령:**

```bash
udevadm info -a -n /dev/ttyUSB0 | grep -E "idVendor|idProduct|serial"
udevadm info -a -n /dev/ttyUSB1 | grep -E "idVendor|idProduct|serial"
```

**99-tty.rules 수정 내용:**

| 장치 | 항목 | 원본 (Jetson) | 수정 (RPi5) |
| --- | --- | --- | --- |
| U2D2 | serial | FT7W93H0 | FT4TCY93 |
| rplidar | serial | 0001 | 3ec99fd24a5bed1198204ae81c62bc44 |

**수정된 99-tty.rules:**

```
SUBSYSTEM=="tty", ATTRS{idVendor}=="0403", ATTRS{idProduct}=="6014", ATTRS{serial}=="FT4TCY93", SYMLINK+="ttyUSBMotor"
SUBSYSTEM=="tty", ATTRS{idVendor}=="10c4", ATTRS{idProduct}=="ea60", ATTRS{serial}=="3ec99fd24a5bed1198204ae81c62bc44", SYMLINK+="ttyUSBLidar"
```

Vendor ID와 Product ID는 같은 제품이면 동일하지만, serial 번호는 개체별로 고유하므로 반드시 각 보드에서 확인이 필요하다.

![image.png](https://github.com/user-attachments/assets/f6f31199-46e8-4fae-9f7b-a3c0d7528946)

---

### 2.6 dynamixel_workbench_controllers 설치

ROS2용 소스 코드 git clone 이후 아래와 같이 CMakeLists.txt, 모든 소스 코드, yaml.cpp 파일을 수정하고 카페 강의 자료의 소스코드 수정사항은 **2.7 모터 변경 관련 수정**을 참고하면 됩니다.

### 문제 1: C++ 표준 및 SDK 링크 (빌드 에러)

3단계와 동일한 문제. C++14 → C++17 변경, `dxl_x64_cpp` → `find_package(dynamixel_sdk)` 변경이 필요했다.

### 문제 2: 하드코딩된 include 경로 삭제 (빌드 에러)

원본 CMakeLists.txt에 원래 개발 환경의 절대 경로가 하드코딩되어 있었다.

```
# 삭제 대상
include_directories(/home/ncslab/ros2_ws/install/dynamixel_workbench_msgs/include/)
include_directories(/usr/local/include/dynamixel_sdk)
```

`ament_target_dependencies`를 통해 `dynamixel_workbench_msgs`와 `dynamixel_sdk`의 include 경로가 자동으로 설정되므로 위 두 줄은 삭제해야 한다.

**수정된 CMakeLists.txt 주요 부분:**

```
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(dynamixel_workbench_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(dynamixel_sdk REQUIRED)

include_directories(include)

add_executable(find_dynamixel src/find_dynamixel.cpp src/dynamixel_driver.cpp
  src/dynamixel_item.cpp src/dynamixel_tool.cpp src/dynamixel_workbench.cpp)
ament_target_dependencies(find_dynamixel rclcpp dynamixel_sdk)
target_link_libraries(find_dynamixel yaml-cpp)

add_executable(dynamixel_workbench_controllers src/dynamixel_workbench_controllers.cpp
  src/dynamixel_driver.cpp src/dynamixel_item.cpp src/dynamixel_tool.cpp src/dynamixel_workbench.cpp)
ament_target_dependencies(dynamixel_workbench_controllers rclcpp dynamixel_workbench_msgs
  sensor_msgs dynamixel_sdk)
target_link_libraries(dynamixel_workbench_controllers yaml-cpp)
```

### 문제 3: 소스코드 내 절대 경로 하드코딩 (빌드 에러)

소스 파일들의 `#include` 지시문에 원래 개발 환경의 절대 경로가 하드코딩되어 있었다.

**에러 메시지:**

```
fatal error: /home/ncslab/ros2_ws/src/dynamixel_workbench_controllers/include/
dynamixel_workbench_controllers/dynamixel_driver.h: No such file or directory
```

**해결 방법:** `grep -rn "/home/ncslab" src/` 명령으로 해당 파일들을 모두 찾아 상대 경로로 변경했다.

```cpp
// 변경 전
#include "/home/ncslab/ros2_ws/src/dynamixel_workbench_controllers/include/dynamixel_workbench_controllers/dynamixel_driver.h"

// 변경 후
#include "dynamixel_workbench_controllers/dynamixel_driver.h"
```

이 문제는 `dynamixel_driver.cpp`, `dynamixel_item.cpp` 등 여러 소스 파일에서 동일하게 발생했다.

### 문제 4: yaml-cpp API 호환성 (빌드 에러)

새 버전의 yaml-cpp에서는 `YAML::Node`를 `NULL`과 직접 비교할 수 없어 컴파일 에러가 발생했다.

```cpp
// 변경 전 (173줄)
if (dynamixel == NULL)

// 변경 후
if (!dynamixel)
```

### 문제 5: YAML 설정 파일 경로 하드코딩 (런타임 에러)

빌드는 성공했지만 실행 시 YAML 설정 파일 경로가 원래 개발 환경으로 하드코딩되어 있어 에러가 발생했다.

**에러 메시지:**

```
terminate called after throwing an instance of 'YAML::BadFile'
  what():  bad file: /home/ncslab/ros2_ws/src/dynamixel_workbench_controllers/config/basic.yaml
```

**해결 방법:**

```cpp
// 변경 전
"/home/ncslab/ros2_ws/src/dynamixel_workbench_controllers/config/basic.yaml"

// 변경 후
"/home/rapi5/ros2_ws/src/dynamixel_workbench_controllers/config/basic.yaml"
```

### 문제 6: publishCallback Segmentation Fault (런타임 에러)

실행 직후 Segfault가 발생했다. GDB 디버깅 결과, `publishCallback` 함수(619줄)에서 `dynamixel_state_list_.dynamixel_state`가 아직 비어 있는 상태에서 접근하여 크래시가 발생한 것으로 확인되었다. `readCallback`이 데이터를 채우기 전에 `publishCallback`이 먼저 호출되는 타이밍 문제였다.

**GDB Backtrace:**

```
#0 DynamixelController::publishCallback at dynamixel_workbench_controllers.cpp:619
   effort = dxl_wb_->convertValue2Current(
     (int16_t)dynamixel_state_list_.dynamixel_state[id_cnt].present_current);
```

**해결 방법 — publishCallback 함수 초반부에 안전 체크 추가:**

```cpp
void DynamixelController::publishCallback()
{
  if (dynamixel_state_list_.dynamixel_state.empty()) return;
  // 기존 코드 계속...
}
```

---

### 2.7 모터 변경 관련 수정 (XC430-W150-T)

기존 가이드는 TurtleBot3 Burger(MX 시리즈) 기준이었으나, 실제 사용 모터가 XC430-W150-T로 변경되면서 추가 수정이 필요했다.

### dxl_nano 패키지 수정 (별도 패키지)

`dxl.hpp`에서도 동일한 수정이 필요했다.

```cpp
// 장치명 수정
#define DEVICENAME  "/dev/ttyUSBMotor"  // 기존: "/dev/ttyUSB0" - 72번 라인

// 모델 변경
#define DXL_MODEL   XC430W150           // 기존: MX12W - 33번 라인

// 보레이트는 모터 설정에 맞게 확인 필요
#define XL_BAUDRATE 4000000             // XC 시리즈용 - 71번 라인
```

### dynamixel_workbench_controllers.cpp 파라미터 수정

| 파라미터 | 원본 (Burger) | 수정 (Waffle) | 해당 라인 |
| --- | --- | --- | --- |
| 휠 사이 간격 | 0.160m | 0.287m | 68번 라인 |
| 휠 반지름 | 0.033m | 0.033m (동일) | 71번 라인 |
| 보레이트 | 2000000 | 400000 | 87번 라인 |
| 장치명 | /dev/ttyUSB0 | /dev/ttyUSBMotor | 86번 라인 |

### 실행 결과

operators의 명령에 따른 controllers의 제어가 수행되는 것을 확인 가능

![image.png](https://github.com/user-attachments/assets/a4371663-1598-4b4f-87a3-6784a702b159)

---

## 3. 마이그레이션 수행 후 고찰

1. **apt 설치 vs 소스 빌드의 차이**: apt로 설치한 ROS2 패키지는 `find_package()`로 자동 인식되므로 경로 하드코딩이나 직접 라이브러리 링크가 불필요하다. `ament_target_dependencies`를 활용하면 아키텍처(x86_64/aarch64)에 관계없이 올바른 라이브러리가 자동으로 링크된다.
2. **ROS2 Jazzy는 C++17을 요구한다**: `rclcpp`의 헤더 파일이 `std::variant`, `std::optional` 등 C++17 기능을 사용하므로 반드시 C++17 이상으로 설정해야 한다.
3. **절대 경로 하드코딩 주의**: 원본 소스코드에 특정 개발 환경의 절대 경로가 `#include`문과 파일 경로에 하드코딩되어 있으면 다른 환경에서 반드시 에러가 발생한다. `grep -rn` 명령으로 사전에 확인하는 것이 좋다.
4. **모터 변경 시 보레이트 확인 필수**: 새 모터의 공장 초기 보레이트와 코드에서 설정한 보레이트가 일치하지 않으면 통신이 되지 않는다. Dynamixel Wizard2로 실제 모터 설정을 먼저 확인하는 것이 가장 안전하다.
5. **GDB 디버깅 활용**: Segfault와 같은 런타임 에러는 `-cmake-args -DCMAKE_BUILD_TYPE=Debug`로 빌드 후 GDB backtrace를 통해 정확한 크래시 위치를 특정할 수 있다.

## 과제

(1) 젯슨보드에서 dynamixel_workbench_operators , dynamixel_workbench_controllers 패키지를 이용하여 로봇을 연구실을 1바퀴 회전하도록 원격제어해보시오. 이전에 작성한 카메라 패키지를 활용하여 카메라를 보면서 제어하시오.

[vedio1](https://youtu.be/DNKiD859GXk)

(2) dynamixel_workbench_operators 는 PC에서 실행하고, dynamixel_workbench_controllers 패키지는 젯슨보드에서 실행하여 (1)번 과제를 수행하시오.

[vedio2](https://youtu.be/lEghx8BfIXk)


(3) dynamixel_workbench_operators , dynamixel_workbench_controllers 에서 정의된 토픽, 서비스, 파라미터의 종류를 조사하고 각각의 기능을 설명하시오. topic, service, parameter관련 ros2명령어를 이용하여 조사하라.

- topic list

![image.png](https://github.com/user-attachments/assets/0549f931-179a-4992-b60e-069b0b5d9c6b)

- dynamixel 서비스 list

![image.png](https://github.com/user-attachments/assets/8b318a0f-309c-40ef-9800-c05344fdbe9c)

- dynamixel param list

![image.png](https://github.com/user-attachments/assets/258ba23a-0f97-4694-91e4-1b520c77d183)

![image.png](https://github.com/user-attachments/assets/3ba07873-a8ca-45e5-8635-53dd7c7ca296)

- topic 발행 주기

![image.png](https://github.com/user-attachments/assets/d489fc56-7eaa-40aa-836b-40775a000b93)

(4) rqt_graph를 통하여 각 노드(operators, controllers)사이에 전달되는 토픽을 파악하라.

![image.png](https://github.com/user-attachments/assets/ef1ca5b0-bea9-4f08-88eb-df25184610a5)

(5) (4)번에서 파악한 토픽의 내용물을 자세히 설명하시오.

cmd_vel : 모터 속도 명령

![image.png](https://github.com/user-attachments/assets/f8d4fd51-5217-4dd8-8e7d-b3c92222d712)

jotin_states : 모터 위치, 속도, 토크 상태

![image.png](https://github.com/user-attachments/assets/1f2f6bf4-f820-44de-9ceb-fbd5b8798f7f)

dynamixel_state : Dynamixel 고유 상태(온도, 전압, 에러)

![image.png](https://github.com/user-attachments/assets/ef679a15-fdd5-418d-8caa-95c6cdb882b8)

(6) wrapper class 가 무엇인가?

다른 객체나 기능을 감싸서 더 쉽게 사용할 수 있도록 만든 클래스

다음과 같은 장점이 있다.

1. 코드 재사용 : 매번 SDK 초기화 코드를 반복하지 않아도 된다.
2. 복잡도 감소 : 사용자 입장에서 내부 동작을 몰라도 된다.
3. 유지보수 용이 : SDK가 바뀌어도 wrapper 내부만 수정하면 외부 코드는 그대로 사용 가능하다.
4. 에러 처리 통합 : 에러 체크 로직을 wrapper 안에 한번만 작성하면 된다.

ex) dynamixel_workbench_controllers가 wrapper 역할로 dynamixel SDK의 저수준 통신을 감싸서, ROS2 토픽이나 서비스로 간편하게 모터를 제어할 수 있게 해준다.
