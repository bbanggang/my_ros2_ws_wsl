cmake_minimum_required(VERSION 3.8)
project(pub1-1)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

# add_executable(실행파일명 소스파일경로)
add_executable(pub1-1 src/pub1-1.cpp)
# 의존성 연결 (rclcpp, std_msgs 라이브러리를 실행 파일에 연결)
ament_target_dependencies(pub1-1 rclcpp std_msgs)

# 빌드 후 실행 파일을 install 폴더로 복사
install(TARGETS
  pub1-1
  DESTINATION lib/${PROJECT_NAME})

ament_package()
