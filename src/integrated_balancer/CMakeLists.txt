cmake_minimum_required(VERSION 3.0.2)
project(integrated_balancer)

## 컴파일러 설정
set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  sensor_msgs
  geometry_msgs
)

## catkin 패키지 설정
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS roscpp std_msgs sensor_msgs geometry_msgs
)

## 헤더 파일 경로 설정
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

## 라이브러리 생성
add_library(${PROJECT_NAME}
  src/hardware_spi.cpp
  src/i2c_controller.cpp
  src/mcp4921.cpp
  src/pid_controller.cpp
  src/balancer_core.cpp
)

target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
  gpiod
)

add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## 실행 파일 생성
add_executable(integrated_balancer_node src/main.cpp)
target_link_libraries(integrated_balancer_node
  ${PROJECT_NAME}
  ${catkin_LIBRARIES}
  gpiod
  rt
)

add_dependencies(integrated_balancer ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## 하드웨어 테스트 실행 파일 생성
add_executable(hardware_test test/hardware_test.cpp)
target_link_libraries(hardware_test
  ${PROJECT_NAME}
  ${catkin_LIBRARIES}
  gpiod
  rt
)

add_dependencies(hardware_test ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Real-Time 스케줄링을 위한 권한 설정 (참고용)
# 실행 시 sudo 권한이 필요할 수 있음:
# sudo setcap cap_sys_nice+ep devel/lib/integrated_balancer/integrated_balancer
# 또는 /etc/security/limits.conf에 다음 추가:
# user soft rtprio 99
# user hard rtprio 99

