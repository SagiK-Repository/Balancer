cmake_minimum_required(VERSION 3.0.2)
project(balancing_robot)

# ROS 패키지 찾기
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  sensor_msgs
  geometry_msgs
)

# catkin 패키지 설정
catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS roscpp rospy std_msgs sensor_msgs geometry_msgs
)

# 포함 디렉토리
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

# 실행 파일 빌드
add_executable(motor_test src/motor_control/motor_test.cpp)
target_link_libraries(motor_test ${catkin_LIBRARIES})