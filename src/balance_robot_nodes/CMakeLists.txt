cmake_minimum_required(VERSION 3.0.2)
project(balance_robot_nodes)

## 컴파일러 설정
set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  sensor_msgs
  geometry_msgs
  message_generation
)

## 메시지 파일 생성
add_message_files(
  FILES
  MotorCommand.msg
  MotorStatus.msg
  SensorData.msg
  ControllerStatus.msg
)

## 메시지 의존성 생성
generate_messages(
  DEPENDENCIES
  std_msgs
  sensor_msgs
  geometry_msgs
)

## catkin 패키지 설정
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS roscpp rospy std_msgs sensor_msgs geometry_msgs message_runtime
  DEPENDS system_lib
)

## 헤더 파일 경로 설정
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

## 라이브러리 생성
add_library(${PROJECT_NAME}
  src/pid_controller.cpp
  src/smooth_controller.cpp
  src/hardware_interface.cpp
)

add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
)

## 실행 파일 생성

# SPI 하드웨어 노드
add_executable(spi_hardware_node src/spi_hardware_node.cpp)
add_dependencies(spi_hardware_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(spi_hardware_node
  ${PROJECT_NAME}
  ${catkin_LIBRARIES}
  gpiod
)

# I2C 센서 노드
add_executable(i2c_sensor_node src/i2c_sensor_node.cpp)
add_dependencies(i2c_sensor_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(i2c_sensor_node
  ${PROJECT_NAME}
  ${catkin_LIBRARIES}
)

# 밸런스 컨트롤러 노드
add_executable(balance_controller_node src/balance_controller_node.cpp)
add_dependencies(balance_controller_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(balance_controller_node
  ${PROJECT_NAME}
  ${catkin_LIBRARIES}
)

# 모터 컨트롤러 브리지 노드 (기존 motor_controller 호환성)
add_executable(motor_controller_bridge src/motor_controller_bridge.cpp)
add_dependencies(motor_controller_bridge ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(motor_controller_bridge
  ${catkin_LIBRARIES}
)

## 설치 설정
install(TARGETS ${PROJECT_NAME} spi_hardware_node i2c_sensor_node balance_controller_node motor_controller_bridge
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
  PATTERN ".svn" EXCLUDE
)

install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
  PATTERN ".svn" EXCLUDE
)

install(DIRECTORY config/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/config
  PATTERN ".svn" EXCLUDE
) 