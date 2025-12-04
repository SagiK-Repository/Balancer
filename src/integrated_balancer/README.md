# Integrated Balancer

High-Performance Balancing Robot (Native Core + ROS Bridge)

## 개요

이 패키지는 기존의 분산 노드 구조를 단일 프로세스 + 멀티 스레드 구조로 통합하여 성능 병목을 해결한 통합 밸런서입니다.

## 주요 특징

- **Native Control Core**: ROS 헤더 없이 동작하는 순수 제어 루프
- **Real-Time Thread**: SCHED_FIFO 우선순위 90으로 100Hz 고정 주기 실행
- **Hardware SPI**: Bit-Banging 대신 Linux 커널 SPI 드라이버 사용
- **ROS Bridge**: 외부 통신(텔레메트리, 리모트 컨트롤)만 담당

## 빌드

```bash
cd ~/catkin_ws
catkin_make
source devel/setup.bash
```

## 실행

```bash
# Real-Time 권한 설정 (필요시)
sudo setcap cap_sys_nice+ep ~/catkin_ws/devel/lib/integrated_balancer/integrated_balancer

# 실행
rosrun integrated_balancer integrated_balancer
```

## 하드웨어 테스트

하드웨어 테스트 가이드는 [하드웨어 테스트 가이드](../../docs/개발과정/하드웨어%20테스트%20가이드.md)를 참조하세요.

### 빠른 테스트

```bash
# 전체 하드웨어 테스트
rosrun integrated_balancer hardware_test all

# 개별 테스트
rosrun integrated_balancer hardware_test spi /dev/spidev0.0
rosrun integrated_balancer hardware_test i2c /dev/i2c-1
rosrun integrated_balancer hardware_test mpu6050
rosrun integrated_balancer hardware_test hardware_spi /dev/spidev0.0
rosrun integrated_balancer hardware_test dac /dev/spidev0.0 19
rosrun integrated_balancer hardware_test gpio 168 out
```

## ROS 토픽

### 발행되는 토픽

- `/sensor_roll` (std_msgs/Float32): 현재 Roll 각도
- `/imu/data` (sensor_msgs/Imu): IMU 데이터
- `/system_ready` (std_msgs/Bool): 시스템 준비 상태

### 구독하는 토픽

- `/cmd_vel` (geometry_msgs/Twist): 속도 명령

## 파라미터

- `pid_kp_balance` (double, 기본값: 9.0): 밸런싱 PID 비례 게인
- `pid_ki_balance` (double, 기본값: 2.0): 밸런싱 PID 적분 게인
- `pid_kd_balance` (double, 기본값: 0.5): 밸런싱 PID 미분 게인
- `target_roll_angle` (double, 기본값: -0.7): 목표 Roll 각도 (도)

## 파일 구조

```
integrated_balancer/
├── CMakeLists.txt
├── package.xml
├── README.md
├── include/
│   └── integrated_balancer/
│       ├── hardware_spi.h
│       ├── i2c_controller.h
│       ├── mcp4921.h
│       ├── pid_controller.h
│       └── balancer_core.h
├── src/
│   ├── hardware_spi.cpp
│   ├── i2c_controller.cpp
│   ├── mcp4921.cpp
│   ├── pid_controller.cpp
│   ├── balancer_core.cpp
│   └── main.cpp
└── test/
    └── hardware_test.cpp
```

## 참고사항

- SPI 디바이스 경로는 하드웨어에 맞게 조정 필요 (`/dev/spidev0.0`)
- CS 핀 번호는 하드웨어에 맞게 조정 필요 (DAC 초기화 시)
- Real-Time 권한 설정이 필요할 수 있음

## 라이선스

MIT




