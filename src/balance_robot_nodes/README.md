# 밸런싱 로봇 노드 분리 프로젝트

이 프로젝트는 기존의 단일 ROS 노드 `motor_controller`를 3개의 전문화된 노드로 분리하여 밸런싱 로봇의 모듈성과 유지보수성을 향상시키는 것을 목표로 합니다.

## 📋 프로젝트 개요

### 기존 구조 (Before)
```
motor_controller (단일 노드)
├── SPI 모터 제어
├── I2C 센서 읽기  
└── PID 제어 로직
```

### 새로운 구조 (After)
```
balance_robot_nodes (분리된 패키지)
├── spi_hardware_node     (SPI 하드웨어 제어)
├── i2c_sensor_node       (I2C 센서 읽기)
└── balance_controller_node (밸런싱 제어 로직)
```

## 🏗️ 시스템 아키텍처

```
┌─────────────────────┐     /sensor_data      ┌─────────────────────┐
│                     │◄──────────────────────│                     │
│ Balance Controller  │                       │  I2C Sensor Node    │
│      Node           │                       │                     │
│                     │                       │ • IMU 센서 읽기      │
└─────────────────────┘                       │ • 각도 계산         │
         │                                    │ • 필터링           │
         │ /motor_command                     └─────────────────────┘
         ▼                                           
┌─────────────────────┐     /motor_status     
│ SPI Hardware Node   │◄──────────────────────┐
│                     │                       │
│ • 모터 출력 제어     │                       │
│ • 홀센서 읽기       │                       │
│ • SPI/GPIO 통신     │                       │
└─────────────────────┘                       │
```

## 📦 패키지 구성

### 1. 메시지 타입 (4개)
- **MotorCommand.msg**: 모터 제어 명령
- **MotorStatus.msg**: 모터 상태 정보
- **SensorData.msg**: IMU 센서 데이터
- **ControllerStatus.msg**: 컨트롤러 상태

### 2. 공통 라이브러리 (6개)
- **pid_controller**: PID 제어기 클래스
- **smooth_controller**: 부드러운 모터 제어
- **hardware_interface**: 하드웨어 공통 인터페이스

### 3. 노드 (3개)
- **spi_hardware_node**: SPI 하드웨어 제어 노드
- **i2c_sensor_node**: I2C 센서 노드
- **balance_controller_node**: 밸런스 컨트롤러 노드

### 4. 설정 및 런치 파일 (5개)
- **balance_robot_params.yaml**: 기본 매개변수
- **balance_robot.launch**: 메인 런치 파일
- **test_hardware_only.launch**: 하드웨어 테스트
- **test_sensors_only.launch**: 센서 테스트

## 🚀 빌드 및 실행

### 빌드
```bash
cd ~/catkin_ws
catkin_make
source devel/setup.bash
```

### 실행
```bash
# 전체 시스템 실행
roslaunch balance_robot_nodes balance_robot.launch

# 하드웨어만 테스트
roslaunch balance_robot_nodes test_hardware_only.launch

# 센서만 테스트
roslaunch balance_robot_nodes test_sensors_only.launch

# 시각화 포함 실행
roslaunch balance_robot_nodes balance_robot.launch enable_visualization:=true
```

## 📊 ROS 토픽 구조

### 발행되는 토픽
- `/balance_robot/motor_status` (MotorStatus)
- `/balance_robot/sensor_data` (SensorData)
- `/balance_robot/controller_status` (ControllerStatus)
- `/balance_robot/system_ready` (std_msgs/Bool)
- `/balance_robot/imu/data` (sensor_msgs/Imu)

### 구독하는 토픽
- `/balance_robot/motor_command` (MotorCommand)
- `/balance_robot/cmd_vel` (geometry_msgs/Twist)
- `/balance_robot/target_speed0` (std_msgs/Float32)
- `/balance_robot/target_speed1` (std_msgs/Float32)
- `/balance_robot/emergency_stop` (std_msgs/Bool)

## ⚙️ 설정 매개변수

### SPI Hardware Node
```yaml
spi_hardware_node:
  spi_speed: 100000000        # SPI 통신 속도 (Hz)
  control_frequency: 500.0    # 제어 주기 (Hz)
  reverse0_pin: 23           # 모터0 방향 제어 핀
  reverse1_pin: 24           # 모터1 방향 제어 핀
```

### I2C Sensor Node
```yaml
i2c_sensor_node:
  filter_enabled: true           # 필터 활성화
  filter_alpha: 0.3             # 필터 계수
  use_complementary_filter: true # 상보 필터 사용
  stability_threshold: 0.5      # 안정성 임계값
```

### Balance Controller Node
```yaml
balance_controller_node:
  use_pid_control: true         # PID 제어 활성화
  use_balance_control: false    # 밸런싱 제어 (개발 중)
  max_speed: 100.0             # 최대 속도 (%)
  control_frequency: 500.0      # 제어 주기 (Hz)
  
  # PID 게인
  pid_kp_motor0: 2.0
  pid_ki_motor0: 0.5
  pid_kd_motor0: 0.1
```

## 🔧 테스트 및 디버깅

### 1. 하드웨어 테스트
```bash
# SPI 하드웨어 노드만 실행
roslaunch balance_robot_nodes test_hardware_only.launch

# 모터 명령 수동 발행
rostopic pub /motor_command balance_robot_nodes/MotorCommand \
  '{motor0_output: 10.0, motor1_output: 10.0, 
    motor0_direction: true, motor1_direction: true, emergency_stop: false}'

# 모터 상태 확인
rostopic echo /motor_status
```

### 2. 센서 테스트
```bash
# I2C 센서 노드만 실행
roslaunch balance_robot_nodes test_sensors_only.launch

# 센서 데이터 확인
rostopic echo /sensor_data
rostopic echo /sensor_roll
```

### 3. 제어 시스템 테스트
```bash
# 목표 속도 설정
rostopic pub /balance_robot/target_speed0 std_msgs/Float32 "data: 20.0"
rostopic pub /balance_robot/target_speed1 std_msgs/Float32 "data: 20.0"

# cmd_vel로 제어
rostopic pub /balance_robot/cmd_vel geometry_msgs/Twist \
  '{linear: {x: 0.1, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.1}}'

# 비상 정지
rostopic pub /balance_robot/emergency_stop std_msgs/Bool "data: true"
```

## 📈 성능 특징

### 제어 성능
- **제어 주기**: 500Hz (2ms)
- **센서 읽기**: 100Hz (10ms)
- **PID 제어**: 적분 와인드업 방지, 출력 제한
- **부드러운 제어**: 가속도 제한, 이중 저역 통과 필터

### 안전 기능
- **비상 정지**: 즉시 모든 모터 정지
- **데이터 타임아웃**: 통신 장애 시 자동 정지
- **각도 제한**: 위험 각도에서 자동 정지
- **속도 제한**: 최대 속도 초과 시 보호

### 모니터링
- **실시간 상태**: 모터, 센서, 컨트롤러 상태 모니터링
- **진단 기능**: 5초마다 시스템 상태 출력
- **에러 로깅**: 하드웨어 오류 및 통신 장애 기록

## 🛠️ 하드웨어 요구사항

### 필수 구성품
- **Jetson Nano**: 메인 컴퓨터
- **SPI DAC**: MCP4922 (모터 제어용)
- **I2C IMU**: MPU6050 또는 유사 센서
- **GPIO**: 방향 제어 및 버튼 입력
- **홀센서**: 속도 피드백용

### 핀 배치
```
GPIO 23 → 모터0 방향 제어
GPIO 24 → 모터1 방향 제어
GPIO 18 → SPI 래치 신호
GPIO 5  → 증가 버튼
GPIO 6  → 감소 버튼
SPI0    → DAC 통신
I2C1    → IMU 센서
```

## 🔍 트러블슈팅

### 자주 발생하는 문제

1. **SPI 통신 오류**
   ```bash
   # SPI 활성화 확인
   lsmod | grep spi
   ls -la /dev/spi*
   ```

2. **I2C 통신 오류**
   ```bash
   # I2C 활성화 확인
   i2cdetect -y 1
   ```

3. **권한 문제**
   ```bash
   # 사용자를 gpio 그룹에 추가
   sudo usermod -a -G gpio $USER
   ```

4. **높은 CPU 사용률**
   - 제어 주파수를 낮춤 (500Hz → 250Hz)
   - 불필요한 로그 출력 제거

## 📝 개발 히스토리

### 완성된 작업
✅ **패키지 구조 생성** (2024-12-19)  
✅ **메시지 타입 정의** (4개)  
✅ **공통 라이브러리 구현** (6개)  
✅ **노드 헤더 파일** (3개)  
✅ **SPI Hardware Node 구현** (완료)  
✅ **I2C Sensor Node 구현** (완료)  
✅ **Balance Controller Node 구현** (완료)  
✅ **설정 파일 및 런치 파일** (5개)  
✅ **테스트 스크립트** (3개)  

### 향후 개발 계획
🔄 **밸런싱 제어 활성화**  
🔄 **자동 보정 기능**  
🔄 **웹 인터페이스 추가**  
🔄 **데이터 로깅 기능**  

## 👥 기여자

- **개발자**: AI Assistant + 사용자
- **프로젝트 기간**: 2024-12-19
- **개발 환경**: ROS Noetic, Ubuntu 20.04

## 📄 라이센스

이 프로젝트는 MIT 라이센스 하에 제공됩니다.

---

**📞 지원 문의**: 프로젝트 관련 문의사항이 있으시면 Issues를 통해 연락주세요. 