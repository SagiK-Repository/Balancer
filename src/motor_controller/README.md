# Motor Controller 패키지

로봇 모터 제어를 위한 ROS 패키지입니다. 홀센서를 이용한 엔코더, MCP4921 DAC를 통한 모터 제어, SPI/I2C 통신을 지원합니다.

## 개요

이 패키지는 다음과 같은 기능을 제공합니다:
- 듀얼 모터 제어 (2개의 독립적인 모터)
- 홀센서 기반 엔코더를 통한 속도 피드백
- MCP4921 DAC를 통한 아날로그 PWM 출력
- **6축 IMU 센서를 통한 지면과의 각도 측정** ⭐ (NEW)
  - **순수 가속도계 기반**: 적분이나 시간 누적 없는 즉시 계산
  - **중력 벡터 분석**: 지면과의 실제 기울기 직접 측정
  - **간단하고 안정적**: 드리프트나 누적 오차 없음
- SPI/I2C 통신 지원
- 버튼 입력 처리
- 실시간 오도메트리 계산

## 시스템 요구사항

### 하드웨어
- 라즈베리파이 또는 호환 SBC
- MCP4921 DAC 칩 (2개)
- 3상 모터 (홀센서 포함)
- **GY-521 (MPU6050) 6축 IMU 모듈** ⭐ (NEW)
  - 가속도계: ±4g 범위 (밸런싱 로봇에 최적화)
  - 자이로스코프: ±500°/s 범위
  - 샘플링 주파수: 100Hz
  - 디지털 로우패스 필터: 44Hz (노이즈 감소)
  - I2C 주소: 0x68 (기본값)
- GPIO 버튼 (증가/감소)

### 소프트웨어
- ROS Melodic/Noetic
- libgpiod 라이브러리
- CMake 3.0.2+
- C++11 이상

## 패키지 구조

```
motor_controller/
├── src/                    # 소스 파일
│   ├── motor_controller.cpp    # 메인 모터 컨트롤러
│   ├── HallSensorEncoder.cpp   # 홀센서 엔코더
│   ├── mcp4921.cpp            # MCP4921 DAC 드라이버
│   ├── spi_controller.cpp     # SPI 통신 컨트롤러
│   └── i2c_controller.cpp     # I2C 통신 컨트롤러 (IMU 각도 계산 포함)
├── include/motor_controller/   # 헤더 파일
│   ├── motor_controller.h
│   ├── HallSensorEncoder.h
│   ├── mcp4921.h
│   ├── spi_controller.h
│   └── i2c_controller.h       # IMU 각도 계산 기능 포함
├── launch/                 # 런치 파일
│   └── motor_controller.launch
├── CMakeLists.txt
├── package.xml
└── README.md
```

## 주요 기능

### 1. 모터 컨트롤러 (MotorController)
- 듀얼 모터 제어
- ROS 토픽을 통한 속도 명령 수신
- GPIO를 통한 방향 제어
- 실시간 속도 피드백

### 2. 홀센서 엔코더 (HallSensorEncoder)
- 3상 모터 홀센서 시퀀스 해석
- 정확한 위치 및 속도 계산
- 오도메트리 계산
- 노이즈 필터링

### 3. MCP4921 DAC 드라이버
- 12비트 아날로그 출력
- SPI 통신을 통한 제어
- 퍼센트 또는 전압 단위 출력

### 4. **PID 제어 시스템** ⭐ (NEW)
- **정밀한 속도 제어**: 목표 속도와 실제 속도 간의 오차를 최소화
- **적응형 제어**: Kp, Ki, Kd 게인을 통한 세밀한 조정
- **와인드업 방지**: 적분 항목의 포화 방지 기능
- **실시간 모니터링**: PID 출력값과 오차 실시간 표시
- **Raw/PID 모드 전환**: 직접 제어와 PID 제어 선택 가능

### 5. **IMU 자세 센서 (I2CController)** ⭐ (기존)
- **MPU6050 6축 IMU 센서 제어**
- **지면과의 각도 측정 (Roll, Pitch)**
- **순수 가속도계 기반 계산**:
  - 중력 벡터를 이용한 직접적인 기울기 측정
  - 적분이나 시간 누적 없는 즉시 계산
  - 드리프트나 누적 오차 없음
- **간단하고 안정적인 알고리즘**:
  - 복잡한 필터링 없이 순수 기하학적 계산
  - 실시간 응답성 우수
- **실시간 기울기 경고 시스템**

### 5. SPI/I2C 컨트롤러
- BitBang SPI 구현
- I2C 통신 지원
- GPIO 기반 저수준 제어

## 설치 및 빌드

### 1. 의존성 설치
```bash
sudo apt-get update
sudo apt-get install libgpiod-dev libgpiod2
```

### 2. 패키지 빌드
```bash
cd ~/catkin_ws
catkin_make
source devel/setup.bash
```

## 사용법

### 1. 런치 파일 실행
```bash
source devel/setup.bash
roslaunch motor_controller motor_controller.launch
```

### 2. 속도 명령 전송

#### 직접 명령
```bash
# 모터 0 속도 설정 (20 RPM)
rostopic pub /target_speed0 std_msgs/Float32 "data: 20.0"

# 모터 1 속도 설정 (-15 RPM, 역방향)
rostopic pub /target_speed1 std_msgs/Float32 "data: -15.0"
```

#### **PID 제어 테스트 노드 사용** ⭐ (NEW)
```bash
# 별도 터미널에서 테스트 노드 실행
rosrun motor_controller test_pid_control

# 키보드 명령:
# w/s: 전진/후진
# a/d: 좌회전/우회전  
# q/e: 속도 증가/감소 (5 RPM 단위)
# x: 정지
# h: 도움말
# ESC: 종료
```

### 3. 실제 속도 모니터링
```bash
# 실제 속도 확인
rostopic echo /actual_speed0
rostopic echo /actual_speed1
```

### 4. **GY-521 캘리브레이션 관리** ⭐ (NEW)

#### 자동 캘리브레이션 시스템
시스템이 시작될 때 자동으로 다음 과정을 수행합니다:

1. **기존 캘리브레이션 로드**: `/tmp/gy521_calibration.dat` 파일에서 이전 캘리브레이션 데이터 읽기
2. **유효성 검사**: 로드된 캘리브레이션의 정확성 검증
3. **필요시 재캘리브레이션**: 파일이 없거나 유효하지 않은 경우에만 새로 캘리브레이션 수행

#### 강제 재캘리브레이션
```bash
# launch 파일에서 강제 재캘리브레이션 활성화
<param name="force_recalibration" value="true"/>

# 또는 런타임에 파라미터 변경
rosparam set /force_recalibration true
roslaunch motor_controller motor_controller.launch
```

#### 캘리브레이션 파일 관리
```bash
# 캘리브레이션 파일 확인
cat /tmp/gy521_calibration.dat

# 캘리브레이션 파일 삭제 (다음 실행시 재캘리브레이션)
rm /tmp/gy521_calibration.dat

# 캘리브레이션 파일 백업
cp /tmp/gy521_calibration.dat ~/gy521_backup.dat
```

### 5. **실시간 모니터링** ⭐ (업데이트)
터미널에서 다음과 같은 로그를 실시간으로 확인할 수 있습니다:

#### 캘리브레이션 로그
```
[INFO]: 캘리브레이션 상태 확인 중...
[INFO]: 캘리브레이션 데이터 로드 완료:
[INFO]:   Accel offsets: X=245.3, Y=-128.7, Z=16384.2
[INFO]:   Gyro offsets: X=12.4, Y=-8.9, Z=3.2
[INFO]: 캘리브레이션 검증 성공: 중력 크기 9.79 m/s² (오차: 0.02)
[INFO]: 기존 캘리브레이션이 유효합니다. 재캘리브레이션을 건너뜁니다.
```

#### PID 제어 모드
```
[INFO]: Tilt R/P: -2.1/-0.8 deg [STABLE](95%) | Speed T/A: [20.0,15.0]/[19.8,14.9] | PID Out: [18.5,14.2] | Err: [0.2,0.1] | Temp: 23.4 C
[WARN]: Tilt warning R/P: 16.2/2.5 deg
```

#### Raw 제어 모드
```
[INFO]: Tilt R/P: -2.1/-0.8 deg [STABLE](95%) | Speed T/A: [20.0,15.0]/[19.8,14.9] | RAW Mode | Temp: 23.4 C
[ERROR]: DANGER! Severe tilt R/P: 35.7/2.5 deg
```

## 설정 매개변수

### GPIO 핀 설정
```xml
<param name="mosi_pin" value="16"/>
<param name="miso_pin" value="17"/>
<param name="clk_pin" value="18"/>
<param name="ss0_pin" value="19"/>
<param name="ss1_pin" value="20"/>
<param name="reverse0_pin" value="168"/>
<param name="reverse1_pin" value="13"/>
<param name="inc_button_pin" value="232"/>
<param name="dec_button_pin" value="15"/>
```

### 모터 설정
```xml
<param name="max_speed" value="100.0"/>
<param name="spi_speed" value="100000000"/>
<param name="gpio_chip_name" value="gpiochip0"/>
```

### 로봇 물리 매개변수
```cpp
#define WHEEL_RADIUS 0.142    // 바퀴 반지름 (m)
#define WHEEL_DISTANCE 0.375  // 바퀴 간격 (m)
```

### **PID 제어 설정** ⭐ (NEW)
```xml
<!-- PID 제어 활성화/비활성화 -->
<param name="use_pid_control" value="true"/>

<!-- 모터 0 PID 게인 -->
<param name="pid_kp_motor0" value="2.0"/>  <!-- 비례 게인 -->
<param name="pid_ki_motor0" value="0.5"/>  <!-- 적분 게인 -->
<param name="pid_kd_motor0" value="0.1"/>  <!-- 미분 게인 -->

<!-- 모터 1 PID 게인 -->
<param name="pid_kp_motor1" value="2.0"/>
<param name="pid_ki_motor1" value="0.5"/>
<param name="pid_kd_motor1" value="0.1"/>
```

### **GY-521 (MPU6050) 설정** ⭐ (업데이트)
```xml
<!-- GY-521 (MPU6050) 센서 설정 -->
<param name="i2c_device" value="/dev/i2c-1"/>
<param name="mpu6050_address" value="0x68"/>
<param name="sample_rate_hz" value="100"/>
<param name="accel_range" value="4"/>  <!-- ±4g -->
<param name="gyro_range" value="500"/> <!-- ±500°/s -->
<param name="dlpf_bandwidth" value="44"/> <!-- 44Hz 로우패스 필터 -->

<!-- 캘리브레이션 설정 -->
<param name="calibration_file_path" value="/tmp/gy521_calibration.dat"/>
<param name="force_recalibration" value="false"/> <!-- true시 강제 재캘리브레이션 -->
```

```cpp
// 하드코딩된 설정값
#define MPU6050_ADDR 0x68     // GY-521 기본 I2C 주소
std::string i2c_device = "/dev/i2c-1";  // I2C 디바이스

// 스케일 팩터 (새로운 설정)
double accel_scale = 8192.0;   // ±4g 설정 시
double gyro_scale = 65.5;      // ±500°/s 설정 시

// 경고 임계값
기울기 경고: ±15°
심각한 경고: ±30°
```

## ROS 토픽

### 구독 토픽
- `/target_speed0` (std_msgs/Float32): 모터 0 목표 속도 (%)
- `/target_speed1` (std_msgs/Float32): 모터 1 목표 속도 (%)

### 발행 토픽
- `/actual_speed0` (std_msgs/Float32): 모터 0 실제 속도 (%)
- `/actual_speed1` (std_msgs/Float32): 모터 1 실제 속도 (%)

## 하드웨어 연결

### MCP4921 DAC
```
MCP4921 #1 (모터 0):
- VDD: 3.3V
- VSS: GND
- CS: GPIO 19
- SCK: GPIO 18
- SDI: GPIO 16
- VOUT: 모터 드라이버 입력

MCP4921 #2 (모터 1):
- VDD: 3.3V
- VSS: GND
- CS: GPIO 20
- SCK: GPIO 18
- SDI: GPIO 16
- VOUT: 모터 드라이버 입력
```

### **GY-521 (MPU6050) IMU 모듈** ⭐ (NEW)
```
GY-521 모듈 연결:
- VCC: 3.3V 또는 5V (모듈에 전압 레귤레이터 내장)
- GND: GND
- SCL: I2C SCL (GPIO 3, Pin 5)
- SDA: I2C SDA (GPIO 2, Pin 3)
- XDA/XCL: 사용 안함 (보조 I2C)
- AD0: GND (I2C 주소 0x68, 3.3V 연결시 0x69)
- INT: 사용 안함 (인터럽트 핀, 선택적)

GY-521 모듈 특징:
- 온보드 3.3V 레귤레이터 (5V 입력 가능)
- 10kΩ 풀업 저항 내장 (SDA/SCL)
- LED 전원 표시등
- 소형 크기: 21mm x 16mm
```

### 홀센서 연결
```
모터 0 홀센서:
- 3상 홀센서 A, B, C → I2C 확장 보드

모터 1 홀센서:
- 3상 홀센서 A, B, C → I2C 확장 보드
```

### 버튼 연결
```
증가 버튼: GPIO 232
감소 버튼: GPIO 15
```

## 문제 해결

### 1. GPIO 권한 오류
```bash
sudo usermod -a -G gpio $USER
sudo chmod 666 /dev/gpiochip0
```

### 2. SPI 통신 오류
- SPI 버스가 활성화되어 있는지 확인
- 연결 상태 점검
- 전원 공급 확인

### 3. 홀센서 시퀀스 오류
- 홀센서 연결 확인
- 시퀀스 매핑 재확인
- 노이즈 필터 조정

### 4. **GY-521 센서 문제** ⭐ (NEW)
```bash
# I2C 디바이스 확인
sudo i2cdetect -y 1

# GY-521이 0x68 주소에 나타나야 함
# 나타나지 않으면:
# - 전원 연결 확인 (3.3V 또는 5V)
# - SDA/SCL 연결 확인 (GPIO 2, 3)
# - GY-521 모듈의 LED가 켜져 있는지 확인
# - AD0 핀이 GND에 연결되어 있는지 확인 (0x68 주소용)

# I2C 활성화 (라즈베리파이)
sudo raspi-config
# Interfacing Options -> I2C -> Enable

# I2C 속도 확인/조정 (필요시)
sudo nano /boot/config.txt
# dtparam=i2c_arm_baudrate=100000 추가

# GY-521 모듈 테스트
sudo i2cget -y 1 0x68 0x75  # WHO_AM_I 레지스터 (0x68 반환되어야 함)
```

### 5. **각도 계산 오류** ⭐ (NEW)
- 센서 캘리브레이션 시 로봇을 평평한 곳에 정지 상태로 두기
- 상보 필터 계수(alpha) 조정 (0.95-0.99)
- 센서 진동 방지를 위한 기계적 고정 확인

## **IMU 기능 상세 설명** ⭐ (NEW)

### 지원하는 각도
- **Roll**: 좌우 기울기 (X축 회전)
- **Pitch**: 앞뒤 기울기 (Y축 회전)  
- **Yaw**: 회전 방향 (Z축 회전)

### 각도 계산 방식
1. **가속도계**: 중력 벡터를 기준으로 Roll/Pitch 계산
2. **자이로스코프**: 각속도를 적분하여 각도 변화 추적
3. **상보 필터**: 두 센서 데이터를 융합하여 안정적인 각도 계산

### 경고 시스템
- **15° 이상**: 경고 메시지 출력
- **30° 이상**: 에러 메시지 출력 (위험 상황)

## 라이센스

이 프로젝트는 TODO 라이센스 하에 배포됩니다.

## 기여자

- 메인테이너: tbot3@todo.todo

## 참고 자료

- [MCP4921 데이터시트](https://ww1.microchip.com/downloads/en/DeviceDoc/22248a.pdf)
- [MPU6050 데이터시트](https://invensense.tdk.com/wp-content/uploads/2015/02/MPU-6000-Datasheet1.pdf) ⭐ (NEW)
- [libgpiod 문서](https://git.kernel.org/pub/scm/libs/libgpiod/libgpiod.git/about/)
- [ROS 공식 문서](http://wiki.ros.org/) 