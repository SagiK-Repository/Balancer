# Jetson Nano 밸런싱 로봇 원격 개발 가이드
Windows에서 개발하고 Jetson Nano에 배포하는 방식의 가이드입니다.

## 1. Jetson Nano 설정하기

### 1.1 JetPack 설치
- SD 카드에 JetPack 4.6.1 이미지 플래시
- 초기 설정 및 WiFi 연결
- 시스템 업데이트:
```bash
sudo apt update && sudo apt upgrade -y
```

### 1.2 원격 접속 설정
```bash
# SSH 서버 설치 및 활성화
sudo apt install openssh-server
sudo systemctl enable ssh
sudo systemctl start ssh

# IP 주소 확인
ip addr show
```

### 1.3 ROS 설치
```bash
# 필수 의존성
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
sudo apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654

# ROS Melodic 설치
sudo apt update
sudo apt install ros-melodic-desktop-full

# 환경 설정
echo "source /opt/ros/melodic/setup.bash" >> ~/.bashrc
source ~/.bashrc

# rosdep 설정
rosdep update  # init은 이미 되어있음
```

### 1.4 catkin 작업공간 설정
```bash
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws/
catkin_make
echo "source ~/catkin_ws/devel/setup.bash" >> ~/.bashrc
```

### 1.5 하드웨어 라이브러리 설치
```bash
# GPIO 및 I2C 설정
sudo pip install --upgrade jetson-gpio
sudo pip install smbus2

# 사용자 권한 설정
sudo usermod -a -G i2c $USER
sudo usermod -a -G gpio $USER
```

## 2. Windows 개발 환경 설정

### 2.1 필수 도구 설치
- **VS Code**: https://code.visualstudio.com/download
- **CMake for Windows**: https://cmake.org/download/
- **Git for Windows**: https://git-scm.com/download/win
- **PuTTY**: SSH 접속용
- **WinSCP**: 파일 전송용

### 2.2 VS Code 확장 프로그램
- Remote - SSH
- C/C++ Extension
- CMake Tools
- ROS
- Python

### 2.3 원격 개발 설정
1. VS Code에서 Remote-SSH 확장 설치
2. F1 -> "Remote-SSH: Connect to Host" 선택
3. Jetson Nano SSH 정보 입력 (ubuntu@JETSON_IP)

## 3. 프로젝트 구조 생성

### Windows에서 작업:
```bash
# VS Code 터미널에서 실행
mkdir balancing_robot
cd balancing_robot

# 디렉토리 구조
balancing_robot/
├── CMakeLists.txt
├── package.xml
├── src/
│   ├── motor_control/
│   │   └── motor_test.cpp
│   ├── sensor_interface/
│   └── balance_algorithm/
├── include/
│   └── balancing_robot/
├── launch/
├── scripts/
│   └── sensor_test.py
└── config/
```

### 3.1 CMakeLists.txt (Windows에서 작성)
```cmake
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
```

### 3.2 src/motor_control/motor_test.cpp
```cpp
#include <ros/ros.h>
#include <iostream>

// Jetson GPIO를 시스템 콜로 제어
void set_gpio(int pin, bool value) {
    std::ofstream export_file("/sys/class/gpio/export");
    export_file << pin;
    export_file.close();
    
    std::string direction_path = "/sys/class/gpio/gpio" + std::to_string(pin) + "/direction";
    std::ofstream direction_file(direction_path);
    direction_file << "out";
    direction_file.close();
    
    std::string value_path = "/sys/class/gpio/gpio" + std::to_string(pin) + "/value";
    std::ofstream value_file(value_path);
    value_file << (value ? "1" : "0");
    value_file.close();
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "motor_test");
    ros::NodeHandle nh;
    
    ROS_INFO("Motor Test Started");
    
    // 테스트 코드
    int test_pin = 161; // J41 핀 7번
    for(int i = 0; i < 5; i++) {
        set_gpio(test_pin, true);
        ros::Duration(0.5).sleep();
        set_gpio(test_pin, false);
        ros::Duration(0.5).sleep();
    }
    
    return 0;
}
```

### 3.3 scripts/sensor_test.py
```python
#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import Imu
import smbus2
import time

class MPU6050:
    def __init__(self, bus_number=1, address=0x68):
        self.bus = smbus2.SMBus(bus_number)
        self.address = address
        # MPU6050 초기화
        self.bus.write_byte_data(self.address, 0x6B, 0)  # Power management
        
    def read_raw_data(self, addr):
        high = self.bus.read_byte_data(self.address, addr)
        low = self.bus.read_byte_data(self.address, addr+1)
        value = ((high << 8) | low)
        if value > 32768:
            value = value - 65536
        return value
    
    def get_accel_data(self):
        acc_x = self.read_raw_data(0x3B) / 16384.0
        acc_y = self.read_raw_data(0x3D) / 16384.0
        acc_z = self.read_raw_data(0x3F) / 16384.0
        return acc_x, acc_y, acc_z
    
    def get_gyro_data(self):
        gyro_x = self.read_raw_data(0x43) / 131.0
        gyro_y = self.read_raw_data(0x45) / 131.0
        gyro_z = self.read_raw_data(0x47) / 131.0
        return gyro_x, gyro_y, gyro_z

def main():
    rospy.init_node('sensor_test')
    pub = rospy.Publisher('/imu/data', Imu, queue_size=10)
    
    mpu = MPU6050(bus_number=1)
    rate = rospy.Rate(100)
    
    while not rospy.is_shutdown():
        accel_data = mpu.get_accel_data()
        gyro_data = mpu.get_gyro_data()
        
        imu_msg = Imu()
        imu_msg.header.stamp = rospy.Time.now()
        imu_msg.header.frame_id = "imu"
        
        # 가속도계 데이터 (m/s^2)
        imu_msg.linear_acceleration.x = accel_data[0] * 9.81
        imu_msg.linear_acceleration.y = accel_data[1] * 9.81
        imu_msg.linear_acceleration.z = accel_data[2] * 9.81
        
        # 자이로스코프 데이터 (rad/s)
        imu_msg.angular_velocity.x = gyro_data[0]
        imu_msg.angular_velocity.y = gyro_data[1]
        imu_msg.angular_velocity.z = gyro_data[2]
        
        pub.publish(imu_msg)
        
        rospy.loginfo("Accel: %.2f, %.2f, %.2f", accel_data[0], accel_data[1], accel_data[2])
        rate.sleep()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
```

## 4. 개발 및 배포 프로세스

### 4.1 원격 개발 (Option 1 - VS Code Remote)
1. VS Code에서 "Remote-SSH: Connect to Host" 실행
2. Jetson Nano에 직접 연결하여 개발
3. 터미널에서 빌드 및 테스트

### 4.2 로컬 개발 + 배포 (Option 2 - 추천)
Windows에서 개발:
```bash
# Git 작업 디렉토리 생성
cd <PROJECT_DIR>
git init
git add .
git commit -m "Initial commit"
```

Jetson Nano에 배포:
```bash
# WinSCP로 파일 전송 또는 Git 사용
# PuTTY로 SSH 접속 후:
cd ~/catkin_ws/src
git clone https://github.com/YOUR_USERNAME/balancing_robot.git

# 빌드
cd ~/catkin_ws
catkin_make

# 실행 권한 설정
chmod +x src/balancing_robot/scripts/sensor_test.py
```

### 4.3 배포 자동화 (PowerShell 스크립트)
```powershell
# deploy.ps1
$JETSON_IP = "192.168.x.x"
$JETSON_USER = "ubuntu"

# SCP로 파일 전송
pscp -r balancing_robot ubuntu@${JETSON_IP}:~/catkin_ws/src/

# SSH로 빌드 명령 실행
plink ubuntu@${JETSON_IP} "cd ~/catkin_ws && catkin_make"
```

## 5. 테스트 및 디버깅

### 5.1 원격 테스트
```bash
# Jetson Nano에서 실행
roslaunch balancing_robot balancing_robot.launch

# Windows에서 RQT 연결 (ROS_MASTER_URI 설정 필요)
set ROS_MASTER_URI=http://JETSON_IP:11311
rqt
```

### 5.2 launch 파일 생성
```xml
<!-- launch/balancing_robot.launch -->
<launch>
  <node name="motor_test" pkg="balancing_robot" type="motor_test" output="screen"/>
  <node name="sensor_test" pkg="balancing_robot" type="sensor_test.py" output="screen"/>
  <node name="rviz" pkg="rviz" type="rviz" args="-d $(find balancing_robot)/config/balancing_robot.rviz" required="false"/>
</launch>
```

## 6. 문제 해결

### 6.1 원격 접속 문제
- WiFi 네트워크 확인
- Jetson Nano의 IP 주소 확인
- 방화벽 설정 확인

### 6.2 빌드 문제
```bash
# 의존성 문제 해결
rosdep install --from-paths src --ignore-src -r -y

# 클린 빌드
cd ~/catkin_ws
rm -rf build devel
catkin_make
```

### 6.3 GPIO 권한 문제
```bash
# 현재 사용자를 gpio 그룹에 추가
sudo usermod -a -G gpio $USER

# I2C 그룹 추가
sudo usermod -a -G i2c $USER

# 재로그인 필요
```

## 7. 개발 팁

1. **코드 동기화**: Git을 사용한 버전 관리 추천
2. **환경 변수 설정**: .env 파일로 로컬/원격 설정 분리
3. **자동화 스크립트**: 배포 과정을 PowerShell/Batch 파일로 자동화
4. **네트워크 설정**: ROS_MASTER_URI를 통한 분산 시스템 구축

이 방식으로 Windows에서 편하게 개발하고 Jetson Nano에서 실행할 수 있습니다.











# Jetson Nano JetPack 설치 완전 가이드

## 1. 필요한 준비물

### 하드웨어
- Jetson Nano Developer Kit
- microSD 카드 (32GB 이상 권장, Class 10)
- USB 키보드/마우스
- HDMI 모니터
- 이더넷 케이블 또는 WiFi 동글
- 5V/4A 전원 어댑터

### 소프트웨어
- SD 카드 포맷터: [SD Card Formatter](https://www.sdcard.org/downloads/formatter/)
- 이미지 플래싱 도구: [Etcher](https://www.balena.io/etcher/) 또는 Win32 Disk Imager

## 2. JetPack 이미지 다운로드

### 2.1 NVIDIA 개발자 사이트 접속
1. [NVIDIA JetPack SDK](https://developer.nvidia.com/embedded/jetpack) 방문
2. NVIDIA 계정으로 로그인 (없으면 가입 필요)

### 2.2 이미지 다운로드
- **Jetson Nano**: `SD Card Image` 클릭
- **JetPack 4.6.1** 선택 (Ubuntu 18.04 기반, 가장 안정적)
- `jp461-xvier-sd-card-image.zip` 다운로드 (약 8GB)

## 3. SD 카드 준비

### 3.1 SD 카드 포맷
1. SD Card Formatter 실행
2. SD 카드 드라이브 선택
3. "Format option" → "Overwrite format" 선택
4. Format 실행

### 3.2 이미지 플래싱 (Etcher 사용)
1. Etcher 실행
2. "Flash from file" → 다운로드한 `.zip` 파일 선택
3. "Select target" → SD 카드 선택
4. "Flash!" 클릭
5. 플래싱 완료까지 대기 (약 15-20분)

### 3.3 이미지 플래싱 (Win32 Disk Imager 사용)
1. `.zip` 파일 압축 해제 → `.img` 파일 획득
2. Win32 Disk Imager 실행
3. Image File: `.img` 파일 선택
4. Device: SD 카드 드라이브 선택
5. "Write" 클릭
6. 완료까지 대기

## 4. Jetson Nano 초기 설정

### 4.1 하드웨어 조립
1. SD 카드를 Jetson Nano에 삽입
2. 키보드, 마우스, 모니터 연결
3. 이더넷 케이블 연결 (또는 WiFi 준비)
4. 점퍼 J48을 사용하여 배럴 잭 전원 활성화
5. 전원 어댑터 연결

### 4.2 첫 부팅 및 설정
1. 전원 연결 후 자동 부팅 시작
2. NVIDIA 로고 화면 표시
3. 언어 선택: 한국어 또는 English
4. 키보드 레이아웃 선택
5. 시간대 설정: Seoul

### 4.3 사용자 계정 설정
1. 호스트명 입력: `jetson-nano` (또는 원하는 이름)
2. 사용자 이름: `ubuntu` (권장)
3. 비밀번호 설정 (기억해야 함!)
4. 컴퓨터 이름 설정

### 4.4 라이선스 동의
- NVIDIA Jetson 소프트웨어 라이선스 동의

### 4.5 Network 설정
1. WiFi 설정 (USB WiFi 어댑터 사용 시)
2. 이더넷 자동 연결 확인

### 4.6 APP 파티션 확장
- "Would you like to continue with the configuration?" → "Yes"
- SD 카드의 모든 공간을 사용하도록 파티션 확장

### 4.7 전력 모드 선택
1. MAXN (최대 성능) 또는 5W (저전력)
2. 개발 시: MAXN 권장

### 4.8 설정 완료
- 재부팅 후 데스크톱 환경 진입

## 5. 초기 설정 완료 후 작업

### 5.1 시스템 업데이트
```bash
sudo apt update
sudo apt upgrade -y
sudo apt autoremove
```

### 5.2 필수 개발 도구 설치
```bash
# 개발 도구
sudo apt install -y build-essential git cmake

# Python 및 pip
sudo apt install -y python3-pip
sudo pip3 install --upgrade pip

# 네트워크 도구
sudo apt install -y net-tools openssh-server
```

### 5.3 원격 접속 설정
```bash
# SSH 서버 활성화
sudo systemctl enable ssh
sudo systemctl start ssh

# IP 주소 확인
ip addr show

# 원격 데스크톱 설정 (선택사항)
sudo apt install -y xrdp
sudo systemctl enable xrdp
```

### 5.4 JetPack 버전 확인
```bash
# JetPack 버전 확인
sudo apt show nvidia-jetpack

# CUDA 설치 확인
nvcc --version
```

### 5.5 GPU 메모리 설정 (선택사항)
만약 GPU 메모리 부족 시:
```bash
# /boot/extlinux/extlinux.conf 편집
sudo nano /boot/extlinux/extlinux.conf

# APPEND 줄에 추가:
APPEND ${cbootargs} quiet root=/dev/mmcblk0p1 rw rootwait rootfstype=ext4 console=ttyS0,115200n8 console=tty0 fbcon=map:0 net.ifnames=0 videooutput=off nvdlamode=emi
```

## 6. 문제 해결

### 6.1 부팅 실패
- SD 카드 재플래싱
- 전원 어댑터 확인 (5V/4A 이상)
- J48 점퍼 위치 확인

### 6.2 디스플레이 문제
```bash
# 해상도 변경
xrandr --output HDMI-0 --mode 1920x1080
```

### 6.3 WiFi 문제
```bash
# WiFi 카드 확인
lsusb
ifconfig

# 재시작 후 연결
sudo systemctl restart NetworkManager
```

### 6.4 SSH 접속 불가
```bash
# 방화벽 확인
sudo ufw status
sudo ufw allow ssh
sudo ufw enable
```

## 7. 성능 확인

### 7.1 시스템 정보
```bash
# 시스템 정보
jtop

# GPU 정보
nvidia-smi
```

### 7.2 벤치마크
```bash
# CPU 벤치마크
sysbench cpu --cpu-max-prime=20000 run

# GPU 테스트
cuda-install-samples-10.2.sh ~/cuda-samples
cd ~/cuda-samples/NVIDIA_CUDA-10.2_Samples/1_Utilities/deviceQuery
make
./deviceQuery
```

## 8. 백업 생성

설정 완료 후 SD 카드 이미지 백업:
```bash
# Linux에서
sudo dd if=/dev/sdb of=jetson-nano-backup.img bs=4M status=progress

# Windows에서 (Win32 Disk Imager)
# "Read" 버튼으로 이미지 생성
```

설치가 완료되면 다음 단계인 ROS 설치와 개발 환경 구축으로 넘어갈 수 있습니다.