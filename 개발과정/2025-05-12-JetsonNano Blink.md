# ROS에서 Catkin을 사용한 LED Blink 프로젝트 튜토리얼

ROS 환경에서 catkin_make를 사용하여 LED Blink 프로젝트를 만드는 방법을 단계별로 알려드리겠습니다. 이 튜토리얼은 Jetson Nano에서 GPIO를 활용하여 LED를 깜빡이는 노드를 만드는 과정을 다룹니다.

## 1. 작업 환경 설정

먼저 catkin 작업 공간이 있는지 확인하고, 없다면 생성합니다:

```bash
# 홈 디렉토리로 이동
cd ~

# 만약 catkin_ws가 없다면 생성
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws/
catkin_make

# 작업 공간 설정 (bashrc에 추가하는 것이 좋습니다)
source devel/setup.bash
```

## 2. Blink 패키지 생성

새로운 ROS 패키지를 생성합니다:

```bash
cd ~/catkin_ws/src
catkin_create_pkg led_blink roscpp rospy std_msgs
cd led_blink
```

## 3. GPIO 라이브러리 설치 (Jetson Nano용)

Jetson Nano의 GPIO를 사용하기 위해 필요한 라이브러리를 설치합니다:

```bash
# Jetson.GPIO 라이브러리 설치
sudo apt-get update
sudo apt-get install python-dev python3-dev
sudo pip install Jetson.GPIO
sudo groupadd -f -r gpio
sudo usermod -a -G gpio $USER
```

## 4. Blink 노드 생성

`src` 디렉토리에 LED 제어용 노드를 생성합니다:

```bash
cd ~/catkin_ws/src/led_blink
mkdir -p src
```

### Python 노드 생성 (led_blink_node.py)

다음 코드를 `src` 디렉토리에 `led_blink_node.py` 파일로 작성합니다:

```python
#!/usr/bin/env python3

import rospy
import Jetson.GPIO as GPIO
import time

class LedBlinker:
    def __init__(self):
        # 초기화
        rospy.init_node('led_blink_node', anonymous=True)
        
        # GPIO 설정
        self.led_pin = rospy.get_param('~led_pin', 18)  # 기본값 18번 핀
        self.blink_rate = rospy.get_param('~blink_rate', 1.0)  # 기본값 1Hz
        
        # GPIO 모드 설정
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(self.led_pin, GPIO.OUT)
        
        rospy.loginfo(f"LED Blink 노드 시작: 핀={self.led_pin}, 주기={self.blink_rate}Hz")
        
        # 종료 시 GPIO 정리
        rospy.on_shutdown(self.cleanup)
        
    def cleanup(self):
        GPIO.cleanup()
        rospy.loginfo("GPIO 리소스 정리 완료")
        
    def run(self):
        rate = rospy.Rate(self.blink_rate * 2)  # ON/OFF 상태를 위해 2배 속도
        led_state = False
        
        while not rospy.is_shutdown():
            led_state = not led_state
            GPIO.output(self.led_pin, led_state)
            rate.sleep()

if __name__ == '__main__':
    try:
        blinker = LedBlinker()
        blinker.run()
    except rospy.ROSInterruptException:
        pass
```

### C++ 노드 생성 (led_blink_node.cpp)

C++로 구현하려면, 먼저 필요한 라이브러리를 설치합니다:

```bash
sudo apt-get install libgpiod-dev
```

다음 코드를 `src` 디렉토리에 `led_blink_node.cpp` 파일로 작성합니다:

```cpp
#include <ros/ros.h>
#include <gpiod.h>
#include <unistd.h>
#include <string>

class LedBlinker {
private:
    ros::NodeHandle nh_;
    ros::NodeHandle private_nh_;
    
    struct gpiod_chip *chip_;
    struct gpiod_line *line_;
    
    int led_pin_;
    double blink_rate_;
    
public:
    LedBlinker() : private_nh_("~") {
        // 파라미터 로드
        private_nh_.param("led_pin", led_pin_, 18);
        private_nh_.param("blink_rate", blink_rate_, 1.0);
        
        // GPIO 초기화
        chip_ = gpiod_chip_open_by_name("gpiochip0");
        if (!chip_) {
            ROS_ERROR("GPIO 칩을 열 수 없습니다");
            ros::shutdown();
            return;
        }
        
        line_ = gpiod_chip_get_line(chip_, led_pin_);
        if (!line_) {
            ROS_ERROR("GPIO 라인을 가져올 수 없습니다");
            gpiod_chip_close(chip_);
            ros::shutdown();
            return;
        }
        
        if (gpiod_line_request_output(line_, "led_blink", 0) < 0) {
            ROS_ERROR("GPIO 라인을 출력으로 설정할 수 없습니다");
            gpiod_chip_close(chip_);
            ros::shutdown();
            return;
        }
        
        ROS_INFO("LED Blink 노드 시작: 핀=%d, 주기=%.1fHz", led_pin_, blink_rate_);
    }
    
    ~LedBlinker() {
        if (line_) {
            gpiod_line_release(line_);
        }
        if (chip_) {
            gpiod_chip_close(chip_);
        }
        ROS_INFO("GPIO 리소스 정리 완료");
    }
    
    void run() {
        ros::Rate rate(blink_rate_ * 2); // ON/OFF 상태를 위해 2배 속도
        int value = 0;
        
        while (ros::ok()) {
            value = !value;
            gpiod_line_set_value(line_, value);
            rate.sleep();
        }
    }
};

int main(int argc, char **argv) {
    ros::init(argc, argv, "led_blink_node");
    
    LedBlinker blinker;
    blinker.run();
    
    return 0;
}
```

## 5. 실행 권한 부여 (Python 사용 시)

Python 스크립트를 실행 가능하게 만듭니다:

```bash
chmod +x ~/catkin_ws/src/led_blink/src/led_blink_node.py
```

## 6. CMakeLists.txt 및 package.xml 수정

### CMakeLists.txt 수정

Python을 사용하는 경우:

```cmake
cmake_minimum_required(VERSION 3.0.2)
project(led_blink)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
)

catkin_package()

include_directories(
  ${catkin_INCLUDE_DIRS}
)

# Python 노드 설치
catkin_install_python(PROGRAMS
  src/led_blink_node.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
```

C++를 사용하는 경우:

```cmake
cmake_minimum_required(VERSION 3.0.2)
project(led_blink)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
)

catkin_package()

include_directories(
  ${catkin_INCLUDE_DIRS}
)

# C++ 노드 빌드
add_executable(led_blink_node src/led_blink_node.cpp)
target_link_libraries(led_blink_node
  ${catkin_LIBRARIES}
  gpiod
)
```

### package.xml 수정

`package.xml` 파일에 필요한 의존성을 추가합니다:

```xml
<?xml version="1.0"?>
<package format="2">
  <name>led_blink</name>
  <version>0.0.1</version>
  <description>LED Blink package for Jetson Nano</description>
  <maintainer email="your_email@example.com">Your Name</maintainer>
  <license>BSD</license>
  
  <buildtool_depend>catkin</buildtool_depend>
  <build_depend>roscpp</build_depend>
  <build_depend>rospy</build_depend>
  <build_depend>std_msgs</build_depend>
  
  <exec_depend>roscpp</exec_depend>
  <exec_depend>rospy</exec_depend>
  <exec_depend>std_msgs</exec_depend>
</package>
```

## 7. 런치 파일 생성

`launch` 디렉토리를 만들고 런치 파일을 작성합니다:

```bash
mkdir -p ~/catkin_ws/src/led_blink/launch
```

`led_blink.launch` 파일을 다음과 같이 작성합니다:

```xml
<launch>
  <!-- LED 핀 번호 및 깜박임 주기 -->
  <arg name="led_pin" default="18"/>
  <arg name="blink_rate" default="1.0"/>
  
  <!-- Python 노드 실행 -->
  <node name="led_blink" pkg="led_blink" type="led_blink_node.py" output="screen">
    <param name="led_pin" value="$(arg led_pin)"/>
    <param name="blink_rate" value="$(arg blink_rate)"/>
  </node>
  
  <!-- C++ 노드 사용 시 아래 코드 대신 사용 -->
  <!--
  <node name="led_blink" pkg="led_blink" type="led_blink_node" output="screen">
    <param name="led_pin" value="$(arg led_pin)"/>
    <param name="blink_rate" value="$(arg blink_rate)"/>
  </node>
  -->
</launch>
```

## 8. 빌드 및 실행

작업 공간으로 돌아가서 패키지를 빌드합니다:

```bash
cd ~/catkin_ws
catkin_make
source devel/setup.bash
```

런치 파일로 노드를 실행합니다:

```bash
# 기본 설정으로 실행
roslaunch led_blink led_blink.launch

# 다른 핀과 주기 설정으로 실행
roslaunch led_blink led_blink.launch led_pin:=12 blink_rate:=2.0
```

## 9. 회로 연결

Jetson Nano에 LED를 연결하는 방법:

1. LED의 양극(긴 다리)을 저항(약 220-330Ω)을 통해 지정된 GPIO 핀(예: 18번)에 연결
2. LED의 음극(짧은 다리)을 GND에 연결

![LED 연결 다이어그램]

## 10. 문제 해결

- **권한 오류**: `sudo usermod -a -G gpio $USER` 이후에 시스템 재시작 필요
- **핀 번호 오류**: GPIO 핀 번호가 올바른지 확인. BCM 모드와 BOARD 모드의 차이 확인
- **LED 작동 없음**: 회로 연결 상태 및 저항값 확인

## 11. 응용 확장

- 서비스를 추가하여 LED 동작 제어하기
- 토픽으로 깜빡임 패턴 변경하기
- 여러 LED를 제어하는 기능 추가하기

이 튜토리얼로 ROS와 Jetson Nano를 이용한 기본적인 하드웨어 제어 방법을 배울 수 있습니다. 이를 기반으로 더 복잡한 로봇 프로젝트에 적용해보세요!


### 결국 gpiochip0으로 했다.
- 근데 좀 먼 길 돌아왔다.
- 18pin != GPIO18 != line num 18

## gpiochip0의 주요 GPIO 라인

| GPIO 라인 번호 | 이름 | 현재 기능 | 상태 | 모드 |
|--------------|------|----------|------|------|
| 12 | SPI1_MOSI | unused | input | active-high |
| 13 | SPI1_MISO | unused | input | active-high |
| 14 | SPI1_SCK | unused | input | active-high |
| 15 | SPI1_CS0 | unused | input | active-high |
| 16 | SPI0_MOSI | unused | input | active-high |
| 17 | SPI0_MISO | unused | input | active-high |
| 18 | SPI0_SCK | led_blink | output | active-high [used] |
| 19 | SPI0_CS0 | unused | input | active-high |
| 20 | SPI0_CS1 | unused | input | active-high |
| 38 | GPIO13 | unused | input | active-high |
| 50 | UART1_RTS | unused | input | active-high |
| 51 | UART1_CTS | unused | input | active-high |
| 76 | I2S0_FS | unused | input | active-high |
| 77 | I2S0_DIN | unused | input | active-high |
| 78 | I2S0_DOUT | unused | input | active-high |
| 79 | I2S0_SCLK | unused | input | active-high |
| 149 | GPIO01 | unused | input | active-high |
| 168 | GPIO07 | unused | input | active-high |
| 194 | GPIO12 | unused | input | active-high |
| 200 | GPIO11 | unused | input | active-high |
| 216 | GPIO09 | unused | input | active-high |
| 232 | SPI1_CS1 | unused | input | active-high |
|   |   |   |   |   |
| 187 | unnamed | ? | output | active-high [used] |
| 189 | unnamed | Power | input | active-low [used] |
| 190 | unnamed | Forcerecovery | input | active-low [used] |
| 201 | unnamed | cd | input | active-high [used] |
| 202 | unnamed | pwm-fan-tach | input | active-high [used] |
| 203 | unnamed | vdd-3v3-sd | output | active-high [used] |
| 225 | unnamed | hdmi2.0_hpd | input | active-high [used] |
| 228 | unnamed | extcon:extcon@1 | input | active-high [used] |

## gpiochip1의 모든 GPIO 라인

| GPIO 라인 번호 | 이름 | 현재 기능 | 상태 | 모드 |
|--------------|------|----------|------|------|
| 0 | unnamed | unused | input | active-high |
| 1 | unnamed | spmic-default-output-high | output | active-high [used] |
| 2 | unnamed | unused | input | active-high |
| 3 | unnamed | vdd-3v3-sys | output | active-high [used] |
| 4 | unnamed | unused | input | active-high |
| 5 | unnamed | unused | input | active-high |
| 6 | unnamed | enable | output | active-high [used] |
| 7 | unnamed | avdd-io-edp-1v05 | output | active-high [used] |

이 표는 각 GPIO 라인의 현재 설정과 사용 상태를 보여줍니다. 특히 주목할 점은 gpiochip0의 라인 18(SPI0_SCK)이 현재 "led_blink"로 설정되어 출력 모드로 사용 중인 것을 확인할 수 있습니다.

gpiochip1은 시스템 전원 관리에 관련된 GPIO 라인을 포함하고 있으며 대부분 이미 사용 중입니다. 외부 핀 제어에는 주로 gpiochip0을 사용하는 것이 좋습니다.



// BitBangSPI.h에 포함
#include <gpiod.h>

class BitBangSPI {
private:
    // 기존 멤버 변수에 추가
    struct gpiod_chip* chip;
    struct gpiod_line* mosi_line;
    struct gpiod_line* miso_line;
    struct gpiod_line* sclk_line;
    struct gpiod_line* cs_line;
    // ...
};

// BitBangSPI.cpp 수정
void BitBangSPI::initGPIO() {
    // gpiochip0 열기
    chip = gpiod_chip_open("/dev/gpiochip0");
    if (!chip) {
        throw std::runtime_error("Failed to open GPIO chip");
    }
    
    // GPIO 라인 얻기
    mosi_line = gpiod_chip_get_line(chip, mosi_pin_);
    miso_line = gpiod_chip_get_line(chip, miso_pin_);
    sclk_line = gpiod_chip_get_line(chip, sclk_pin_);
    cs_line = gpiod_chip_get_line(chip, cs_pin_);
    
    if (!mosi_line || !miso_line || !sclk_line || !cs_line) {
        gpiod_chip_close(chip);
        throw std::runtime_error("Failed to get GPIO lines");
    }
    
    // 라인 설정
    int ret = gpiod_line_request_output(mosi_line, "spi_mosi", 0);
    if (ret < 0) {
        gpiod_chip_close(chip);
        throw std::runtime_error("Failed to request MOSI GPIO line as output");
    }
    
    ret = gpiod_line_request_input(miso_line, "spi_miso");
    if (ret < 0) {
        gpiod_line_release(mosi_line);
        gpiod_chip_close(chip);
        throw std::runtime_error("Failed to request MISO GPIO line as input");
    }
    
    ret = gpiod_line_request_output(sclk_line, "spi_sclk", cpol_);
    if (ret < 0) {
        gpiod_line_release(mosi_line);
        gpiod_line_release(miso_line);
        gpiod_chip_close(chip);
        throw std::runtime_error("Failed to request SCLK GPIO line as output");
    }
    
    ret = gpiod_line_request_output(cs_line, "spi_cs", 1);  // CS는 HIGH로 시작
    if (ret < 0) {
        gpiod_line_release(mosi_line);
        gpiod_line_release(miso_line);
        gpiod_line_release(sclk_line);
        gpiod_chip_close(chip);
        throw std::runtime_error("Failed to request CS GPIO line as output");
    }
    
    gpio_initialized_ = true;
}

// GPIO 상태 설정/읽기 함수도 수정
void BitBangSPI::writePin(int line, bool value) {
    if (line == mosi_pin_) {
        gpiod_line_set_value(mosi_line, value ? 1 : 0);
    } else if (line == sclk_pin_) {
        gpiod_line_set_value(sclk_line, value ? 1 : 0);
    } else if (line == cs_pin_) {
        gpiod_line_set_value(cs_line, value ? 1 : 0);
    }
}

bool BitBangSPI::readPin(int line) {
    if (line == miso_pin_) {
        return gpiod_line_get_value(miso_line) == 1;
    }
    return false;
}

// 소멸자도 수정
BitBangSPI::~BitBangSPI() {
    if (gpio_initialized_) {
        gpiod_line_release(mosi_line);
        gpiod_line_release(miso_line);
        gpiod_line_release(sclk_line);
        gpiod_line_release(cs_line);
        gpiod_chip_close(chip);
        gpio_initialized_ = false;
    }
}


#include <iostream>
#include <fstream>
#include <sstream>
#include <string>
#include <thread>
#include <chrono>
#include <vector>
#include <unistd.h>

class JetsonGPIO {
public:
    // 핀 모드 설정
    static bool setup(int pin, bool isOutput) {
        // GPIO 내보내기
        std::ofstream exportFile("/sys/class/gpio/export");
        if (!exportFile.is_open()) {
            std::cerr << "Cannot open GPIO export file" << std::endl;
            return false;
        }
        exportFile << pin;
        exportFile.close();
        
        // 방향 설정
        std::stringstream directionPath;
        directionPath << "/sys/class/gpio/gpio" << pin << "/direction";
        
        // 약간의 지연을 통해 파일 생성 기다리기
        usleep(100000);  // 100ms
        
        std::ofstream directionFile(directionPath.str());
        if (!directionFile.is_open()) {
            std::cerr << "Cannot open GPIO direction file for pin " << pin << std::endl;
            return false;
        }
        directionFile << (isOutput ? "out" : "in");
        directionFile.close();
        
        return true;
    }
    
    // GPIO 값 설정
    static bool output(int pin, bool value) {
        std::stringstream valuePath;
        valuePath << "/sys/class/gpio/gpio" << pin << "/value";
        
        std::ofstream valueFile(valuePath.str());
        if (!valueFile.is_open()) {
            std::cerr << "Cannot open GPIO value file for pin " << pin << std::endl;
            return false;
        }
        valueFile << (value ? "1" : "0");
        valueFile.close();
        
        return true;
    }
    
    // GPIO 값 읽기
    static bool input(int pin, bool& value) {
        std::stringstream valuePath;
        valuePath << "/sys/class/gpio/gpio" << pin << "/value";
        
        std::ifstream valueFile(valuePath.str());
        if (!valueFile.is_open()) {
            std::cerr << "Cannot open GPIO value file for pin " << pin << std::endl;
            return false;
        }
        
        char val;
        valueFile >> val;
        valueFile.close();
        
        value = (val == '1');
        return true;
    }
    
    // GPIO 정리
    static bool cleanup(int pin) {
        std::ofstream unexportFile("/sys/class/gpio/unexport");
        if (!unexportFile.is_open()) {
            std::cerr << "Cannot open GPIO unexport file" << std::endl;
            return false;
        }
        unexportFile << pin;
        unexportFile.close();
        
        return true;
    }
};

class JetsonBitBangSPI {
private:
    int mosi_pin_;
    int miso_pin_;
    int sclk_pin_;
    int cs_pin_;
    
    bool cpol_;
    bool cpha_;
    unsigned int delay_us_;
    bool bit_order_lsb_;
    
    void delayMicroseconds(unsigned int us) {
        std::this_thread::sleep_for(std::chrono::microseconds(us));
    }
    
public:
    JetsonBitBangSPI(int mosi_pin, int miso_pin, int sclk_pin, int cs_pin, 
                   int mode = 0, unsigned int speed_hz = 1000000, bool bit_order_lsb = false)
        : mosi_pin_(mosi_pin), miso_pin_(miso_pin), sclk_pin_(sclk_pin), cs_pin_(cs_pin),
          bit_order_lsb_(bit_order_lsb) {
        
        // 모드에 따른 CPOL, CPHA 설정
        cpol_ = (mode & 2) != 0;
        cpha_ = (mode & 1) != 0;
        
        // 지연 시간 계산 (클럭 반주기)
        delay_us_ = 500000 / speed_hz;
        if (delay_us_ < 1) delay_us_ = 1;
        
        // GPIO 초기화
        JetsonGPIO::setup(mosi_pin_, true);  // 출력
        JetsonGPIO::setup(miso_pin_, false); // 입력
        JetsonGPIO::setup(sclk_pin_, true);  // 출력
        JetsonGPIO::setup(cs_pin_, true);    // 출력
        
        // 초기 상태 설정
        JetsonGPIO::output(sclk_pin_, cpol_);
        JetsonGPIO::output(cs_pin_, true);   // CS 비활성화
    }
    
    ~JetsonBitBangSPI() {
        // GPIO 정리
        JetsonGPIO::cleanup(mosi_pin_);
        JetsonGPIO::cleanup(miso_pin_);
        JetsonGPIO::cleanup(sclk_pin_);
        JetsonGPIO::cleanup(cs_pin_);
    }
    
    // 단일 바이트 전송
    uint8_t transfer(uint8_t data) {
        uint8_t received = 0;
        
        for (int i = 0; i < 8; i++) {
            int bit_pos = bit_order_lsb_ ? i : (7 - i);
            bool bit_val = (data & (1 << bit_pos)) != 0;
            
            // MOSI에 비트 설정
            JetsonGPIO::output(mosi_pin_, bit_val);
            
            // CPHA에 따른 클럭 에지 처리
            if (cpha_) {
                delayMicroseconds(delay_us_);
                JetsonGPIO::output(sclk_pin_, !cpol_);
                delayMicroseconds(delay_us_);
                
                // MISO에서 비트 읽기
                bool bit_read = false;
                JetsonGPIO::input(miso_pin_, bit_read);
                if (bit_read) {
                    received |= (1 << bit_pos);
                }
                
                JetsonGPIO::output(sclk_pin_, cpol_);
            } else {
                delayMicroseconds(delay_us_);
                JetsonGPIO::output(sclk_pin_, !cpol_);
                
                // MISO에서 비트 읽기
                bool bit_read = false;
                JetsonGPIO::input(miso_pin_, bit_read);
                if (bit_read) {
                    received |= (1 << bit_pos);
                }
                
                delayMicroseconds(delay_us_);
                JetsonGPIO::output(sclk_pin_, cpol_);
            }
        }
        
        return received;
    }
    
    // 16비트 데이터 전송
    uint16_t transfer16(uint16_t data) {
        JetsonGPIO::output(cs_pin_, false);  // CS 활성화
        
        uint8_t msb = (data >> 8) & 0xFF;
        uint8_t lsb = data & 0xFF;
        
        uint8_t msb_in = transfer(msb);
        uint8_t lsb_in = transfer(lsb);
        
        JetsonGPIO::output(cs_pin_, true);   // CS 비활성화
        
        return ((uint16_t)msb_in << 8) | lsb_in;
    }
    
    // 버퍼 전송
    std::vector<uint8_t> transferBuffer(const std::vector<uint8_t>& data) {
        std::vector<uint8_t> received(data.size());
        
        JetsonGPIO::output(cs_pin_, false);  // CS 활성화
        
        for (size_t i = 0; i < data.size(); i++) {
            received[i] = transfer(data[i]);
        }
        
        JetsonGPIO::output(cs_pin_, true);   // CS 비활성화
        
        return received;
    }
};



 pnh_.param<int>("mosi_pin", mosi_pin_, 10); // 물리적 핀 7에 해당하는 GPIO 번호
    pnh_.param<int>("miso_pin", miso_pin_, 9); // 물리적 핀 7에 해당하는 GPIO 번호
    pnh_.param<int>("clk_pin", clk_pin_, 11); // 물리적 핀 7에 해당하는 GPIO 번호
    pnh_.param<int>("ss0_pin", ss0_pin_, 8); // 물리적 핀 7에 해당하는 GPIO 번호
    pnh_.param<int>("ss1_pin", ss1_pin_, 7); // 물리적 핀 7에 해당하는 GPIO 번호
    pnh_.param<int>("reverse_pin", reverse_pin_, 194); // 물리적 핀 7에 해당하는 GPIO 번호
    pnh_.param<int>("hall_sensor_pin", hall_sensor_pin_, 38); // 물리적 핀 13에 해당하는 GPIO 번호
    pnh_.param<int>("inc_button_pin", inc_button_pin_, 23); // 물리적 핀 16에 해당하는 GPIO 번호
    pnh_.param<int>("dec_button_pin", dec_button_pin_, 24); // 물리적 핀 18에 해당하는 GPIO 번호
    pnh_.param<float>("max_speed", max_speed_, 100.0);
    pnh_.param<std::string>("gpio_chip_name", gpio_chip_name_, "gpiochip0");


안됨..

