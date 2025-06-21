#include "balance_robot_nodes/spi_hardware_node.h"
#include <ros/ros.h>
#include <std_msgs/Float32.h>

SPIHardwareNode::SPIHardwareNode(ros::NodeHandle& nh, ros::NodeHandle& pnh)
    : nh_(nh), pnh_(pnh),
      dac0_(nullptr), dac1_(nullptr), chip_(nullptr),
      reverse0_line_(nullptr), reverse1_line_(nullptr),
      inc_button_line_(nullptr), dec_button_line_(nullptr), latch_line_(nullptr),
      hall_state_0_(0), hall_state_1_(0),
      hall_count_0_(0), hall_count_1_(0),
      motor0_speed_(0.0), motor1_speed_(0.0),
      prev_inc_button_state_(false), prev_dec_button_state_(false),
      hardware_initialized_(false), last_error_(HardwareError::NONE),
      encoder_(HardwareConstants::WHEEL_RADIUS, HardwareConstants::WHEEL_DISTANCE, 999) {
    
    // 기본 모터 명령 초기화
    current_command_.motor0_output = 0.0;
    current_command_.motor1_output = 0.0;
    current_command_.motor0_direction = true;
    current_command_.motor1_direction = true;
    current_command_.control_mode = 0; // DIRECT_MODE
    current_command_.emergency_stop = false;
    
    // 상태 초기화
    current_status_.header.frame_id = "base_link";
    current_status_.motor0_speed = 0.0;
    current_status_.motor1_speed = 0.0;
    current_status_.spi_communication_ok = false;
    current_status_.dac0_ok = false;
    current_status_.dac1_ok = false;
    current_status_.gpio_ok = false;
    current_status_.error_code = static_cast<uint8_t>(HardwareError::NONE);
}

SPIHardwareNode::~SPIHardwareNode() {
    // 모터 정지
    if (dac0_) {
        dac0_->setOutputPercent(0.0);
    }
    if (dac1_) {
        dac1_->setOutputPercent(0.0);
    }
    
    // 하드웨어 정리
    cleanupGPIO();
    
    // DAC 객체 해제
    if (dac0_) {
        delete dac0_;
        dac0_ = nullptr;
    }
    if (dac1_) {
        delete dac1_;
        dac1_ = nullptr;
    }
    
    ROS_INFO("SPI Hardware Node shutdown complete");
}

bool SPIHardwareNode::initialize() {
    ROS_INFO("Initializing SPI Hardware Node...");
    
    // 매개변수 로드
    loadParameters();
    
    // 하드웨어 초기화
    if (!initializeGPIO()) {
        setError(HardwareError::GPIO_INIT_FAILED, "GPIO 초기화 실패");
        return false;
    }
    
    if (!initializeSPI()) {
        setError(HardwareError::SPI_INIT_FAILED, "SPI 초기화 실패");
        return false;
    }
    
    if (!initializeDAC()) {
        setError(HardwareError::DAC_INIT_FAILED, "DAC 초기화 실패");
        return false;
    }
    
    // 엔코더 초기화
    encoder_.initialize();
    
    // ROS 토픽 설정
    motor_command_sub_ = nh_.subscribe("motor_command", 10, 
                                      &SPIHardwareNode::motorCommandCallback, this);
    
    motor_status_pub_ = nh_.advertise<balance_robot_nodes::MotorStatus>("motor_status", 10);
    motor0_speed_pub_ = nh_.advertise<std_msgs::Float32>("actual_speed0", 10);  // 호환성
    motor1_speed_pub_ = nh_.advertise<std_msgs::Float32>("actual_speed1", 10);  // 호환성
    
    // 타이머 설정
    control_timer_ = nh_.createTimer(ros::Duration(HardwareConstants::CONTROL_LOOP_PERIOD), 
                                    &SPIHardwareNode::controlTimerCallback, this);
    sensor_timer_ = nh_.createTimer(ros::Duration(HardwareConstants::SENSOR_READ_PERIOD), 
                                   &SPIHardwareNode::sensorTimerCallback, this);
    button_timer_ = nh_.createTimer(ros::Duration(0.05), // 50ms 버튼 체크
                                   &SPIHardwareNode::buttonTimerCallback, this);
    
    // 초기 시간 설정
    last_button_time_ = ros::Time::now();
    
    hardware_initialized_ = true;
    current_status_.spi_communication_ok = true;
    current_status_.gpio_ok = true;
    
    ROS_INFO("SPI Hardware Node initialization complete");
    return true;
}

void SPIHardwareNode::loadParameters() {
    // GPIO 핀 설정
    pnh_.param<int>("mosi_pin", pin_config_.mosi_pin, 16);
    pnh_.param<int>("miso_pin", pin_config_.miso_pin, 17);
    pnh_.param<int>("clk_pin", pin_config_.clk_pin, 18);
    pnh_.param<int>("ss0_pin", pin_config_.ss0_pin, 19);
    pnh_.param<int>("ss1_pin", pin_config_.ss1_pin, 20);
    pnh_.param<int>("latch_pin", pin_config_.latch_pin, 38);
    pnh_.param<int>("reverse0_pin", pin_config_.reverse0_pin, 168);
    pnh_.param<int>("reverse1_pin", pin_config_.reverse1_pin, 13);
    pnh_.param<int>("inc_button_pin", pin_config_.inc_button_pin, 232);
    pnh_.param<int>("dec_button_pin", pin_config_.dec_button_pin, 15);
    pnh_.param<std::string>("gpio_chip_name", pin_config_.gpio_chip_name, "gpiochip0");
    pnh_.param<int>("spi_speed", pin_config_.spi_speed, HardwareConstants::DEFAULT_SPI_SPEED);
    
    ROS_INFO("Loaded GPIO pin configuration:");
    ROS_INFO("  SPI: MOSI=%d, MISO=%d, CLK=%d, SS0=%d, SS1=%d, LATCH=%d", 
             pin_config_.mosi_pin, pin_config_.miso_pin, pin_config_.clk_pin,
             pin_config_.ss0_pin, pin_config_.ss1_pin, pin_config_.latch_pin);
    ROS_INFO("  Motor: REV0=%d, REV1=%d", pin_config_.reverse0_pin, pin_config_.reverse1_pin);
    ROS_INFO("  Button: INC=%d, DEC=%d", pin_config_.inc_button_pin, pin_config_.dec_button_pin);
}

bool SPIHardwareNode::initializeGPIO() {
    ROS_INFO("Initializing GPIO...");
    
    // gpiod 칩 열기
    chip_ = gpiod_chip_open_by_name(pin_config_.gpio_chip_name.c_str());
    if (!chip_) {
        ROS_ERROR("Failed to open GPIO chip: %s", pin_config_.gpio_chip_name.c_str());
        return false;
    }
    
    // 출력 라인 설정
    if (!setOutputLine(pin_config_.reverse0_pin, &reverse0_line_)) return false;
    if (!setOutputLine(pin_config_.reverse1_pin, &reverse1_line_)) return false;
    if (!setOutputLine(pin_config_.latch_pin, &latch_line_)) return false;
    
    // 입력 라인 설정  
    if (!setInputLine(pin_config_.inc_button_pin, &inc_button_line_)) return false;
    if (!setInputLine(pin_config_.dec_button_pin, &dec_button_line_)) return false;
    
    // 초기 상태 설정
    gpiod_line_set_value(reverse0_line_, 1);  // 정방향
    gpiod_line_set_value(reverse1_line_, 1);  // 정방향
    gpiod_line_set_value(latch_line_, 1);     // 비활성 상태
    
    ROS_INFO("GPIO initialization complete");
    return true;
}

bool SPIHardwareNode::initializeSPI() {
    ROS_INFO("Initializing SPI...");
    
    // BitBangSPI 초기화
    spi_.init(pin_config_.mosi_pin, pin_config_.miso_pin, pin_config_.clk_pin, 
              BitBangSPI::MODE0, pin_config_.spi_speed, false);
    
    ROS_INFO("SPI initialization complete");
    return true;
}

bool SPIHardwareNode::initializeDAC() {
    ROS_INFO("Initializing DAC...");
    
    // DAC 객체 생성
    dac0_ = new MCP4921(&spi_, pin_config_.ss0_pin);
    if (!dac0_ || !dac0_->isInitialized()) {
        ROS_ERROR("Failed to initialize DAC0");
        current_status_.dac0_ok = false;
        return false;
    }
    current_status_.dac0_ok = true;
    
    dac1_ = new MCP4921(&spi_, pin_config_.ss1_pin);
    if (!dac1_ || !dac1_->isInitialized()) {
        ROS_ERROR("Failed to initialize DAC1");
        current_status_.dac1_ok = false;
        return false;
    }
    current_status_.dac1_ok = true;
    
    // 초기 출력값 설정 (정지)
    dac0_->setOutputPercent(0.0);
    dac1_->setOutputPercent(0.0);
    
    ROS_INFO("DAC initialization complete");
    return true;
}

bool SPIHardwareNode::setOutputLine(int pin, struct gpiod_line** pline) {
    *pline = gpiod_chip_get_line(chip_, pin);
    if (!*pline) {
        ROS_ERROR("Failed to get GPIO line: %d", pin);
        return false;
    }
    
    if (gpiod_line_request_output(*pline, "spi_hardware_node", 1) < 0) {
        ROS_ERROR("Failed to request GPIO line as output: pin %d", pin);
        return false;
    }
    return true;
}

bool SPIHardwareNode::setInputLine(int pin, struct gpiod_line** pline) {
    *pline = gpiod_chip_get_line(chip_, pin);
    if (!*pline) {
        ROS_ERROR("Failed to get GPIO line: %d", pin);
        return false;
    }
    
    if (gpiod_line_request_input(*pline, "spi_hardware_node") < 0) {
        ROS_ERROR("Failed to request GPIO line as input: pin %d", pin);
        return false;
    }
    return true;
}

void SPIHardwareNode::cleanupGPIO() {
    if (reverse0_line_) {
        gpiod_line_release(reverse0_line_);
        reverse0_line_ = nullptr;
    }
    if (reverse1_line_) {
        gpiod_line_release(reverse1_line_);
        reverse1_line_ = nullptr;
    }
    if (inc_button_line_) {
        gpiod_line_release(inc_button_line_);
        inc_button_line_ = nullptr;
    }
    if (dec_button_line_) {
        gpiod_line_release(dec_button_line_);
        dec_button_line_ = nullptr;
    }
    if (latch_line_) {
        gpiod_line_release(latch_line_);
        latch_line_ = nullptr;
    }
    if (chip_) {
        gpiod_chip_close(chip_);
        chip_ = nullptr;
    }
}

void SPIHardwareNode::motorCommandCallback(const balance_robot_nodes::MotorCommand::ConstPtr& msg) {
    current_command_ = *msg;
    
    // 비상 정지 체크
    if (current_command_.emergency_stop) {
        current_command_.motor0_output = 0.0;
        current_command_.motor1_output = 0.0;
        ROS_WARN("Emergency stop activated!");
    }
    
    // 모터 명령 적용
    applyMotorCommand();
}

void SPIHardwareNode::controlTimerCallback(const ros::TimerEvent& event) {
    if (!hardware_initialized_) return;
    
    // 모터 속도 계산
    calculateMotorSpeeds();
    
    // 모터 상태 발행
    publishMotorStatus();
}

void SPIHardwareNode::sensorTimerCallback(const ros::TimerEvent& event) {
    if (!hardware_initialized_) return;
    
    // 홀센서 상태 읽기
    readHallSensors();
}

void SPIHardwareNode::buttonTimerCallback(const ros::TimerEvent& event) {
    if (!hardware_initialized_) return;
    
    // 버튼 상태 확인
    checkButtons();
}

void SPIHardwareNode::applyMotorCommand() {
    if (!hardware_initialized_) return;
    
    // 모터 방향 설정
    setMotorDirection(0, current_command_.motor0_direction);
    setMotorDirection(1, current_command_.motor1_direction);
    
    // 모터 속도 설정
    setMotorSpeed(0, std::abs(current_command_.motor0_output));
    setMotorSpeed(1, std::abs(current_command_.motor1_output));
    
    // 실제 출력값 저장
    current_status_.motor0_actual_output = current_command_.motor0_output;
    current_status_.motor1_actual_output = current_command_.motor1_output;
}

void SPIHardwareNode::setMotorDirection(int motor_id, bool forward) {
    if (motor_id == 0 && reverse0_line_) {
        gpiod_line_set_value(reverse0_line_, forward ? 1 : 0);
    } else if (motor_id == 1 && reverse1_line_) {
        gpiod_line_set_value(reverse1_line_, forward ? 1 : 0);
    }
}

void SPIHardwareNode::setMotorSpeed(int motor_id, float speed_percent) {
    if (speed_percent < 0.0f) speed_percent = 0.0f;
    if (speed_percent > 100.0f) speed_percent = 100.0f;
    
    if (motor_id == 0 && dac0_) {
        dac0_->setOutputPercent(speed_percent);
    } else if (motor_id == 1 && dac1_) {
        dac1_->setOutputPercent(speed_percent);
    }
}

void SPIHardwareNode::readHallSensors() {
    if (!latch_line_) return;
    
    // SPI를 통해 홀센서 데이터 읽기
    gpiod_line_set_value(latch_line_, 1);
    uint8_t data = spi_.transfer(0x00);
    gpiod_line_set_value(latch_line_, 0);
    
    // 홀센서 상태 추출
    uint8_t new_hall_state_0 = (data >> 1) & 0x7;
    uint8_t new_hall_state_1 = (data >> 5) & 0x7;
    
    // 상태 변화 감지 및 카운트 업데이트
    if (new_hall_state_0 != hall_state_0_) {
        hall_count_0_++;
        hall_state_0_ = new_hall_state_0;
    }
    
    if (new_hall_state_1 != hall_state_1_) {
        hall_count_1_++;
        hall_state_1_ = new_hall_state_1;
    }
    
    // 엔코더 업데이트
    encoder_.updateHallSensors(hall_state_0_, hall_state_1_);
    
    // 상태 업데이트
    current_status_.motor0_hall_state = hall_state_0_;
    current_status_.motor1_hall_state = hall_state_1_;
    current_status_.motor0_hall_count = hall_count_0_;
    current_status_.motor1_hall_count = hall_count_1_;
}

void SPIHardwareNode::calculateMotorSpeeds() {
    // 엔코더에서 속도 계산 (rad/s에서 RPM으로 변환)
    motor0_speed_ = encoder_.getWheel0Velocity() * 60.0 / (2.0 * M_PI);
    motor1_speed_ = encoder_.getWheel1Velocity() * 60.0 / (2.0 * M_PI);
    
    // 상태 업데이트
    current_status_.motor0_speed = motor0_speed_;
    current_status_.motor1_speed = motor1_speed_;
}

void SPIHardwareNode::checkButtons() {
    if (!inc_button_line_ || !dec_button_line_) return;
    
    ros::Time now = ros::Time::now();
    double elapsed = (now - last_button_time_).toSec();
    
    // 디바운싱 (50ms)
    if (elapsed < 0.05) return;
    
    // 버튼 상태 읽기 (풀업 저항 사용)
    int inc_value = gpiod_line_get_value(inc_button_line_);
    int dec_value = gpiod_line_get_value(dec_button_line_);
    
    bool inc_button_state = (inc_value == 0);
    bool dec_button_state = (dec_value == 0);
    
    // 상승 엣지 감지 및 처리
    if (inc_button_state && !prev_inc_button_state_) {
        ROS_INFO("Increase button pressed");
        // 버튼 이벤트는 로그만 출력 (실제 속도 제어는 Controller Node에서)
        last_button_time_ = now;
    }
    
    if (dec_button_state && !prev_dec_button_state_) {
        ROS_INFO("Decrease button pressed");
        last_button_time_ = now;
    }
    
    prev_inc_button_state_ = inc_button_state;
    prev_dec_button_state_ = dec_button_state;
}

void SPIHardwareNode::publishMotorStatus() {
    current_status_.header.stamp = ros::Time::now();
    current_status_.error_code = static_cast<uint8_t>(last_error_);
    current_status_.error_message = hardwareErrorToString(last_error_);
    
    motor_status_pub_.publish(current_status_);
    
    // 호환성을 위한 개별 토픽 발행
    std_msgs::Float32 speed0_msg, speed1_msg;
    speed0_msg.data = motor0_speed_;
    speed1_msg.data = motor1_speed_;
    motor0_speed_pub_.publish(speed0_msg);
    motor1_speed_pub_.publish(speed1_msg);
}

void SPIHardwareNode::setError(HardwareError error, const std::string& message) {
    last_error_ = error;
    if (error != HardwareError::NONE) {
        ROS_ERROR("SPI Hardware Error: %s - %s", hardwareErrorToString(error).c_str(), message.c_str());
    }
}

// 메인 함수
int main(int argc, char** argv) {
    ros::init(argc, argv, "spi_hardware_node");
    ros::NodeHandle nh;
    ros::NodeHandle pnh("~");
    
    SPIHardwareNode node(nh, pnh);
    
    if (!node.initialize()) {
        ROS_ERROR("Failed to initialize SPI Hardware Node");
        return -1;
    }
    
    ROS_INFO("SPI Hardware Node started successfully");
    ros::spin();
    
    return 0;
} 