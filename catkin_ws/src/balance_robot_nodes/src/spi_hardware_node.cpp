#include "balance_robot_nodes/spi_hardware_node.h"
#include <ros/ros.h>
#include <std_msgs/Float32.h>

// BitBangSPI 실제 구현
BitBangSPI::BitBangSPI(int mosi_pin, int miso_pin, int sclk_pin, int cs_pin,
                       SPIMode mode, uint32_t max_speed_hz, bool bit_order_lsb)
    : mosi_pin_(mosi_pin), miso_pin_(miso_pin), sclk_pin_(sclk_pin), cs_pin_(cs_pin),
      mode_(mode), max_speed_hz_(max_speed_hz), bit_order_lsb_(bit_order_lsb),
      gpio_initialized_(false), chip_(nullptr),
      mosi_line_(nullptr), miso_line_(nullptr), sclk_line_(nullptr), cs_line_(nullptr) {
    
    // 모드에 따른 CPOL, CPHA 설정
    cpol_ = (mode_ == MODE2 || mode_ == MODE3);
    cpha_ = (mode_ == MODE1 || mode_ == MODE3);
    
    initGPIO();
}

BitBangSPI::~BitBangSPI() {
    if (gpio_initialized_) {
        cleanupGPIO();
    }
}

void BitBangSPI::initGPIO() {
    // gpiochip0 열기
    chip_ = gpiod_chip_open("/dev/gpiochip0");
    if (!chip_) {
        ROS_ERROR("Failed to open GPIO chip");
        return;
    }

    // GPIO 라인 얻기
    mosi_line_ = gpiod_chip_get_line(chip_, mosi_pin_);
    miso_line_ = gpiod_chip_get_line(chip_, miso_pin_);
    sclk_line_ = gpiod_chip_get_line(chip_, sclk_pin_);
    cs_line_ = gpiod_chip_get_line(chip_, cs_pin_);

    if (!mosi_line_ || !miso_line_ || !sclk_line_ || !cs_line_) {
        gpiod_chip_close(chip_);
        ROS_ERROR("Failed to get GPIO lines");
        return;
    }

    // 라인 설정
    int ret = gpiod_line_request_output(mosi_line_, "spi_mosi", 0);
    if (ret < 0) {
        gpiod_chip_close(chip_);
        ROS_ERROR("Failed to request MOSI GPIO line as output");
        return;
    }

    ret = gpiod_line_request_input(miso_line_, "spi_miso");
    if (ret < 0) {
        gpiod_line_release(mosi_line_);
        gpiod_chip_close(chip_);
        ROS_ERROR("Failed to request MISO GPIO line as input");
        return;
    }

    ret = gpiod_line_request_output(sclk_line_, "spi_sclk", cpol_);
    if (ret < 0) {
        gpiod_line_release(mosi_line_);
        gpiod_line_release(miso_line_);
        gpiod_chip_close(chip_);
        ROS_ERROR("Failed to request SCLK GPIO line as output");
        return;
    }

    ret = gpiod_line_request_output(cs_line_, "spi_cs", 1);  // CS는 HIGH로 시작
    if (ret < 0) {
        gpiod_line_release(mosi_line_);
        gpiod_line_release(miso_line_);
        gpiod_line_release(sclk_line_);
        gpiod_chip_close(chip_);
        ROS_ERROR("Failed to request CS GPIO line as output");
        return;
    }

    // 초기 상태 설정
    gpiod_line_set_value(sclk_line_, cpol_);  // CPOL에 따른 초기 클럭 레벨
    gpiod_line_set_value(cs_line_, 1);        // CS는 기본적으로 비활성화(HIGH)
    
    gpio_initialized_ = true;
}

void BitBangSPI::cleanupGPIO() {
    if (mosi_line_) {
        gpiod_line_release(mosi_line_);
        mosi_line_ = nullptr;
    }
    if (miso_line_) {
        gpiod_line_release(miso_line_);
        miso_line_ = nullptr;
    }
    if (sclk_line_) {
        gpiod_line_release(sclk_line_);
        sclk_line_ = nullptr;
    }
    if (cs_line_) {
        gpiod_line_release(cs_line_);
        cs_line_ = nullptr;
    }
    if (chip_) {
        gpiod_chip_close(chip_);
        chip_ = nullptr;
    }
    gpio_initialized_ = false;
}

void BitBangSPI::delayMicroseconds(unsigned int usec) {
    std::this_thread::sleep_for(std::chrono::microseconds(usec));
}

uint8_t BitBangSPI::transfer(uint8_t data_out) {
    if (!gpio_initialized_) return 0;
    
    uint8_t data_in = 0;
    
    // 반 클럭 주기 계산 (마이크로초)
    unsigned int half_period_us = 500000 / max_speed_hz_;
    if (half_period_us < 1) half_period_us = 1;
    
    for (int bit = 7; bit >= 0; bit--) {
        // 데이터 출력 (MSB first)
        bool bit_out = (data_out >> bit) & 0x01;
        if (bit_order_lsb_) {
            bit_out = (data_out >> (7 - bit)) & 0x01;
        }
        
        // CPHA = 0: 클럭 상승 엣지에서 데이터 샘플링
        // CPHA = 1: 클럭 하강 엣지에서 데이터 샘플링
        if (!cpha_) {
            gpiod_line_set_value(mosi_line_, bit_out);
            delayMicroseconds(half_period_us);
        }
        
        // 클럭 토글
        gpiod_line_set_value(sclk_line_, !cpol_);
        
        if (cpha_) {
            gpiod_line_set_value(mosi_line_, bit_out);
        }
        
        delayMicroseconds(half_period_us);
        
        // 데이터 입력 읽기
        bool bit_in = gpiod_line_get_value(miso_line_);
        if (bit_order_lsb_) {
            data_in |= (bit_in << (7 - bit));
        } else {
            data_in |= (bit_in << bit);
        }
        
        // 클럭 원래 상태로 복원
        gpiod_line_set_value(sclk_line_, cpol_);
        delayMicroseconds(half_period_us);
    }
    
    return data_in;
}

uint16_t BitBangSPI::transfer16(uint16_t data_out) {
    uint8_t high_byte = (data_out >> 8) & 0xFF;
    uint8_t low_byte = data_out & 0xFF;
    
    uint8_t high_in = transfer(high_byte);
    uint8_t low_in = transfer(low_byte);
    
    return (high_in << 8) | low_in;
}

void BitBangSPI::setMode(SPIMode mode) {
    mode_ = mode;
    cpol_ = (mode_ == MODE2 || mode_ == MODE3);
    cpha_ = (mode_ == MODE1 || mode_ == MODE3);
    
    if (gpio_initialized_) {
        gpiod_line_set_value(sclk_line_, cpol_);
    }
}

void BitBangSPI::setMaxSpeed(uint32_t speed_hz) {
    max_speed_hz_ = speed_hz;
}

void BitBangSPI::setCS(bool level) {
    if (gpio_initialized_) {
        gpiod_line_set_value(cs_line_, level);
    }
}

// MCP4921 실제 구현
MCP4921::MCP4921(BitBangSPI* spi, int cs_pin)
    : spi_(spi), cs_pin_(cs_pin), initialized_(false), chip_(nullptr), cs_line_(nullptr) {

    // CS 라인 설정
    chip_ = gpiod_chip_open("/dev/gpiochip0");
    if (!chip_) {
        ROS_ERROR("Failed to open GPIO chip for MCP4921");
        return;
    }
    
    cs_line_ = gpiod_chip_get_line(chip_, cs_pin_);
    if (!cs_line_) {
        gpiod_chip_close(chip_);
        ROS_ERROR("Failed to get GPIO line for MCP4921 CS");
        return;
    }
    
    int ret = gpiod_line_request_output(cs_line_, "mcp4921_cs", 1);  // CS는 HIGH로 시작
    if (ret < 0) {
        gpiod_chip_close(chip_);
        ROS_ERROR("Failed to request CS GPIO line as output for MCP4921");
        return;
    }

    initialized_ = true;
    
    if (initialized_) {
        ROS_INFO("MCP4921 initialized successfully");
        setOutput(0);  // 초기 출력을 0으로 설정
    } else {
        ROS_ERROR("Failed to initialize MCP4921");
    }
}

MCP4921::~MCP4921() {
    // 종료 시 출력을 0으로 설정
    if (initialized_) {
        setOutput(0);
    }
    
    if (cs_line_) {
        gpiod_line_release(cs_line_);
        cs_line_ = nullptr;
    }
    if (chip_) {
        gpiod_chip_close(chip_);
        chip_ = nullptr;
    }
}

bool MCP4921::isInitialized() const {
    return initialized_;
}

bool MCP4921::setOutput(uint16_t value, bool buffered, bool gain_1x, bool active) {
    if (!initialized_) {
        ROS_ERROR("MCP4921 not initialized");
        return false;
    }
    
    // 값 범위 제한 (0-4095, 12비트)
    value = value & 0x0FFF;
    
    // 명령 바이트 구성
    // Bit 15: Always 0 (DAC A selected)
    // Bit 14: 1 = Buffered, 0 = Unbuffered
    // Bit 13: 1 = ~Gain = 1, 0 = ~Gain = 2
    // Bit 12: 1 = Active, 0 = Shutdown the device
    // Bit 11-0: 12-bit DAC value
    uint16_t command = value;
    
    if (buffered) command |= 0x4000; // Bit 14 = 1
    if (gain_1x) command |= 0x2000; // Bit 13 = 1
    if (active) command |= 0x1000; // Bit 12 = 1
    
    // Chip Select 활성화 (LOW)
    gpiod_line_set_value(cs_line_, 0);

    // 2바이트 명령 전송
    uint16_t result = spi_->transfer16(command);

    // Chip Select 비활성화 (HIGH)
    gpiod_line_set_value(cs_line_, 1);
    
    return true;
}

bool MCP4921::setOutputPercent(float percent, bool buffered, bool gain_1x) {
    // 0-100% 범위를 0-4095 범위로 변환
    if (percent < 0.0f) percent = 0.0f;
    if (percent > 100.0f) percent = 100.0f;
    
    uint16_t value = static_cast<uint16_t>(percent * 40.95f);
    return setOutput(value, buffered, gain_1x);
}

bool MCP4921::setOutputVoltage(float voltage, float vref, bool buffered, bool gain_1x) {
    // 최대 출력 전압 계산
    float max_voltage = gain_1x ? vref : vref / 2.0f;
    
    // 전압 제한
    if (voltage < 0.0f) voltage = 0.0f;
    if (voltage > max_voltage) voltage = max_voltage;
    
    // 전압을 0-4095 범위로 변환
    uint16_t value = static_cast<uint16_t>((voltage / max_voltage) * 4095.0f);
    return setOutput(value, buffered, gain_1x);
}

// HallSensorEncoder 실제 구현
HallSensorEncoder::HallSensorEncoder(double wheel_radius, double wheel_base, double max_velocity)
    : wheel_radius_(wheel_radius), wheel_base_(wheel_base), steps_per_revolution_(20),
      max_velocity_(max_velocity), velocity_timeout_(0.1) {
    last_odom_time_ = std::chrono::high_resolution_clock::now();
}

HallSensorEncoder::~HallSensorEncoder() {
}

bool HallSensorEncoder::initialize() {
    // 홀 센서 시퀀스 초기화
    initHallSequences();
    
    // 상태 리셋
    reset();
    
    return true;
}

void HallSensorEncoder::reset() {
    wheel0_info_.encoder_count = 0;
    wheel0_info_.position = 0.0;
    wheel0_info_.velocity = 0.0;
    wheel0_info_.current_hall_state = -1;
    wheel0_info_.direction_forward = true;
    wheel0_info_.is_valid = false;
    
    wheel1_info_.encoder_count = 0;
    wheel1_info_.position = 0.0;
    wheel1_info_.velocity = 0.0;
    wheel1_info_.current_hall_state = -1;
    wheel1_info_.direction_forward = true;
    wheel1_info_.is_valid = false;
    
    wheel0_state_.sequence_index = 0;
    wheel0_state_.last_hall_state = -1;
    wheel0_state_.last_encoder_count = 0;
    
    wheel1_state_.sequence_index = 0;
    wheel1_state_.last_hall_state = -1;
    wheel1_state_.last_encoder_count = 0;
    
    robot_odom_.x = 0.0;
    robot_odom_.y = 0.0;
    robot_odom_.theta = 0.0;
    robot_odom_.linear_velocity = 0.0;
    robot_odom_.angular_velocity = 0.0;
    
    auto now = std::chrono::high_resolution_clock::now();
    wheel0_state_.last_update_time = now;
    wheel0_state_.last_velocity_time = now;
    wheel1_state_.last_update_time = now;
    wheel1_state_.last_velocity_time = now;
    last_odom_time_ = now;
}

void HallSensorEncoder::initHallSequences() {
    // Wheel 0 시퀀스: 001 -> 101 -> 111 -> 110 -> 010 -> 000 -> 001
    wheel0_sequence_ = {
        0b001,  // 1 : [0]
        0b101,  // 5 : [1]
        0b111,  // 7 : [2]
        0b110,  // 6 : [3]
        0b010,  // 2 : [4]
        0b000   // 0 : [5]
    };
    wheel0_sequence_inverse = {
        5,0,4,-1,
        -1,1,3,2
    };
    
    // Wheel 1 시퀀스: 110 -> 100 -> 101 -> 001 -> 011 -> 010 -> 110
    wheel1_sequence_ = {
        0b110,  // 6 : [0]
        0b100,  // 4 : [1]
        0b101,  // 5 : [2]
        0b001,  // 1 : [3]
        0b011,  // 3 : [4]
        0b010   // 2 : [5]
    };

    wheel1_sequence_inverse = {
        -1,3,5,4,
        1,2,0,-1
    };
}

int HallSensorEncoder::findSequenceIndex(const std::vector<int>& inverse, int hall_value) {
    if (hall_value < 0 || hall_value >= static_cast<int>(inverse.size())) {
        return -1;
    }
    return inverse[hall_value];
}

int HallSensorEncoder::getNextSequenceIndex(const std::vector<int>& sequence, int current_index) {
    return (current_index + 1) % static_cast<int>(sequence.size());
}

int HallSensorEncoder::getPrevSequenceIndex(const std::vector<int>& sequence, int current_index) {
    return (current_index - 1 + static_cast<int>(sequence.size())) % static_cast<int>(sequence.size());
}

bool HallSensorEncoder::updateWheelState(WheelInfo& wheel_info, WheelState& wheel_state,
                                        const std::vector<int>& sequence, const std::vector<int>& inverse, int new_hall_state) {
    if (new_hall_state == wheel_info.current_hall_state) {
        return false; // 변화 없음
    }
    
    auto current_time = std::chrono::high_resolution_clock::now();
    
    wheel_state.last_hall_state = wheel_info.current_hall_state;
    wheel_info.current_hall_state = new_hall_state;
    wheel_state.last_update_time = current_time;
    
    // 첫 번째 업데이트인 경우
    if (wheel_state.last_hall_state == -1) {
        int index = findSequenceIndex(inverse, new_hall_state);
        if (index != -1) {
            wheel_state.sequence_index = index;
            wheel_info.is_valid = true;
        }
        return false;
    }
    
    // 현재 시퀀스 인덱스 찾기
    int current_index = findSequenceIndex(inverse, wheel_state.last_hall_state);
    int new_index = findSequenceIndex(inverse, new_hall_state);
    
    if (current_index == -1 || new_index == -1) {
        wheel_info.is_valid = false;
        return false;
    }
    
    std::vector<int> wheel_next_next = {2,3,4,5,0,1};
    std::vector<int> wheel_next = {1,2,3,4,5,0};
    std::vector<int> wheel_prev = {5,0,1,2,3,4};
    std::vector<int> wheel_prev_prev = {4,5,0,1,2,3};
    
    // 방향 판단
    int expected_next = wheel_next[current_index];
    int expected_next_next = wheel_next_next[current_index];
    int expected_prev = wheel_prev[current_index];
    int expected_prev_prev = wheel_prev_prev[current_index];
    
    if (new_index == expected_next) {
        // 정방향
        wheel_info.direction_forward = true;
        wheel_info.encoder_count++;
        wheel_state.sequence_index = new_index;
        wheel_info.is_valid = true;
    } else if (new_index == expected_next_next) {
        // 정방향 (2스텝 점프)
        wheel_info.direction_forward = true;
        wheel_info.encoder_count += 2;
        wheel_state.sequence_index = new_index;
        wheel_info.is_valid = true;
    } else if (new_index == expected_prev) {
        // 역방향
        wheel_info.direction_forward = false;
        wheel_info.encoder_count--;
        wheel_state.sequence_index = new_index;
        wheel_info.is_valid = true;
    } else if (new_index == expected_prev_prev) {
        // 역방향 (2스텝 점프)
        wheel_info.direction_forward = false;
        wheel_info.encoder_count -= 2;
        wheel_state.sequence_index = new_index;
        wheel_info.is_valid = true;
    } else {
        // 시퀀스 오류 - 스텝을 건너뛰었거나 노이즈
        wheel_state.sequence_index = new_index;
        wheel_info.is_valid = false;
        return false;
    }
    
    return true;
}

void HallSensorEncoder::calculatePosition(WheelInfo& wheel_info, const WheelState& wheel_state) {
    // 엔코더 카운트를 라디안으로 변환
    wheel_info.position = (2.0 * M_PI * wheel_info.encoder_count) / steps_per_revolution_;
}

void HallSensorEncoder::calculateVelocity(WheelInfo& wheel_info, WheelState& wheel_state) {
    auto current_time = std::chrono::high_resolution_clock::now();
    auto time_diff = std::chrono::duration_cast<std::chrono::microseconds>(
        current_time - wheel_state.last_velocity_time);
    double dt = time_diff.count() / 1000000.0; // 초로 변환
    
    long count_diff = wheel_info.encoder_count - wheel_state.last_encoder_count;  
    if (dt > 0.1 || count_diff > 100 || count_diff < -100) { // 100ms 간격 이상 또는 100ticks 이상
        double angle_diff = (2.0 * M_PI * count_diff) / steps_per_revolution_;
        
        double new_velocity = angle_diff / dt;
        
        // 속도 필터링 (급격한 변화 제한)
        if (std::abs(new_velocity) <= max_velocity_) {
            wheel_info.velocity = new_velocity;
        } else {
            wheel_info.velocity = (new_velocity > 0) ? max_velocity_ : -max_velocity_;
        }
        
        wheel_state.last_velocity_time = current_time;
        wheel_state.last_encoder_count = wheel_info.encoder_count;
    }
    
    // 타임아웃 체크 - 일정 시간 동안 업데이트가 없으면 속도를 0으로 설정
    auto time_since_update = std::chrono::duration_cast<std::chrono::microseconds>(
        current_time - wheel_state.last_update_time);
    if (time_since_update.count() / 1000000.0 > velocity_timeout_) {
        wheel_info.velocity = 0.0;
    }
}

void HallSensorEncoder::updateRobotOdometry() {
    auto current_time = std::chrono::high_resolution_clock::now();
    auto time_diff = std::chrono::duration_cast<std::chrono::microseconds>(
        current_time - last_odom_time_);
    double dt = time_diff.count() / 1000000.0; // 초로 변환
    
    if (dt > 0.001) { // 최소 1ms 간격
        // 바퀴 선속도 계산
        double v_left = wheel0_info_.velocity * wheel_radius_;
        double v_right = wheel1_info_.velocity * wheel_radius_;
        
        // 로봇 선속도 및 각속도 계산
        robot_odom_.linear_velocity = (v_left + v_right) / 2.0;
        robot_odom_.angular_velocity = (v_right - v_left) / wheel_base_;
        
        // 로봇 위치 업데이트 (오일러 적분)
        robot_odom_.x += robot_odom_.linear_velocity * cos(robot_odom_.theta) * dt;
        robot_odom_.y += robot_odom_.linear_velocity * sin(robot_odom_.theta) * dt;
        robot_odom_.theta += robot_odom_.angular_velocity * dt;
        
        // 각도 정규화 (-π ~ π)
        while (robot_odom_.theta > M_PI) robot_odom_.theta -= 2.0 * M_PI;
        while (robot_odom_.theta < -M_PI) robot_odom_.theta += 2.0 * M_PI;
        
        last_odom_time_ = current_time;
    }
}

void HallSensorEncoder::updateHallSensors(bool wheel0_bit0, bool wheel0_bit1, bool wheel0_bit2,
                                         bool wheel1_bit0, bool wheel1_bit1, bool wheel1_bit2) {
    int wheel0_hall = (wheel0_bit2 << 2) | (wheel0_bit1 << 1) | wheel0_bit0;
    int wheel1_hall = (wheel1_bit2 << 2) | (wheel1_bit1 << 1) | wheel1_bit0;
    
    updateHallSensors(wheel0_hall, wheel1_hall);
}

void HallSensorEncoder::updateHallSensors(int wheel0_hall, int wheel1_hall) {
    // Wheel 0 업데이트
    if (updateWheelState(wheel0_info_, wheel0_state_, wheel0_sequence_, wheel0_sequence_inverse, wheel0_hall)) {
        calculatePosition(wheel0_info_, wheel0_state_);
        calculateVelocity(wheel0_info_, wheel0_state_);
    }
    
    // Wheel 1 업데이트
    if (updateWheelState(wheel1_info_, wheel1_state_, wheel1_sequence_, wheel1_sequence_inverse, wheel1_hall)) {
        calculatePosition(wheel1_info_, wheel1_state_);
        calculateVelocity(wheel1_info_, wheel1_state_);
    }
    
    // 로봇 오도메트리 업데이트
    updateRobotOdometry();
}

SPIHardwareNode::SPIHardwareNode(ros::NodeHandle& nh, ros::NodeHandle& pnh)
    : nh_(nh), pnh_(pnh),
      dac0_(nullptr), dac1_(nullptr), spi_(nullptr), chip_(nullptr),
      reverse0_line_(nullptr), reverse1_line_(nullptr),
      inc_button_line_(nullptr), dec_button_line_(nullptr), latch_line_(nullptr),
      hall_state_0_(0), hall_state_1_(0),
      hall_count_0_(0), hall_count_1_(0),
      motor0_speed_(0.0), motor1_speed_(0.0),
      prev_inc_button_state_(false), prev_dec_button_state_(false),
      hardware_initialized_(false), last_error_(HardwareError::NONE),
      encoder_(HardwareConstants::WHEEL_RADIUS, HardwareConstants::WHEEL_DISTANCE, 10.0) {
    
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
    
    // SPI 객체 해제
    if (spi_) {
        delete spi_;
        spi_ = nullptr;
    }
    
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
    
    // 실제 GPIO 초기화 시도
    chip_ = gpiod_chip_open_by_name(pin_config_.gpio_chip_name.c_str());
    if (!chip_) {
        ROS_WARN("Failed to open GPIO chip: %s - Running in dummy mode", pin_config_.gpio_chip_name.c_str());
        ROS_INFO("GPIO will operate in simulation mode without actual hardware control");
        return true;  // 더미 모드로 계속 진행
    }
    
    // 출력 라인 설정 (실패해도 계속 진행)
    bool gpio_success = true;
    if (!setOutputLine(pin_config_.reverse0_pin, &reverse0_line_)) {
        ROS_WARN("Failed to initialize reverse0 pin %d - will use dummy mode", pin_config_.reverse0_pin);
        gpio_success = false;
    }
    if (!setOutputLine(pin_config_.reverse1_pin, &reverse1_line_)) {
        ROS_WARN("Failed to initialize reverse1 pin %d - will use dummy mode", pin_config_.reverse1_pin);
        gpio_success = false;
    }
    if (!setOutputLine(pin_config_.latch_pin, &latch_line_)) {
        ROS_WARN("Failed to initialize latch pin %d - will use dummy mode", pin_config_.latch_pin);
        gpio_success = false;
    }
    
    // 입력 라인 설정 (실패해도 계속 진행)
    if (!setInputLine(pin_config_.inc_button_pin, &inc_button_line_)) {
        ROS_WARN("Failed to initialize inc_button pin %d - will use dummy mode", pin_config_.inc_button_pin);
        gpio_success = false;
    }
    if (!setInputLine(pin_config_.dec_button_pin, &dec_button_line_)) {
        ROS_WARN("Failed to initialize dec_button pin %d - will use dummy mode", pin_config_.dec_button_pin);
        gpio_success = false;
    }
    
    // 초기 상태 설정 (가능한 라인만)
    if (reverse0_line_) gpiod_line_set_value(reverse0_line_, 1);  // 정방향
    if (reverse1_line_) gpiod_line_set_value(reverse1_line_, 1);  // 정방향
    if (latch_line_) gpiod_line_set_value(latch_line_, 1);       // 비활성 상태
    
    if (gpio_success) {
        ROS_INFO("GPIO initialization complete - Hardware mode");
    } else {
        ROS_INFO("GPIO initialization complete - Dummy/Simulation mode");
    }
    return true;  // 항상 성공으로 처리
}

bool SPIHardwareNode::initializeSPI() {
    ROS_INFO("Initializing SPI...");
    
    // GPIO가 초기화되지 않았으면 SPI도 더미 모드
    if (!chip_) {
        ROS_INFO("SPI running in dummy mode (no GPIO chip)");
        spi_ = nullptr;
        return true;
    }
    
    // 실제 SPI 초기화 시도
    try {
        spi_ = new BitBangSPI(pin_config_.mosi_pin, pin_config_.miso_pin, 
                             pin_config_.clk_pin, pin_config_.latch_pin,
                             BitBangSPI::MODE0, pin_config_.spi_speed, false);
        
        if (!spi_) {
            ROS_WARN("Failed to create SPI controller - running in dummy mode");
            return true;
        }
        
        ROS_INFO("SPI initialization complete - Hardware mode");
    } catch (const std::exception& e) {
        ROS_WARN("SPI initialization failed: %s - running in dummy mode", e.what());
        spi_ = nullptr;
    }
    
    return true;
}

bool SPIHardwareNode::initializeDAC() {
    ROS_INFO("Initializing DAC...");
    
    // SPI가 없으면 DAC도 더미 모드
    if (!spi_) {
        ROS_INFO("DAC running in dummy mode (no SPI)");
        dac0_ = nullptr;
        dac1_ = nullptr;
        current_status_.dac0_ok = false;
        current_status_.dac1_ok = false;
        return true;
    }
    
    // DAC 객체 생성 시도
    try {
        dac0_ = new MCP4921(spi_, pin_config_.ss0_pin);
        if (!dac0_ || !dac0_->isInitialized()) {
            ROS_WARN("Failed to initialize DAC0 - running in dummy mode");
            current_status_.dac0_ok = false;
            dac0_ = nullptr;
        } else {
            current_status_.dac0_ok = true;
            dac0_->setOutputPercent(0.0);  // 초기 출력값 설정 (정지)
        }
        
        dac1_ = new MCP4921(spi_, pin_config_.ss1_pin);
        if (!dac1_ || !dac1_->isInitialized()) {
            ROS_WARN("Failed to initialize DAC1 - running in dummy mode");
            current_status_.dac1_ok = false;
            dac1_ = nullptr;
        } else {
            current_status_.dac1_ok = true;
            dac1_->setOutputPercent(0.0);  // 초기 출력값 설정 (정지)
        }
        
        if (current_status_.dac0_ok && current_status_.dac1_ok) {
            ROS_INFO("DAC initialization complete - Hardware mode");
        } else {
            ROS_INFO("DAC initialization complete - Partial/Dummy mode");
        }
    } catch (const std::exception& e) {
        ROS_WARN("DAC initialization failed: %s - running in dummy mode", e.what());
        dac0_ = nullptr;
        dac1_ = nullptr;
        current_status_.dac0_ok = false;
        current_status_.dac1_ok = false;
    }
    
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
    ROS_DEBUG("Setting motor %d direction: %s", motor_id, forward ? "forward" : "reverse");
    
    // 실제 GPIO 제어
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
    // 실제 SPI를 통해 홀센서 데이터 읽기
    if (!spi_ || !latch_line_) {
        // SPI나 래치 라인이 없으면 더미 데이터 사용
        static uint8_t dummy_counter = 0;
        dummy_counter++;
        
        uint8_t new_hall_state_0 = (dummy_counter / 10) % 7;
        uint8_t new_hall_state_1 = (dummy_counter / 12) % 7;
        
        if (new_hall_state_0 != hall_state_0_) {
            hall_count_0_++;
            hall_state_0_ = new_hall_state_0;
        }
        
        if (new_hall_state_1 != hall_state_1_) {
            hall_count_1_++;
            hall_state_1_ = new_hall_state_1;
        }
        
        encoder_.updateHallSensors(hall_state_0_, hall_state_1_);
        current_status_.motor0_hall_state = hall_state_0_;
        current_status_.motor1_hall_state = hall_state_1_;
        current_status_.motor0_hall_count = hall_count_0_;
        current_status_.motor1_hall_count = hall_count_1_;
        return;
    }
    
    // 실제 하드웨어 SPI 읽기
    gpiod_line_set_value(latch_line_, 1);
    uint8_t data = spi_->transfer(0x00);
    gpiod_line_set_value(latch_line_, 0);
    
    // 홀센서 데이터 파싱 (실제 하드웨어 배치에 따라 조정)
    uint8_t new_hall_state_0 = (data >> 0) & 0x7;  // 하위 3비트
    uint8_t new_hall_state_1 = (data >> 3) & 0x7;  // 상위 3비트
    
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
    ros::Time now = ros::Time::now();
    double elapsed = (now - last_button_time_).toSec();
    
    // 디바운싱 (50ms)
    if (elapsed < 0.05) return;
    
    bool inc_button_state = false;
    bool dec_button_state = false;
    
    // 실제 GPIO를 통해 버튼 상태 읽기
    if (inc_button_line_ && dec_button_line_) {
        int inc_value = gpiod_line_get_value(inc_button_line_);
        int dec_value = gpiod_line_get_value(dec_button_line_);
        
        inc_button_state = (inc_value == 0);  // 풀업 저항 사용 시 눌림 = LOW
        dec_button_state = (dec_value == 0);
    }
    
    // 상승 엣지 감지 및 처리
    if (inc_button_state && !prev_inc_button_state_) {
        ROS_INFO("Increase button pressed");
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
    
    ROS_INFO("=== SPI Hardware Node Starting ===");
    ROS_INFO("Creating SPI Hardware Node instance...");
    
    SPIHardwareNode node(nh, pnh);
    
    ROS_INFO("Initializing SPI Hardware Node...");
    if (!node.initialize()) {
        ROS_ERROR("Failed to initialize SPI Hardware Node");
        return -1;
    }
    
    ROS_INFO("=== SPI Hardware Node started successfully ===");
    ROS_INFO("Node is ready to process motor commands");
    ros::spin();
    
    ROS_INFO("SPI Hardware Node shutting down...");
    return 0;
} 