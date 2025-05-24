// src/motor_controller.cpp
#include "motor_controller/motor_controller.h"
#include <chrono>
#include <iostream>
#include <cmath>
#include <unistd.h>

MotorController::MotorController(ros::NodeHandle& nh, ros::NodeHandle& pnh)
    : nh_(nh), pnh_(pnh), dac_(nullptr), chip_(nullptr), reverse_line_(nullptr),
      hall_sensor_line_(nullptr), inc_button_line_(nullptr), dec_button_line_(nullptr),
      target_speed_(10.0), actual_speed_(0.0), prev_hall_state_(false), hall_count_(0),
      prev_inc_button_state_(false), prev_dec_button_state_(false) {
    
    // 매개변수 로드
    pnh_.param<std::string>("spi_device", spi_device_, "/dev/spidev1.0");
    pnh_.param<int>("spi_speed", spi_speed_, 1000000);
    pnh_.param<int>("mosi_pin", mosi_pin_, 16); // 물리적 핀 7에 해당하는 GPIO 번호
    pnh_.param<int>("miso_pin", miso_pin_, 17); // 물리적 핀 7에 해당하는 GPIO 번호
    pnh_.param<int>("clk_pin", clk_pin_, 18); // 물리적 핀 7에 해당하는 GPIO 번호
    pnh_.param<int>("ss0_pin", ss0_pin_, 19); // 물리적 핀 7에 해당하는 GPIO 번호
    pnh_.param<int>("ss1_pin", ss1_pin_, 20); // 물리적 핀 7에 해당하는 GPIO 번호
    pnh_.param<int>("reverse_pin", reverse_pin_, 168); // 물리적 핀 7에 해당하는 GPIO 번호
    pnh_.param<int>("hall_sensor_pin", hall_sensor_pin_, 38); // 물리적 핀 13에 해당하는 GPIO 번호
    pnh_.param<int>("inc_button_pin", inc_button_pin_, 232/*23*/); // 물리적 핀 16에 해당하는 GPIO 번호
    pnh_.param<int>("dec_button_pin", dec_button_pin_, 15/*24*/); // 물리적 핀 18에 해당하는 GPIO 번호
    pnh_.param<float>("max_speed", max_speed_, 100.0);
    pnh_.param<std::string>("gpio_chip_name", gpio_chip_name_, "gpiochip0");
    
    // GPIO 설정
    setupGPIO();
    
    // DAC 초기화
    dac_ = new MCP4921(mosi_pin_,miso_pin_,clk_pin_,ss0_pin_, spi_speed_);
    if (!dac_ || !dac_->isInitialized()) {
        ROS_ERROR("Failed to initialize DAC");
        ros::shutdown();
        return;
    }
    
    // 초기화 - 정지 상태로 시작
    dac_->setOutputPercent(0.0);
    setMotorDirection(true); // 초기 방향: 정방향
    
    // ROS 토픽 구독/발행
    speed_sub_ = nh_.subscribe("target_speed", 10, &MotorController::speedCallback, this);
    actual_speed_pub_ = nh_.advertise<std_msgs::Float32>("actual_speed", 10);
    
    // 타이머 설정
    timer_ = nh_.createTimer(ros::Duration(0.05), &MotorController::timerCallback, this);
    
    // 초기 시간 설정
    last_hall_time_ = ros::Time::now();
    last_button_time_ = ros::Time::now();
    
    ROS_INFO("Motor controller initialized");
}
MotorController::~MotorController() {
    // DAC 출력을 0으로 설정
    if (dac_) {
        dac_->setOutputPercent(0.0);
        delete dac_;
    }
    
    // GPIO 정리
    cleanupGPIO();
    
    ROS_INFO("Motor controller shutdown");
}
void MotorController::setupGPIO() {
    // gpiod 칩 열기
    chip_ = gpiod_chip_open_by_name(gpio_chip_name_.c_str());
    if (!chip_) {
        ROS_ERROR("Failed to open GPIO chip: %s", gpio_chip_name_.c_str());
        return;
    }
    
    // 리버스 핀 설정 (출력)
    reverse_line_ = gpiod_chip_get_line(chip_, reverse_pin_);
    if (!reverse_line_) {
        ROS_ERROR("Failed to get reverse GPIO line: %d", reverse_pin_);
        return;
    }
    
    if (gpiod_line_request_output(reverse_line_, "motor_controller", 1) < 0) {
        ROS_ERROR("Failed to request reverse GPIO line as output");
        return;
    }
    
    // 홀센서 핀 설정 (입력, 풀업)
    hall_sensor_line_ = gpiod_chip_get_line(chip_, hall_sensor_pin_);
    if (!hall_sensor_line_) {
        ROS_ERROR("Failed to get hall sensor GPIO line: %d", hall_sensor_pin_);
        return;
    }
    
    if (gpiod_line_request_input(hall_sensor_line_, "motor_controller") < 0) {
        ROS_ERROR("Failed to request hall sensor GPIO line as input");
        return;
    }
    
    // 증가 버튼 핀 설정 (입력, 풀업)
    inc_button_line_ = gpiod_chip_get_line(chip_, inc_button_pin_);
    if (!inc_button_line_) {
        ROS_ERROR("Failed to get increment button GPIO line: %d", inc_button_pin_);
        return;
    }
    
    if (gpiod_line_request_input(inc_button_line_, "motor_controller") < 0) {
        ROS_ERROR("Failed to request increment button GPIO line as input");
        return;
    }
    
    // 감소 버튼 핀 설정 (입력, 풀업)
    dec_button_line_ = gpiod_chip_get_line(chip_, dec_button_pin_);
    if (!dec_button_line_) {
        ROS_ERROR("Failed to get decrement button GPIO line: %d", dec_button_pin_);
        return;
    }
    
    if (gpiod_line_request_input(dec_button_line_, "motor_controller") < 0) {
        ROS_ERROR("Failed to request decrement button GPIO line as input");
        return;
    }
    
    ROS_INFO("GPIO setup complete");
}
void MotorController::cleanupGPIO() {
    // GPIO 라인 해제
    if (reverse_line_) {
        gpiod_line_release(reverse_line_);
        reverse_line_ = nullptr;
    }
    
    if (hall_sensor_line_) {
        gpiod_line_release(hall_sensor_line_);
        hall_sensor_line_ = nullptr;
    }
    
    if (inc_button_line_) {
        gpiod_line_release(inc_button_line_);
        inc_button_line_ = nullptr;
    }
    
    if (dec_button_line_) {
        gpiod_line_release(dec_button_line_);
        dec_button_line_ = nullptr;
    }
    
    // GPIO 칩 닫기
    if (chip_) {
        gpiod_chip_close(chip_);
        chip_ = nullptr;
    }
    
    ROS_INFO("GPIO cleanup complete");
}
void MotorController::speedCallback(const std_msgs::Float32::ConstPtr& msg) {
    // 목표 속도 업데이트
    target_speed_ = msg->data;
    
    // 속도 범위 제한
    if (target_speed_ > max_speed_) target_speed_ = max_speed_;
    if (target_speed_ < -max_speed_) target_speed_ = -max_speed_;
    
    ROS_INFO("Target speed set to: %.2f", target_speed_);
}
void MotorController::timerCallback(const ros::TimerEvent& event) {
    // 홀센서 및 버튼 상태 확인
    checkHallSensor();
    checkButtons();
    
    // 모터 제어
    updateActualSpeed();
    setMotorDirection(target_speed_ >= 0);
    setMotorSpeed(std::abs(target_speed_));
    
    // 현재 속도 발행
    std_msgs::Float32 speed_msg;
    speed_msg.data = actual_speed_;
    actual_speed_pub_.publish(speed_msg);
    
    ROS_INFO("Target: %.2f, Actual: %.2f RPM", target_speed_, actual_speed_);
}
void MotorController::checkHallSensor() {
    if (!hall_sensor_line_) return;
    
    // 홀센서 상태 읽기
    int hall_value = gpiod_line_get_value(hall_sensor_line_);
    bool hall_state = (hall_value == 1);
    
    // 상승 엣지 감지
    if (hall_state && !prev_hall_state_) {
        hall_count_++;
        
        // 현재 시간과 마지막 감지 시간 간의 차이 계산
        ros::Time now = ros::Time::now();
        double duration = (now - last_hall_time_).toSec();
        
        // 속도 계산 (RPM)
        if (duration > 0.001) { // 1ms 이상의 간격
            float rpm = 60.0 / duration; // 1분당 회전수 (RPM)
            actual_speed_ = (target_speed_ >= 0) ? rpm : -rpm;
        }
        
        last_hall_time_ = now;
    }
    
    prev_hall_state_ = hall_state;
}
void MotorController::checkButtons() {
    if (!inc_button_line_ || !dec_button_line_) return;
    
    // 디바운싱을 위한 타임 체크
    ros::Time now = ros::Time::now();
    double elapsed = (now - last_button_time_).toSec();
    
    // 최소 50ms의 버튼 디바운싱 간격
    if (elapsed < 0.05) return;
    
    // 버튼 상태 읽기
    int inc_value = gpiod_line_get_value(inc_button_line_);
    int dec_value = gpiod_line_get_value(dec_button_line_);
    
    bool inc_button_state = (inc_value == 0); // 풀업 저항 사용 시 누르면 0
    bool dec_button_state = (dec_value == 0); // 풀업 저항 사용 시 누르면 0
    
    // 증가 버튼 상승 엣지 감지
    if (inc_button_state && !prev_inc_button_state_) {
        target_speed_ += 5.0;
        if (target_speed_ > max_speed_) target_speed_ = max_speed_;
        ROS_INFO("Speed increased: %.2f", target_speed_);
        last_button_time_ = now;
    }
    
    // 감소 버튼 상승 엣지 감지
    if (dec_button_state && !prev_dec_button_state_) {
        target_speed_ -= 5.0;
        if (target_speed_ < -max_speed_) target_speed_ = -max_speed_;
        ROS_INFO("Speed decreased: %.2f", target_speed_);
        last_button_time_ = now;
    }
    
    prev_inc_button_state_ = inc_button_state;
    prev_dec_button_state_ = dec_button_state;
}
void MotorController::updateActualSpeed() {
    // 홀센서 신호가 없으면 정지 상태 감지
    ros::Time now = ros::Time::now();
    double duration = (now - last_hall_time_).toSec();
    
    // 1초 이상 신호가 없으면 속도를 0으로 간주
    if (duration > 1.0) {
        actual_speed_ = 0.0;
    }
}
void MotorController::setMotorDirection(bool forward) {
    if (!reverse_line_) return;
    
    // gpiod를 사용하여 리버스 핀 제어
    gpiod_line_set_value(reverse_line_, forward ? 1 : 0);
}
void MotorController::setMotorSpeed(float speed) {
    if (!dac_) return;
    
    // 속도 범위 제한 및 퍼센트 변환
    float speed_percent = (speed / max_speed_) * 100.0;
    if (speed_percent < 0.0f) speed_percent = 0.0f;
    if (speed_percent > 100.0f) speed_percent = 100.0f;
    
    // DAC 출력 설정
    dac_->setOutputPercent(speed_percent);
}
// 메인 함수
int main(int argc, char** argv) {
    ros::init(argc, argv, "motor_controller");
    ros::NodeHandle nh;
    ros::NodeHandle pnh("~");
    
    MotorController controller(nh, pnh);
    
    ros::spin();
    
    return 0;
}
