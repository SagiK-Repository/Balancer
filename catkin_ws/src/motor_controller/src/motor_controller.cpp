// src/motor_controller.cpp
#include "motor_controller/motor_controller.h"
#include <chrono>
#include <iostream>
#include <cmath>
#include <unistd.h>

MotorController::MotorController(ros::NodeHandle& nh, ros::NodeHandle& pnh)
    : nh_(nh), pnh_(pnh), dac0_(nullptr), dac1_(nullptr), chip_(nullptr), reverse0_line_(nullptr), 
      reverse1_line_(nullptr), inc_button_line_(nullptr), dec_button_line_(nullptr),
      target_speed_0(0.0), actual_speed_0(0.0), target_speed_1(0.0), actual_speed_1(0.0),
      prev_hall_state_(false), hall_count_(0), prev_inc_button_state_(false), prev_dec_button_state_(false),
      encoder(WHEEL_RADIUS,WHEEL_DISTANCE,999)
{
    
    // 매개변수 로드
    pnh_.param<std::string>("spi_device", spi_device_, "/dev/spidev1.0");
    pnh_.param<int>("spi_speed", spi_speed_, 100000000);
    pnh_.param<int>("mosi_pin", mosi_pin_, 16); // 물리적 핀 7에 해당하는 GPIO 번호
    pnh_.param<int>("miso_pin", miso_pin_, 17); // 물리적 핀 7에 해당하는 GPIO 번호
    pnh_.param<int>("clk_pin", clk_pin_, 18); // 물리적 핀 7에 해당하는 GPIO 번호
    pnh_.param<int>("ss0_pin", ss0_pin_, 19); // 물리적 핀 7에 해당하는 GPIO 번호
    pnh_.param<int>("ss1_pin", ss1_pin_, 20); // 물리적 핀 7에 해당하는 GPIO 번호
    pnh_.param<int>("reverse0_pin", reverse0_pin_, 168); // 물리적 핀 7에 해당하는 GPIO 번호
    pnh_.param<int>("reverse1_pin", reverse1_pin_, 13); // 물리적 핀 7에 해당하는 GPIO 번호
    //pnh_.param<int>("hall_sensor_pin", hall_sensor_pin_, 38); // 물리적 핀 13에 해당하는 GPIO 번호

    pnh_.param<int>("latch_pin", latch_pin_, 38); // 물리적 핀 7에 해당하는 GPIO 번호

    pnh_.param<int>("inc_button_pin", inc_button_pin_, 232/*23*/); // 물리적 핀 16에 해당하는 GPIO 번호
    pnh_.param<int>("dec_button_pin", dec_button_pin_, 15/*24*/); // 물리적 핀 18에 해당하는 GPIO 번호
    pnh_.param<float>("max_speed", max_speed_, 100.0);
    pnh_.param<std::string>("gpio_chip_name", gpio_chip_name_, "gpiochip0");
    
    // GPIO 설정
    setupGPIO();

    // SPI 초기화
    spi = BitBangSPI(mosi_pin_, miso_pin_, clk_pin_, BitBangSPI::MODE0, spi_speed_,false);

    // 바퀴 수 계산기 초기화
    encoder.initialize();

    // CS 사용 과정
    chip = gpiod_chip_open("/dev/gpiochip0");
    if (!chip) {ROS_ERROR("Failed to open GPIO latch chip");return;}
    latch_line = gpiod_chip_get_line(chip, latch_pin_);
    if (!latch_line) {gpiod_chip_close(chip);ROS_ERROR("Failed to get GPIO lines");}
    int ret = gpiod_line_request_output(latch_line, "latch", 1);  // CS는 HIGH로 시작
    if (ret < 0) {gpiod_chip_close(chip);ROS_ERROR("Failed to request CS GPIO line as output");return;}

    // DAC 초기화
    dac0_ = new MCP4921(&spi,ss0_pin_);
    if (!dac0_ || !dac0_->isInitialized()) {
        ROS_ERROR("Failed to initialize DAC");
        ros::shutdown();
        return;
    }
    dac1_ = new MCP4921(&spi,ss1_pin_);
    if (!dac1_ || !dac1_->isInitialized()) {
        ROS_ERROR("Failed to initialize DAC");
        ros::shutdown();
        return;
    }
    // 초기화 - 정지 상태로 시작
    dac0_->setOutputPercent(0.0);
    setMotor0Direction(true); // 초기 방향: 정방향

    dac1_->setOutputPercent(0.0);
    setMotor1Direction(true); // 초기 방향: 정방향
    
    i2c.init();

    // ROS 토픽 구독/발행
    speed0_sub_ = nh_.subscribe("target_speed0", 10, &MotorController::speed0Callback, this);
    actual_speed0_pub_ = nh_.advertise<std_msgs::Float32>("actual_speed0", 10);
    
    speed1_sub_ = nh_.subscribe("target_speed1", 10, &MotorController::speed1Callback, this);
    actual_speed1_pub_ = nh_.advertise<std_msgs::Float32>("actual_speed1", 10);
    
    // 타이머 설정
    timer0_ = nh_.createTimer(ros::Duration(0.05), &MotorController::timerCallback, this);
    
    timer1_ = nh_.createTimer(ros::Duration(0.0001), &MotorController::checkHallSensor, this);
    
    // 초기 시간 설정
    last_hall_time_ = ros::Time::now();
    last_button_time_ = ros::Time::now();
    
    ROS_INFO("Motor controller initialized");
}
MotorController::~MotorController() {
    // DAC 출력을 0으로 설정
    if (dac0_) {
        dac0_->setOutputPercent(0.0);
        dac1_->setOutputPercent(0.0);
        delete dac0_;
        delete dac1_;
    }
    
    // GPIO 정리
    cleanupGPIO();
    
    ROS_INFO("Motor controller shutdown");
}
bool MotorController::setOutputLine(int pin,struct gpiod_line** pline){
    *pline = gpiod_chip_get_line(chip_, pin);
    if (!*pline) {
        ROS_ERROR("Failed to get GPIO line: %d", pin);
        return false;
    }
    
    if (gpiod_line_request_output(*pline, "motor_controller", 1) < 0) {
        ROS_ERROR("Failed to request GPIO line as output: pin %d",pin);
        return false;
    }
    return true;
}
bool MotorController::setInputLine(int pin,struct gpiod_line** pline){
    *pline = gpiod_chip_get_line(chip_, pin);
    if (!*pline) {
        ROS_ERROR("Failed to get GPIO line: %d", pin);
        return false;
    }
    
    if (gpiod_line_request_input(*pline, "motor_controller") < 0) {
        ROS_ERROR("Failed to request GPIO line as output: pin %d",pin);
        return false;
    }
    return true;
}
void MotorController::setupGPIO()
{
    // gpiod 칩 열기
    chip_ = gpiod_chip_open_by_name(gpio_chip_name_.c_str());
    if (!chip_) {
        ROS_ERROR("Failed to open GPIO chip: %s", gpio_chip_name_.c_str());
        return;
    }
    
    // 리버스 핀 설정 (출력)
    if (!setOutputLine(reverse0_pin_, &reverse0_line_))return;
    if (!setOutputLine(reverse1_pin_, &reverse1_line_))return;
    
    // 홀센서 핀 설정 (입력, 풀업)
    //if (!setInputLine(hall_sensor_pin_, &hall_sensor_line_))return;

    // 증가 버튼 핀 설정 (입력, 풀업)
    if (!setInputLine(inc_button_pin_, &inc_button_line_))return;
    
    // 감소 버튼 핀 설정 (입력, 풀업)
    if (!setInputLine(dec_button_pin_, &dec_button_line_))return;
    
    ROS_INFO("GPIO setup complete");
}
void MotorController::cleanupGPIO() {
    // GPIO 라인 해제
    if (reverse1_line_) {
        gpiod_line_release(reverse1_line_);
        reverse1_line_ = nullptr;
    }

    if (reverse0_line_) {
        gpiod_line_release(reverse0_line_);
        reverse0_line_ = nullptr;
    }
    
    // if (hall_sensor_line_) {
    //     gpiod_line_release(hall_sensor_line_);
    //     hall_sensor_line_ = nullptr;
    // }
    
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
void MotorController::speed0Callback(const std_msgs::Float32::ConstPtr& msg) {
    // 목표 속도 업데이트
    target_speed_0 = msg->data;
    
    // 속도 범위 제한
    if (target_speed_0 > max_speed_) target_speed_0 = max_speed_;
    if (target_speed_0 < -max_speed_) target_speed_0 = -max_speed_;
    
    ROS_INFO("Target speed set to: %.2f", target_speed_0);
}
void MotorController::speed1Callback(const std_msgs::Float32::ConstPtr& msg) {
    // 목표 속도 업데이트
    target_speed_1 = msg->data;
    
    // 속도 범위 제한
    if (target_speed_1 > max_speed_) target_speed_1 = max_speed_;
    if (target_speed_1 < -max_speed_) target_speed_1 = -max_speed_;
    
    ROS_INFO("Target speed set to: %.2f", target_speed_1);
}
void MotorController::timerCallback(const ros::TimerEvent& event) {
    // 버튼 상태 확인
    checkButtons();
    
    // 모터 제어
    //updateActualSpeed();
    actual_speed_0 = encoder.getWheel0Velocity() * 60 / 3.1415 / 2;
    actual_speed_1 = encoder.getWheel1Velocity() * 60 / 3.1415 / 2;
    setMotor0Direction(target_speed_0 >= 0);
    setMotor0Speed(std::abs(target_speed_0));
    setMotor1Direction(target_speed_1 >= 0);
    setMotor1Speed(std::abs(target_speed_1));
    
    // 현재 속도 발행
    std_msgs::Float32 speed0_msg,speed1_msg;
    speed0_msg.data = actual_speed_0;
    speed1_msg.data = actual_speed_1;
    actual_speed0_pub_.publish(speed0_msg);
    actual_speed1_pub_.publish(speed1_msg);

    double sensors[7];
    i2c.getData(sensors,sensors+6);

    ROS_INFO("x:(%.2f %.2f) | v:(Target,Actual)= [(%.2f,  %.2f),(%.2f,  %.2f)], (%+.2f,  %+.2f,  %+.2f),(%+.2f,  %+.2f,  %+.2f),%.2f"
        , encoder.getWheel0Position()/3.1415/2, encoder.getWheel1Position()/3.1415/2
        , target_speed_0, actual_speed_0, target_speed_1, actual_speed_1
        , sensors[0], sensors[1], sensors[2]
        , sensors[3], sensors[4], sensors[5]
        , sensors[6]);
}
void MotorController::checkHallSensor(const ros::TimerEvent& event) {
    gpiod_line_set_value(latch_line, 1);
    uint8_t data = spi.transfer(0x00);
    gpiod_line_set_value(latch_line, 0);

    hall_state_0 = (data >> 1) & 0x7;
    hall_state_1 = (data >> 5) & 0x7;

    encoder.updateHallSensors(hall_state_0,hall_state_1);
    // 3바퀴에 10
    /*if (!hall_sensor_line_) return;
    

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
    
    prev_hall_state_ = hall_state;*/
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
        target_speed_0 += 5.0;
        target_speed_1 = target_speed_0;
        if (target_speed_0 > max_speed_) target_speed_0 = max_speed_;
        ROS_INFO("Speed increased: %.2f", target_speed_0);
        last_button_time_ = now;
    }
    
    // 감소 버튼 상승 엣지 감지
    if (dec_button_state && !prev_dec_button_state_) {
        target_speed_0 -= 5.0;
        target_speed_1 = target_speed_0;
        if (target_speed_0 < -max_speed_) target_speed_0 = -max_speed_;
        ROS_INFO("Speed decreased: %.2f", target_speed_0);
        last_button_time_ = now;
    }
    
    prev_inc_button_state_ = inc_button_state;
    prev_dec_button_state_ = dec_button_state;
}
void MotorController::updateActualSpeed() {
}
void MotorController::setMotor0Direction(bool forward) {
    if (!reverse0_line_) return;
    
    // gpiod를 사용하여 리버스 핀 제어
    gpiod_line_set_value(reverse0_line_, forward ? 1 : 0);
}
void MotorController::setMotor1Direction(bool forward) {
    if (!reverse1_line_) return;
    
    // gpiod를 사용하여 리버스 핀 제어
    gpiod_line_set_value(reverse1_line_, forward ? 1 : 0);
}
void MotorController::setMotor0Speed(float speed) {
    if (!dac0_) return;
    
    // 속도 범위 제한 및 퍼센트 변환
    float speed_percent = (speed / max_speed_) * 100.0;
    if (speed_percent < 0.0f) speed_percent = 0.0f;
    if (speed_percent > 100.0f) speed_percent = 100.0f;
    
    // DAC 출력 설정
    dac0_->setOutputPercent(speed_percent);
}
void MotorController::setMotor1Speed(float speed) {
    if (!dac1_) return;
    
    // 속도 범위 제한 및 퍼센트 변환
    float speed_percent = (speed / max_speed_) * 100.0;
    if (speed_percent < 0.0f) speed_percent = 0.0f;
    if (speed_percent > 100.0f) speed_percent = 100.0f;
    
    // DAC 출력 설정
    dac1_->setOutputPercent(speed_percent);
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
