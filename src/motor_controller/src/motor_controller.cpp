// src/motor_controller.cpp
#include "motor_controller/motor_controller.h"
#include <chrono>
#include <iostream>
#include <cmath>
#include <unistd.h>

// PID 컨트롤러 구현
PIDController::PIDController(double kp, double ki, double kd, double min_output, double max_output)
    : kp_(2), ki_(2), kd_(2), 
      error_(0.0), prev_error_(0.0), integral_(0.0), derivative_(0.0), output_(0.0),
      min_output_(min_output), max_output_(max_output), 
      integral_max_(std::abs(max_output) * 0.8), // 적분 제한을 최대 출력의 80%로 설정
      first_run_(true) {
}

void PIDController::setGains(double kp, double ki, double kd) {
    kp_ = kp;
    ki_ = ki;
    kd_ = kd;
}

void PIDController::setOutputLimits(double min_output, double max_output) {
    min_output_ = min_output;
    max_output_ = max_output;
    integral_max_ = std::abs(max_output) * 0.8;
}

void PIDController::reset() {
    error_ = 0.0;
    prev_error_ = 0.0;
    integral_ = 0.0;
    derivative_ = 0.0;
    output_ = 0.0;
    first_run_ = true;
}

double PIDController::compute(double setpoint, double measured_value, double gyro_x, double dt) {
    if (dt <= 0.0) return output_;
    
    // 오차 계산
    error_ = setpoint - measured_value;
    
    // 적분 계산 (와인드업 방지)
    integral_ += error_ * dt;
    if (integral_ > integral_max_) integral_ = integral_max_;
    if (integral_ < -integral_max_) integral_ = -integral_max_;
    
    // 미분 계산 (첫 실행 시 미분은 0)
    /*if (first_run_) {
        derivative_ = 0.0;
        first_run_ = false;
    } else {
        derivative_ = (error_ - prev_error_) / dt;
    }*/
    derivative_ = -gyro_x;

    // PID 출력 계산
    output_ = kp_ * error_ + ki_ * integral_ + kd_ * derivative_;
    
    // 출력 제한
    if (output_ > max_output_) output_ = max_output_;
    if (output_ < min_output_) output_ = min_output_;
    
    // 다음 계산을 위해 현재 오차 저장
    prev_error_ = error_;
    
    return output_;
}

MotorController::MotorController(ros::NodeHandle& nh, ros::NodeHandle& pnh)
    : nh_(nh), pnh_(pnh), dac0_(nullptr), dac1_(nullptr), chip_(nullptr), reverse0_line_(nullptr), 
      reverse1_line_(nullptr), inc_button_line_(nullptr), dec_button_line_(nullptr),
      target_speed_0(0.0), actual_speed_0(0.0), target_speed_1(0.0), actual_speed_1(0.0),
      prev_hall_state_(false), hall_count_(0), prev_inc_button_state_(false), prev_dec_button_state_(false),
      encoder(WHEEL_RADIUS,WHEEL_DISTANCE,999),
      max_speed_(100.0),
      pid_motor0_(2.0, 0.5, 0.1, -100.0, 100.0),  // 기본 PID 게인
      pid_motor1_(2.0, 0.5, 0.1, -100.0, 100.0),
      pid_balance_(2.0, 2.0, 2.0, -100.0, 100.0),  // 밸런싱 PID 게인
      pid_output_0_(0.0), pid_output_1_(0.0), balance_output_(0.0),
      target_roll_(0.0), current_roll_(0.0),
      actual_motor_output_0_(0.0), actual_motor_output_1_(0.0),
      motor_deadzone_(5.0), deadzone_compensation_(8.0),  // 데드존 설정
      brake_zone_threshold_(10.0), brake_duration_(0.1),  // 브레이크 존 설정
      prev_output_0_(0.0), prev_output_1_(0.0),
      is_braking_0_(false), is_braking_1_(false),
      hall_state_0(0), hall_state_1(0)
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
    
    // PID 게인 파라미터 읽기
    double kp0, ki0, kd0, kp1, ki1, kd1;
    double kp_bal, ki_bal, kd_bal;
    pnh_.param("pid_kp_motor0", kp0, 2.0);
    pnh_.param("pid_ki_motor0", ki0, 0.0);
    pnh_.param("pid_kd_motor0", kd0, 0.0);
    pnh_.param("pid_kp_motor1", kp1, 2.0);
    pnh_.param("pid_ki_motor1", ki1, 0.0);
    pnh_.param("pid_kd_motor1", kd1, 0.1);
    
    // 밸런싱 PID 게인 파라미터
    pnh_.param("pid_kp_balance", kp_bal, 2.0);
    pnh_.param("pid_ki_balance", ki_bal, 2.0);
    pnh_.param("pid_kd_balance", kd_bal, 2.0);
    
    // PID 제어 모드 파라미터
    pnh_.param("target_roll_angle", target_roll_, 0.0);
    
    // 데드존 보상 파라미터
    pnh_.param("motor_deadzone", motor_deadzone_, 5.0);
    pnh_.param("deadzone_compensation", deadzone_compensation_, 8.0);
    
    // 방향 전환 제어 파라미터
    pnh_.param("brake_zone_threshold", brake_zone_threshold_, 10.0);
    pnh_.param("brake_duration", brake_duration_, 0.1);
    
    ROS_INFO("kp0: %f, ki0: %f, kd0: %f", kp0, ki0, kd0);
    ROS_INFO("kp1: %f, ki1: %f, kd1: %f", kp1, ki1, kd1);
    ROS_INFO("kp_bal: %f, ki_bal: %f, kd_bal: %f", kp_bal, ki_bal, kd_bal);
    ROS_INFO("target_roll_angle: %f", target_roll_);
    ROS_INFO("motor_deadzone: %f, deadzone_compensation: %f", motor_deadzone_, deadzone_compensation_);
    ROS_INFO("brake_zone_threshold: %f, brake_duration: %f", brake_zone_threshold_, brake_duration_);

    // PID 게인 설정
    pid_balance_.setGains(kp_bal, ki_bal, kd_bal);

    ROS_INFO("Motor0 PID Gains - Kp: %.2f, Ki: %.2f, Kd: %.2f", kp0, ki0, kd0);
    ROS_INFO("Motor1 PID Gains - Kp: %.2f, Ki: %.2f, Kd: %.2f", kp1, ki1, kd1);
    
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
    //speed0_sub_ = nh_.subscribe("target_speed0", 10, &MotorController::speed0Callback, this);
    //actual_speed0_pub_ = nh_.advertise<std_msgs::Float32>("actual_speed0", 10);
    
    //speed1_sub_ = nh_.subscribe("target_speed1", 10, &MotorController::speed1Callback, this);
    //actual_speed1_pub_ = nh_.advertise<std_msgs::Float32>("actual_speed1", 10);
    
    // 센서 데이터 발행자 (테스트용)
    aroll_pub_ = nh_.advertise<std_msgs::Float32>("aroll", 10);
    groll_pub_ = nh_.advertise<std_msgs::Float32>("groll", 10);

    target_roll_pub_ = nh_.advertise<std_msgs::Float32>("target_roll", 10);
    actual_roll_pub_ = nh_.advertise<std_msgs::Float32>("actual_roll", 10);
    balance_output_pub_ = nh_.advertise<std_msgs::Float32>("balance_output", 10);
    balance_P_pub_ = nh_.advertise<std_msgs::Float32>("balance_P", 10);
    balance_I_pub_ = nh_.advertise<std_msgs::Float32>("balance_I", 10);
    balance_D_pub_ = nh_.advertise<std_msgs::Float32>("balance_D", 10);

    
    // 타이머 설정
    timer0_ = nh_.createTimer(ros::Duration(0.005), &MotorController::timerCallback, this);
    
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
void MotorController::setupGPIO(){
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
    
    // 실제 속도 업데이트
    actual_speed_0 = encoder.getWheel0Velocity() * 60 / 3.1415 / 2;
    actual_speed_1 = encoder.getWheel1Velocity() * 60 / 3.1415 / 2;
    
    static double sensors[13]; // 실패 시 이전 연산값 활용
    bool result = i2c.getData(sensors,sensors+11);

    // 지면과의 각도 계산 및 출력
    double roll_angle = sensors[6];           // 즉시 응답 각도
    double pitch_angle = sensors[7];
    double yaw_angle = sensors[8];

    // 상보 필터를 출력으로 사용
    current_roll_ = sensors[6];

    // 밸런싱 제어: Roll 각도 기반
    updateBalanceControl(sensors[3]); // gyro_x를 PID의 Derivative 입력으로 사용
    applyBalanceOutput();
    
    /*ROS_INFO("sangbo(%4.1f) | acc_angle: %4.1f | target/actual : %4.1f/%4.1f | PID Out: [%.1f] | Motor Out: [%.1f] | Err: [%.1f]"
        , (float)sensors[6]
        ,(float)roll_angle
        ,(float)target_roll_
        ,(float)current_roll_
        ,(float)balance_output_
        ,(float)actual_motor_output_0_
        ,(float)pid_balance_.getError());*/
    static int count = 0;
    count++;
    if (count % 10 == 0) {
        ROS_INFO("target roll : %5.1f / current roll : %5.1f / balance output : %5.1f / arg_PID : %5.1f %5.1f %5.1f"
            ,(float)target_roll_
            ,(float)current_roll_
            ,(float)balance_output_
            ,(float)pid_balance_.getError()*pid_balance_.kp_
            ,(float)pid_balance_.getIntegral()*pid_balance_.ki_
            ,(float)pid_balance_.getDerivative()*pid_balance_.kd_
            );
    }
    std_msgs::Float32 aroll_msg;
    std_msgs::Float32 groll_msg;

    std_msgs::Float32 target_roll_msg;
    std_msgs::Float32 actual_roll_msg;
    std_msgs::Float32 balance_output_msg;
    std_msgs::Float32 balance_P_msg;
    std_msgs::Float32 balance_I_msg;
    std_msgs::Float32 balance_D_msg;

    aroll_msg.data = sensors[3];
    groll_msg.data = i2c.getRoll();
    target_roll_msg.data = target_roll_;
    actual_roll_msg.data = current_roll_;
    balance_output_msg.data = balance_output_;
    balance_P_msg.data = (float)pid_balance_.getError()*pid_balance_.kp_;
    balance_I_msg.data = pid_balance_.getIntegral()*pid_balance_.ki_;
    balance_D_msg.data = pid_balance_.getDerivative()*pid_balance_.kd_;

    aroll_pub_.publish(aroll_msg);
    groll_pub_.publish(groll_msg);
    target_roll_pub_.publish(target_roll_msg);
    actual_roll_pub_.publish(actual_roll_msg);
    balance_output_pub_.publish(balance_output_msg);
    balance_P_pub_.publish(balance_P_msg);
    balance_I_pub_.publish(balance_I_msg);
    balance_D_pub_.publish(balance_D_msg);
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
        target_speed_0 += 0.5;
        target_speed_1 = target_speed_0;
        if (target_speed_0 > max_speed_) target_speed_0 = max_speed_;
        ROS_INFO("Speed increased: %.2f", target_speed_0);
        last_button_time_ = now;
    }
    
    // 감소 버튼 상승 엣지 감지
    if (dec_button_state && !prev_dec_button_state_) {
        target_speed_0 -= 0.5;
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

void MotorController::updateBalanceControl(double gyro_x) {
    ros::Time current_time = ros::Time::now();
    double dt = (current_time - last_pid_time_).toSec();
    
    // 유효한 시간 간격인지 확인 (1ms ~ 1s)
    if (dt < 0.0001 || dt > 1.0) {
        last_pid_time_ = current_time;
        return;
    }
    // 밸런싱 PID 계산: 목표 Roll(0.0) vs 현재 Roll
    balance_output_ = pid_balance_.compute(target_roll_, current_roll_, gyro_x, dt);
    
    last_pid_time_ = current_time;
}

void MotorController::applyBalanceOutput() {
    // 방향 전환 제어 적용 (양쪽 모터 동일한 출력)
    double controlled_output = balance_output_;

    double motor_speed = std::abs(controlled_output);
    bool forward = (controlled_output < 0);
    
    // 실제 모터 출력값 저장 (로그용) - 양쪽 모터 동일
    actual_motor_output_0_ = forward ? motor_speed : -motor_speed;
    actual_motor_output_1_ = forward ? motor_speed : -motor_speed;
    
    // 양쪽 모터 동일하게 제어 (밸런싱)
    setMotor0Speed(motor_speed);
    setMotor0Direction(forward);
    setMotor1Speed(motor_speed);
    setMotor1Direction(forward);
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
