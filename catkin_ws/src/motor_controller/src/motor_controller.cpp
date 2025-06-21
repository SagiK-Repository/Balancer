// src/motor_controller.cpp
#include "motor_controller/motor_controller.h"
#include <chrono>
#include <iostream>
#include <cmath>
#include <unistd.h>

// PID 컨트롤러 구현
PIDController::PIDController(double kp, double ki, double kd, double min_output, double max_output)
    : kp_(kp), ki_(ki), kd_(kd), 
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

double PIDController::compute(double setpoint, double measured_value, double dt) {
    if (dt <= 0.0) return output_;
    
    // 오차 계산
    error_ = setpoint - measured_value;
    
    // 적분 계산 (와인드업 방지)
    integral_ += error_ * dt;
    if (integral_ > integral_max_) integral_ = integral_max_;
    if (integral_ < -integral_max_) integral_ = -integral_max_;
    
    // 미분 계산 (첫 실행 시 미분은 0)
    if (first_run_) {
        derivative_ = 0.0;
        first_run_ = false;
    } else {
        derivative_ = (error_ - prev_error_) / dt;
    }
    
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
      pid_balance_(15.0, 2.0, 0.5, -100.0, 100.0),  // 밸런싱 PID 게인
      pid_output_0_(0.0), pid_output_1_(0.0), balance_output_(0.0),
      target_roll_(0.0), current_roll_(0.0),
      use_balance_control_(true),  // 기본적으로 밸런싱 제어 사용
      actual_motor_output_0_(0.0), actual_motor_output_1_(0.0),
      motor_deadzone_(5.0), deadzone_compensation_(8.0),  // 데드존 설정
      brake_zone_threshold_(10.0), brake_duration_(0.1),  // 브레이크 존 설정
      prev_output_0_(0.0), prev_output_1_(0.0),
      is_braking_0_(false), is_braking_1_(false),
      use_pid_control_(true),  // 기본적으로 PID 제어 사용
      use_smooth_control_(true),  // 기본적으로 부드러운 제어 사용
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
    pnh_.param("pid_kp_balance", kp_bal, 15.0);
    pnh_.param("pid_ki_balance", ki_bal, 2.0);
    pnh_.param("pid_kd_balance", kd_bal, 0.5);
    
    // PID 제어 모드 파라미터
    pnh_.param("use_pid_control", use_pid_control_, true);
    pnh_.param("use_balance_control", use_balance_control_, true);
    pnh_.param("target_roll_angle", target_roll_, 0.0);
    pnh_.param("use_smooth_control", use_smooth_control_, true);
    
    // 데드존 보상 파라미터
    pnh_.param("motor_deadzone", motor_deadzone_, 5.0);
    pnh_.param("deadzone_compensation", deadzone_compensation_, 8.0);
    
    // 방향 전환 제어 파라미터
    pnh_.param("brake_zone_threshold", brake_zone_threshold_, 10.0);
    pnh_.param("brake_duration", brake_duration_, 0.1);
    
    // PID 게인 설정
    pid_motor0_.setGains(kp0, ki0, kd0);
    pid_motor1_.setGains(kp1, ki1, kd1);
    pid_balance_.setGains(kp_bal, ki_bal, kd_bal);
    pid_motor0_.setOutputLimits(-max_speed_, max_speed_);
    pid_motor1_.setOutputLimits(-max_speed_, max_speed_);

    ROS_INFO("PID Control Mode: %s", use_pid_control_ ? "ENABLED" : "DISABLED");
    ROS_INFO("Smooth Control Mode: %s", use_smooth_control_ ? "ENABLED" : "DISABLED");
    ROS_INFO("Motor0 PID Gains - Kp: %.2f, Ki: %.2f, Kd: %.2f", kp0, ki0, kd0);
    ROS_INFO("Motor1 PID Gains - Kp: %.2f, Ki: %.2f, Kd: %.2f", kp1, ki1, kd1);
    
    // 부드러운 제어 매개변수 설정
    if (use_smooth_control_) {
        double max_accel, max_decel, vel_filter, out_filter;
        pnh_.param("smooth_max_acceleration", max_accel, 30.0);    // 30%/초
        pnh_.param("smooth_max_deceleration", max_decel, 60.0);    // 60%/초  
        pnh_.param("smooth_velocity_filter", vel_filter, 0.4);     // 속도 필터
        pnh_.param("smooth_output_filter", out_filter, 0.6);       // 출력 필터
        
        smooth_motor0_.setParameters(max_accel, max_decel, vel_filter, out_filter);
        smooth_motor1_.setParameters(max_accel, max_decel, vel_filter, out_filter);
        
        ROS_INFO("Smooth Control Parameters:");
        ROS_INFO("  Max Accel/Decel: %.1f/%.1f %%/s", max_accel, max_decel);
        ROS_INFO("  Filter coefficients: vel=%.2f, out=%.2f", vel_filter, out_filter);
    }
    
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
    
    // 센서 데이터 발행자 (테스트용)
    sensor_roll_pub_ = nh_.advertise<std_msgs::Float32>("sensor_roll", 10);
    sensor_pitch_pub_ = nh_.advertise<std_msgs::Float32>("sensor_pitch", 10);
    sensor_confidence_pub_ = nh_.advertise<std_msgs::Float32>("sensor_confidence", 10);
    
    // 타이머 설정
    timer0_ = nh_.createTimer(ros::Duration(0.002), &MotorController::timerCallback, this);
    
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
    
    // 실제 속도 업데이트
    actual_speed_0 = encoder.getWheel0Velocity() * 60 / 3.1415 / 2;
    actual_speed_1 = encoder.getWheel1Velocity() * 60 / 3.1415 / 2;
    
    // 현재 속도 발행
    std_msgs::Float32 speed0_msg, speed1_msg;
    speed0_msg.data = actual_speed_0;
    speed1_msg.data = actual_speed_1;
    actual_speed0_pub_.publish(speed0_msg);
    actual_speed1_pub_.publish(speed1_msg);
    
    double sensors[7];
    i2c.getData(sensors,sensors+6);

    // 지면과의 각도 계산 및 출력
    double roll_angle = i2c.getRoll();           // 즉시 응답 각도
    double pitch_angle = i2c.getPitch();
    double yaw_angle = i2c.getYaw();
    
    // 최적화된 안정화 각도
    double stable_roll = i2c.getStableRoll();    // 안정화된 각도
    double stable_pitch = i2c.getStablePitch();
    bool is_stable = i2c.isStable();
    
    // 센서 데이터 발행 (테스트용)
    std_msgs::Float32 roll_msg, pitch_msg, confidence_msg;
    roll_msg.data = is_stable ? stable_roll : roll_angle;
    pitch_msg.data = is_stable ? stable_pitch : pitch_angle;
    confidence_msg.data = i2c.getTiltConfidence();
    
    sensor_roll_pub_.publish(roll_msg);
    sensor_pitch_pub_.publish(pitch_msg);
    sensor_confidence_pub_.publish(confidence_msg);
    
    // 현재 Roll 각도 업데이트 (IMU 데이터 읽은 후)
    current_roll_ = is_stable ? stable_roll : roll_angle;
    
    // 제어 모드에 따른 처리
    if (use_pid_control_) {
        if (use_balance_control_) {
            // 밸런싱 제어: Roll 각도 기반
            updateBalanceControl();
            applyBalanceOutput();
        } else {
            // 속도 제어: 기존 방식
            updatePIDControl();
            applyPIDOutput();
        }
    } else {
        // 직접 제어 (부드러운 제어 적용)
        double smooth_output_0 = target_speed_0;
        double smooth_output_1 = target_speed_1;
        
        // 부드러운 제어 적용
        if (use_smooth_control_) {
            smooth_output_0 = applySmoothControl(target_speed_0, 0);
            smooth_output_1 = applySmoothControl(target_speed_1, 1);
        }
        
        actual_motor_output_0_ = smooth_output_0;
        actual_motor_output_1_ = smooth_output_1;
        
        setMotor0Direction(smooth_output_0 >= 0);
        setMotor0Speed(std::abs(smooth_output_0));
        setMotor1Direction(smooth_output_1 >= 0);
        setMotor1Speed(std::abs(smooth_output_1));
    }
    
    // 신뢰도 정보 가져오기
    double tilt_confidence = i2c.getTiltConfidence();
    
    // 안정 상태에 따라 다른 각도 사용
    double display_roll = is_stable ? stable_roll : roll_angle;
    double display_pitch = is_stable ? stable_pitch : pitch_angle;
    
    if (use_pid_control_) {
        if (use_balance_control_) {
            ROS_INFO("Tilt R/P: %.1f/%.1f deg %s(%.0f%%) | Target/Current Roll: %.1f/%.1f | Balance Out: %.1f | Motor Out: [%.1f,%.1f] | Err: %.1f | Temp: %.1f C",
                display_roll, display_pitch, 
                is_stable ? "[STABLE] " : "[MOVING] ",
                tilt_confidence,
                target_roll_, current_roll_, balance_output_,
                actual_motor_output_0_, actual_motor_output_1_,
                pid_balance_.getError(),
                sensors[6]);
        } else {
            ROS_INFO("Tilt R/P: %.1f/%.1f deg %s(%.0f%%) | Speed T/A: [%.1f,%.1f]/[%.1f,%.1f] | PID Out: [%.1f,%.1f] | Motor Out: [%.1f,%.1f] | Err: [%.1f,%.1f] | Temp: %.1f C",
                display_roll, display_pitch, 
                is_stable ? "[STABLE] " : "[MOVING] ",
                tilt_confidence,
                target_speed_0, target_speed_1, actual_speed_0, actual_speed_1,
                pid_output_0_, pid_output_1_,
                actual_motor_output_0_, actual_motor_output_1_,
                pid_motor0_.getError(), pid_motor1_.getError(),
                sensors[6]);
        }
    } else {
        ROS_INFO("Tilt R/P: %.1f/%.1f deg %s(%.0f%%) | Speed T/A: [%.1f,%.1f]/[%.1f,%.1f] | RAW Mode | Motor Out: [%.1f,%.1f] | Temp: %.1f C",
            display_roll, display_pitch, 
            is_stable ? "[STABLE] " : "[MOVING] ",
            tilt_confidence,
            target_speed_0, target_speed_1, actual_speed_0, actual_speed_1,
            actual_motor_output_0_, actual_motor_output_1_,
            sensors[6]);
    }
    
    // 기울기 경고 출력 (안정화된 각도 기준)
    if (is_stable && tilt_confidence > 70.0) {  // 안정 상태이고 신뢰도 높을 때만 경고
        if (std::abs(display_roll) > 30.0 || std::abs(display_pitch) > 30.0) {
            ROS_ERROR("DANGER! Severe tilt R/P: %.1f/%.1f deg", display_roll, display_pitch);
        } else if (std::abs(display_roll) > 15.0 || std::abs(display_pitch) > 15.0) {
            ROS_WARN("Tilt warning R/P: %.1f/%.1f deg", display_roll, display_pitch);
        }
    }
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
void MotorController::updatePIDControl() {
    ros::Time current_time = ros::Time::now();
    double dt = (current_time - last_pid_time_).toSec();
    
    // 유효한 시간 간격인지 확인 (1ms ~ 1s)
    if (dt < 0.001 || dt > 1.0) {
        last_pid_time_ = current_time;
        return;
    }
    
    // PID 계산
    pid_output_0_ = pid_motor0_.compute(target_speed_0, actual_speed_0, dt);
    pid_output_1_ = pid_motor1_.compute(target_speed_1, actual_speed_1, dt);
    
    last_pid_time_ = current_time;
}
void MotorController::applyPIDOutput() {
    // 방향 전환 제어 적용
    double controlled_output_0 = applyDirectionChangeControl(pid_output_0_, 0);
    double controlled_output_1 = applyDirectionChangeControl(pid_output_1_, 1);
    
    // 부드러운 제어 적용
    if (use_smooth_control_) {
        controlled_output_0 = applySmoothControl(controlled_output_0, 0);
        controlled_output_1 = applySmoothControl(controlled_output_1, 1);
    }
    
    // 데드존 보상 적용
    double compensated_speed_0 = compensateDeadzone(std::abs(controlled_output_0));
    double compensated_speed_1 = compensateDeadzone(std::abs(controlled_output_1));
    
    // 실제 모터 출력값 저장 (로그용)
    actual_motor_output_0_ = (controlled_output_0 >= 0) ? compensated_speed_0 : -compensated_speed_0;
    actual_motor_output_1_ = (controlled_output_1 >= 0) ? compensated_speed_1 : -compensated_speed_1;
    
    // 모터 0 제어
    setMotor0Direction(controlled_output_0 >= 0);
    setMotor0Speed(compensated_speed_0);
    
    // 모터 1 제어
    setMotor1Direction(controlled_output_1 >= 0);
    setMotor1Speed(compensated_speed_1);
}
void MotorController::resetPIDControllers() {
    pid_motor0_.reset();
    pid_motor1_.reset();
    pid_balance_.reset();
    pid_output_0_ = 0.0;
    pid_output_1_ = 0.0;
    balance_output_ = 0.0;
    last_pid_time_ = ros::Time::now();
    
    ROS_INFO("PID controllers reset");
}

void MotorController::updateBalanceControl() {
    ros::Time current_time = ros::Time::now();
    double dt = (current_time - last_pid_time_).toSec();
    
    // 유효한 시간 간격인지 확인 (1ms ~ 1s)
    if (dt < 0.001 || dt > 1.0) {
        last_pid_time_ = current_time;
        return;
    }
    
    // 밸런싱 PID 계산: 목표 Roll(0.0) vs 현재 Roll
    balance_output_ = pid_balance_.compute(target_roll_, current_roll_, dt);
    
    last_pid_time_ = current_time;
}

void MotorController::applyBalanceOutput() {
    // 방향 전환 제어 적용 (양쪽 모터 동일한 출력)
    double controlled_output = applyDirectionChangeControl(balance_output_, 0);
    
    // 부드러운 제어 적용 (밸런싱 시에는 양쪽 모터 동일)
    if (use_smooth_control_) {
        controlled_output = applySmoothControl(controlled_output, 0);
    }
    
    // 밸런싱 출력을 양쪽 모터에 동일하게 적용
    double motor_speed = std::abs(controlled_output);
    bool forward = (controlled_output < 0);
    
    // 데드존 보상 적용
    motor_speed = compensateDeadzone(motor_speed);
    
    // 실제 모터 출력값 저장 (로그용) - 양쪽 모터 동일
    actual_motor_output_0_ = forward ? motor_speed : -motor_speed;
    actual_motor_output_1_ = forward ? motor_speed : -motor_speed;
    
    // 양쪽 모터 동일하게 제어 (밸런싱)
    setMotor0Speed(motor_speed);
    setMotor0Direction(forward);
    setMotor1Speed(motor_speed);
    setMotor1Direction(forward);
}

double MotorController::compensateDeadzone(double speed) {
    if (speed == 0.0) {
        return 0.0;  // 완전 정지는 그대로
    }
    
    // 작은 속도값에 대해 데드존 보상 적용
    if (std::abs(speed) < motor_deadzone_) {
        // 데드존 이하의 속도는 최소 동작 속도로 보상
        return (speed > 0) ? deadzone_compensation_ : -deadzone_compensation_;
    } else {
        // 데드존 이상의 속도는 선형 스케일링
        double sign = (speed > 0) ? 1.0 : -1.0;
        double abs_speed = std::abs(speed);
        
        // 데드존을 고려한 선형 매핑
        // 입력: motor_deadzone_ ~ 100%
        // 출력: deadzone_compensation_ ~ 100%
        double scaled_speed = deadzone_compensation_ + 
                             (abs_speed - motor_deadzone_) * 
                             (100.0 - deadzone_compensation_) / 
                             (100.0 - motor_deadzone_);
        
        return sign * std::min(scaled_speed, 100.0);
    }
}

double MotorController::applySmoothControl(double target_output, int motor_id) {
    ros::Time current_time = ros::Time::now();
    static ros::Time last_smooth_time0 = current_time;
    static ros::Time last_smooth_time1 = current_time;
    
    double dt;
    double smooth_output;
    
    if (motor_id == 0) {
        dt = (current_time - last_smooth_time0).toSec();
        
        // 유효한 시간 간격인지 확인
        if (dt < 0.001 || dt > 1.0) {
            last_smooth_time0 = current_time;
            return target_output;
        }
        
        smooth_motor0_.setTargetVelocity(target_output);
        smooth_output = smooth_motor0_.update(dt);
        last_smooth_time0 = current_time;
    } else {
        dt = (current_time - last_smooth_time1).toSec();
        
        // 유효한 시간 간격인지 확인
        if (dt < 0.001 || dt > 1.0) {
            last_smooth_time1 = current_time;
            return target_output;
        }
        
        smooth_motor1_.setTargetVelocity(target_output);
        smooth_output = smooth_motor1_.update(dt);
        last_smooth_time1 = current_time;
    }
    
    return smooth_output;
}

bool MotorController::isDirectionChange(double current_output, double prev_output) {
    // 부호가 다르고, 둘 다 브레이크 존 임계값 이하인 경우
    return (current_output * prev_output < 0) && 
           (std::abs(current_output) < brake_zone_threshold_) && 
           (std::abs(prev_output) < brake_zone_threshold_);
}

double MotorController::applyDirectionChangeControl(double output, int motor_id) {
    ros::Time current_time = ros::Time::now();
    double prev_output = (motor_id == 0) ? prev_output_0_ : prev_output_1_;
    bool& is_braking = (motor_id == 0) ? is_braking_0_ : is_braking_1_;
    ros::Time& brake_start_time = (motor_id == 0) ? brake_start_time_0_ : brake_start_time_1_;
    
    // 방향 전환 감지
    if (!is_braking && isDirectionChange(output, prev_output)) {
        // 브레이킹 시작
        is_braking = true;
        brake_start_time = current_time;
        /*ROS_INFO("Motor %d: Direction change detected, starting brake (%.1f -> %.1f)", 
                 motor_id, prev_output, output);*/
        
        // 이전 출력값 업데이트
        if (motor_id == 0) prev_output_0_ = output;
        else prev_output_1_ = output;
        
        return 0.0;  // 브레이크 시작 - 정지
    }
    
    // 브레이킹 중인 경우
    if (is_braking) {
        double elapsed = (current_time - brake_start_time).toSec();
        
        if (elapsed < brake_duration_) {
            // 브레이킹 지속
            return 0.0;
        } else {
            // 브레이킹 종료
            is_braking = false;
            //ROS_INFO("Motor %d: Brake finished, resuming with output %.1f", motor_id, output);
        }
    }
    
    // 이전 출력값 업데이트
    if (motor_id == 0) prev_output_0_ = output;
    else prev_output_1_ = output;
    
    return output;  // 정상 출력
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
