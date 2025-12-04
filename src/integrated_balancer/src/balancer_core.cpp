#include "integrated_balancer/balancer_core.h"
#include <unistd.h>
#include <sched.h>
#include <sys/mman.h>

BalancerCore::BalancerCore()
    : spi_(nullptr), dac0_(nullptr), dac1_(nullptr),
      pid_balance_(9.0, 2.0, 0.5, -100.0, 100.0),  // 기본 PID 게인
      running_(false),
      target_speed_(0.0f),
      target_angle_(-0.7f),  // 기본 목표 각도
      current_angle_(0.0f),
      current_gyro_rate_(0.0f),
      motor_output_(0.0f),
      stable_angle_(0.0f),
      prev_stable_angle_(0.0f),
      first_angle_calc_(true) {
    
    // 센서 보정값 초기화
    for (int i = 0; i < 3; i++) {
        accel_offset_[i] = 0.0;
        gyro_offset_[i] = 0.0;
    }
}

BalancerCore::~BalancerCore() {
    stop();
    
    if (dac0_) {
        dac0_->setOutput(0);
        delete dac0_;
    }
    if (dac1_) {
        dac1_->setOutput(0);
        delete dac1_;
    }
    if (spi_) {
        delete spi_;
    }
}

bool BalancerCore::initialize() {
    // I2C 초기화
    if (!i2c_controller_.initI2C()) {
        return false;
    }
    
    if (!i2c_controller_.initMPU6050()) {
        return false;
    }
    
    // 센서 보정
    i2c_controller_.calibrateSensor();
    accel_offset_[0] = i2c_controller_.getAccelOffsetX();
    accel_offset_[1] = i2c_controller_.getAccelOffsetY();
    accel_offset_[2] = i2c_controller_.getAccelOffsetZ();
    gyro_offset_[0] = i2c_controller_.getGyroOffsetX();
    gyro_offset_[1] = i2c_controller_.getGyroOffsetY();
    gyro_offset_[2] = i2c_controller_.getGyroOffsetZ();
    
    // SPI 초기화 (기본 경로는 Jetson Nano에 맞게 조정 필요)
    spi_ = new HardwareSPI("/dev/spidev0.0", 1000000, SPI_MODE_0, 8);
    if (!spi_->initialize()) {
        // SPI 초기화 실패 시 nullptr로 설정 (더미 모드)
        delete spi_;
        spi_ = nullptr;
    }
    
    // DAC 초기화 (CS 핀 번호는 하드웨어에 맞게 조정 필요)
    if (spi_) {
        dac0_ = new MCP4921(spi_, 19);  // CS0 핀
        dac1_ = new MCP4921(spi_, 20);  // CS1 핀
    }
    
    return true;
}

void BalancerCore::start() {
    if (running_.load()) {
        return;
    }
    
    running_.store(true);
    control_thread_ = std::thread(&BalancerCore::controlLoop, this);
}

void BalancerCore::stop() {
    if (!running_.load()) {
        return;
    }
    
    running_.store(false);
    if (control_thread_.joinable()) {
        control_thread_.join();
    }
    
    // 모터 정지
    if (dac0_) {
        dac0_->setOutput(0);
    }
    if (dac1_) {
        dac1_->setOutput(0);
    }
}

void BalancerCore::controlLoop() {
    // Real-Time 우선순위 설정
    setRealTimePriority();
    
    // 고정 주기 루프 (10ms = 100Hz)
    struct timespec next_period;
    clock_gettime(CLOCK_MONOTONIC, &next_period);
    
    const long period_ns = 10000000;  // 10ms = 10,000,000 ns
    
    while (running_.load()) {
        // 1. 주기 계산 (10ms = 10,000,000 ns)
        next_period.tv_nsec += period_ns;
        if (next_period.tv_nsec >= 1000000000) {
            next_period.tv_nsec -= 1000000000;
            next_period.tv_sec++;
        }
        
        // 2. 제어 로직 실행 (센서 -> PID -> 모터)
        processControl();
        
        // 3. 남은 시간만큼 정확히 Sleep
        clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &next_period, NULL);
    }
}

void BalancerCore::processControl() {
    // 센서 읽기
    if (!readSensorData()) {
        return;
    }
    
    // 각도 계산
    calculateAngle();
    
    // PID 제어 계산 (고정 dt = 0.01s)
    double dt = 0.01;
    double target = target_angle_.load();
    
    std::lock_guard<std::mutex> lock(control_mutex_);
    float output = static_cast<float>(pid_balance_.compute(target, current_angle_, dt));
    
    // 모터 출력 설정
    setMotorOutput(output);
    motor_output_ = output;
}

bool BalancerCore::readSensorData() {
    SensorData raw_data;
    if (!i2c_controller_.readSensorData(raw_data)) {
        return false;
    }
    
    sensor_data_ = raw_data;
    
    // 데이터 변환 및 보정
    accel_x_ = i2c_controller_.convertAccelData(raw_data.accel_x - accel_offset_[0]);
    accel_y_ = i2c_controller_.convertAccelData(raw_data.accel_y - accel_offset_[1]);
    accel_z_ = i2c_controller_.convertAccelData(raw_data.accel_z - accel_offset_[2]);
    gyro_x_ = i2c_controller_.convertGyroData(raw_data.gyro_x - gyro_offset_[0]);
    gyro_y_ = i2c_controller_.convertGyroData(raw_data.gyro_y - gyro_offset_[1]);
    gyro_z_ = i2c_controller_.convertGyroData(raw_data.gyro_z - gyro_offset_[2]);
    
    return true;
}

void BalancerCore::calculateAngle() {
    // 가속도계에서 Roll 각도 계산 (도 단위)
    double accel_roll = atan2(accel_y_, accel_z_) * 180.0 / M_PI;
    
    // 즉시 응답 각도
    current_angle_ = static_cast<float>(accel_roll);
    
    // 상보 필터 적용 (안정화된 각도)
    const double complementary_alpha = 0.02;
    double dt = 0.01;
    
    if (first_angle_calc_) {
        stable_angle_ = accel_roll;
        prev_stable_angle_ = accel_roll;
        first_angle_calc_ = false;
    } else {
        // 상보 필터: angle = α * (이전각도 + 자이로적분) + (1-α) * 가속도각도
        stable_angle_ = complementary_alpha * (stable_angle_ + gyro_x_ * dt) + 
                        (1.0 - complementary_alpha) * accel_roll;
    }
    
    // 안정화된 각도 사용
    current_angle_ = static_cast<float>(stable_angle_);
    current_gyro_rate_ = static_cast<float>(gyro_x_);
    
    prev_stable_angle_ = stable_angle_;
}

void BalancerCore::setMotorOutput(float output) {
    // 출력 범위 제한 (-100 ~ 100%)
    if (output > 100.0f) output = 100.0f;
    if (output < -100.0f) output = -100.0f;
    
    // 절댓값으로 변환 (방향은 별도 GPIO로 제어)
    float abs_output = std::abs(output);
    
    // 두 모터에 동일한 출력 적용
    if (dac0_) {
        dac0_->setOutputPercent(abs_output);
    }
    if (dac1_) {
        dac1_->setOutputPercent(abs_output);
    }
}

bool BalancerCore::setRealTimePriority() {
    // 메모리 잠금 (옵션, Real-Time 성능 향상)
    if (mlockall(MCL_CURRENT | MCL_FUTURE) != 0) {
        // 실패해도 계속 진행
    }
    
    // 스레드 스케줄링 정책 설정
    struct sched_param param;
    param.sched_priority = 90;  // RT Priority 90
    
    if (pthread_setschedparam(pthread_self(), SCHED_FIFO, &param) != 0) {
        // 실패 시 일반 스레드로 실행
        return false;
    }
    
    return true;
}




