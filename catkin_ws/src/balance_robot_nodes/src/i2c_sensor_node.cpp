#include "balance_robot_nodes/i2c_sensor_node.h"
#include <ros/ros.h>
#include <std_msgs/Float32.h>
#include <cmath>

// I2CController 실제 구현
I2CController::I2CController() : 
    i2c_file(-1),
    i2c_device("/dev/i2c-1"),  // Jetson Nano의 기본 I2C 버스
    accel_offset_x(0), accel_offset_y(0), accel_offset_z(0),
    gyro_offset_x(0), gyro_offset_y(0), gyro_offset_z(0),
    accel_scale(8192.0),   // ±4g 설정 시 (16384/2)
    gyro_scale(65.5)       // ±500°/s 설정 시 (131/2)
{
    ROS_INFO("I2C Controller initialized");
}

I2CController::~I2CController() {
    closeI2C();
}

bool I2CController::initI2C() {
    // I2C 디바이스 파일 열기
    i2c_file = open(i2c_device.c_str(), O_RDWR);
    if (i2c_file < 0) {
        ROS_ERROR("Failed to open I2C device: %s", i2c_device.c_str());
        return false;
    }
    
    // I2C 슬레이브 주소 설정
    if (ioctl(i2c_file, I2C_SLAVE, MPU6050_ADDR) < 0) {
        ROS_ERROR("Failed to set I2C slave address");
        close(i2c_file);
        i2c_file = -1;
        return false;
    }
    
    ROS_INFO("I2C initialized successfully on %s", i2c_device.c_str());
    return true;
}

void I2CController::closeI2C() {
    if (i2c_file >= 0) {
        close(i2c_file);
        i2c_file = -1;
        ROS_INFO("I2C connection closed");
    }
}

bool I2CController::initMPU6050() {
    ROS_INFO("Initializing GY-521 (MPU6050) module...");
    
    // 1. MPU6050 리셋 및 깨우기
    if (!writeRegister(PWR_MGMT_1, 0x80)) {  // 디바이스 리셋
        ROS_ERROR("Failed to reset MPU6050");
        return false;
    }
    usleep(100000); // 100ms 대기 (리셋 완료)
    
    if (!writeRegister(PWR_MGMT_1, 0x01)) {  // X 자이로 클럭 사용, 슬립 해제
        ROS_ERROR("Failed to wake up MPU6050");
        return false;
    }
    usleep(50000); // 50ms 대기
    
    // 2. 샘플레이트 설정 (1kHz / (1 + SMPLRT_DIV))
    if (!writeRegister(SMPLRT_DIV, 0x09)) { // 100Hz (밸런싱에 최적)
        ROS_ERROR("Failed to set sample rate");
        return false;
    }
    
    // 3. 디지털 로우패스 필터 설정 (GY-521 최적화)
    if (!writeRegister(CONFIG, 0x03)) {  // 44Hz LPF (노이즈 감소)
        ROS_ERROR("Failed to set DLPF config");
        return false;
    }
    
    // 4. 자이로스코프 설정 (±500°/s - 밸런싱에 적합)
    if (!writeRegister(GYRO_CONFIG, 0x08)) {  // ±500°/s
        ROS_ERROR("Failed to set gyro config");
        return false;
    }
    
    // 5. 가속도계 설정 (±4g - 더 넓은 범위)
    if (!writeRegister(ACCEL_CONFIG, 0x08)) {  // ±4g
        ROS_ERROR("Failed to set accel config");
        return false;
    }
    
    usleep(100000); // 100ms 대기 (설정 안정화)
    
    ROS_INFO("GY-521 (MPU6050) initialized successfully");
    ROS_INFO("Configuration: 100Hz, ±4g accel, ±500°/s gyro, 44Hz LPF");
    return true;
}

bool I2CController::writeRegister(uint8_t reg, uint8_t value) {
    uint8_t buffer[2] = {reg, value};
    
    if (write(i2c_file, buffer, 2) != 2) {
        ROS_ERROR("Failed to write to register 0x%02X", reg);
        return false;
    }
    
    return true;
}

bool I2CController::readRegister(uint8_t reg, uint8_t* data, int length) {
    // 레지스터 주소 쓰기
    if (write(i2c_file, &reg, 1) != 1) {
        ROS_ERROR("Failed to write register address 0x%02X", reg);
        return false;
    }
    
    // 데이터 읽기
    if (read(i2c_file, data, length) != length) {
        ROS_ERROR("Failed to read from register 0x%02X", reg);
        return false;
    }
    
    return true;
}

bool I2CController::readSensorData(SensorData& data) {
    uint8_t buffer[14];
    
    // 가속도, 온도, 자이로 데이터를 한 번에 읽기
    if (!readRegister(ACCEL_XOUT_H, buffer, 14)) {
        return false;
    }
    
    // 16비트 데이터 조합
    data.accel_x = (buffer[0] << 8) | buffer[1];
    data.accel_y = (buffer[2] << 8) | buffer[3];
    data.accel_z = (buffer[4] << 8) | buffer[5];
    data.temp = (buffer[6] << 8) | buffer[7];
    data.gyro_x = (buffer[8] << 8) | buffer[9];
    data.gyro_y = (buffer[10] << 8) | buffer[11];
    data.gyro_z = (buffer[12] << 8) | buffer[13];
    
    return true;
}

void I2CController::calibrateSensor() {
    ROS_INFO("Calibrating GY-521 sensor... Keep the sensor still for 3 seconds!");
    
    const int samples = 100;  // 더 많은 샘플로 정확도 향상
    double accel_x_sum = 0, accel_y_sum = 0, accel_z_sum = 0;
    double gyro_x_sum = 0, gyro_y_sum = 0, gyro_z_sum = 0;
    
    SensorData data;
    int valid_samples = 0;
    
    for (int i = 0; i < samples; i++) {
        if (readSensorData(data)) {
            accel_x_sum += data.accel_x;
            accel_y_sum += data.accel_y;
            accel_z_sum += data.accel_z;
            gyro_x_sum += data.gyro_x;
            gyro_y_sum += data.gyro_y;
            gyro_z_sum += data.gyro_z;
            valid_samples++;
        }
        
        // 진행 상황 표시
        if (i % 40 == 0) {
            ROS_INFO("Calibration progress: %d%%", (i * 100) / samples);
        }
        
        usleep(25000); // 25ms 대기 (100Hz에 맞춤)
    }
    
    if (valid_samples < samples * 0.8) {
        ROS_WARN("Only %d/%d samples collected during calibration", valid_samples, samples);
    }
    
    // 오프셋 계산
    accel_offset_x = accel_x_sum / valid_samples;
    accel_offset_y = accel_y_sum / valid_samples;
    accel_offset_z = accel_z_sum / valid_samples;
    gyro_offset_x = gyro_x_sum / valid_samples;
    gyro_offset_y = gyro_y_sum / valid_samples;
    gyro_offset_z = gyro_z_sum / valid_samples;
    
    ROS_INFO("Sensor calibration complete:");
    ROS_INFO("  Accel offsets: X=%.2f, Y=%.2f, Z=%.2f", accel_offset_x, accel_offset_y, accel_offset_z);
    ROS_INFO("  Gyro offsets: X=%.2f, Y=%.2f, Z=%.2f", gyro_offset_x, gyro_offset_y, gyro_offset_z);
}

double I2CController::convertAccelData(int16_t raw_data) {
    return (raw_data / accel_scale) * 9.81; // m/s²로 변환
}

double I2CController::convertGyroData(int16_t raw_data) {
    return raw_data / gyro_scale; // °/s로 변환
}

double I2CController::convertTempData(int16_t raw_data) {
    return (raw_data / 340.0) + 36.53; // °C로 변환
}

// I2CSensorNode 구현
I2CSensorNode::I2CSensorNode(ros::NodeHandle& nh, ros::NodeHandle& pnh)
    : nh_(nh), pnh_(pnh),
      sensor_initialized_(false), sensor_calibrated_(false),
      last_error_(HardwareError::NONE),
      filter_enabled_(true), filter_alpha_(0.3),
      use_complementary_filter_(true), complementary_alpha_(0.02),
      stability_threshold_(0.5), confidence_threshold_(70.0) {
    
    // 센서 데이터 초기화
    current_sensor_data_.header.frame_id = "imu_link";
    current_sensor_data_.accel_x = 0.0;
    current_sensor_data_.accel_y = 0.0;
    current_sensor_data_.accel_z = 0.0;
    current_sensor_data_.gyro_x = 0.0;
    current_sensor_data_.gyro_y = 0.0;
    current_sensor_data_.gyro_z = 0.0;
    current_sensor_data_.mag_x = 0.0;
    current_sensor_data_.mag_y = 0.0;
    current_sensor_data_.mag_z = 0.0;
    current_sensor_data_.roll_angle = 0.0;
    current_sensor_data_.pitch_angle = 0.0;
    current_sensor_data_.yaw_angle = 0.0;
    current_sensor_data_.stable_roll_angle = 0.0;
    current_sensor_data_.stable_pitch_angle = 0.0;
    current_sensor_data_.is_stable = false;
    current_sensor_data_.tilt_confidence = 0.0;
    current_sensor_data_.temperature = 0.0;
    current_sensor_data_.i2c_communication_ok = false;
    current_sensor_data_.sensor_error_code = 0;
    
    // 보정값 초기화
    for (int i = 0; i < 3; i++) {
        accel_offset_[i] = 0.0;
        gyro_offset_[i] = 0.0;
        mag_offset_[i] = 0.0;
    }
    
    // 센서 데이터 버퍼 초기화
    for (int i = 0; i < 7; i++) {
        sensor_data_buffer_[i] = 0.0;
    }
}

I2CSensorNode::~I2CSensorNode() {
    ROS_INFO("I2C Sensor Node shutdown complete");
}

bool I2CSensorNode::initialize() {
    ROS_INFO("Initializing I2C Sensor Node...");
    
    // 매개변수 로드
    loadParameters();
    
    // I2C 센서 초기화
    if (!initializeSensor()) {
        setError(HardwareError::I2C_INIT_FAILED, "I2C 센서 초기화 실패");
        return false;
    }
    
    // ROS 토픽 설정
    sensor_data_pub_ = nh_.advertise<balance_robot_nodes::SensorData>("sensor_data", 10);
    imu_pub_ = nh_.advertise<sensor_msgs::Imu>("imu/data", 10);
    temperature_pub_ = nh_.advertise<sensor_msgs::Temperature>("imu/temperature", 10);
    
    // 호환성을 위한 개별 토픽
    roll_pub_ = nh_.advertise<std_msgs::Float32>("sensor_roll", 10);
    pitch_pub_ = nh_.advertise<std_msgs::Float32>("sensor_pitch", 10);
    confidence_pub_ = nh_.advertise<std_msgs::Float32>("sensor_confidence", 10);
    
    // 타이머 설정 (100Hz)
    sensor_timer_ = nh_.createTimer(ros::Duration(0.01), 
                                   &I2CSensorNode::sensorTimerCallback, this);
    
    // IMU 메시지 기본 설정
    imu_msg_.header.frame_id = "imu_link";
    for (int i = 0; i < 9; i++) {
        imu_msg_.orientation_covariance[i] = 0.0;
        imu_msg_.angular_velocity_covariance[i] = 0.0;
        imu_msg_.linear_acceleration_covariance[i] = 0.0;
    }
    imu_msg_.orientation_covariance[0] = 0.01;
    imu_msg_.orientation_covariance[4] = 0.01;
    imu_msg_.orientation_covariance[8] = 0.01;
    imu_msg_.angular_velocity_covariance[0] = 0.001;
    imu_msg_.angular_velocity_covariance[4] = 0.001;
    imu_msg_.angular_velocity_covariance[8] = 0.001;
    imu_msg_.linear_acceleration_covariance[0] = 0.1;
    imu_msg_.linear_acceleration_covariance[4] = 0.1;
    imu_msg_.linear_acceleration_covariance[8] = 0.1;
    
    // 온도 메시지 설정
    temp_msg_.header.frame_id = "imu_link";
    temp_msg_.variance = 1.0;
    
    sensor_initialized_ = true;
    current_sensor_data_.i2c_communication_ok = true;
    
    ROS_INFO("I2C Sensor Node initialization complete");
    return true;
}

void I2CSensorNode::loadParameters() {
    // 필터 설정
    pnh_.param("filter_enabled", filter_enabled_, true);
    pnh_.param("filter_alpha", filter_alpha_, 0.3);
    pnh_.param("use_complementary_filter", use_complementary_filter_, true);
    pnh_.param("complementary_alpha", complementary_alpha_, 0.02);
    
    // 안정성 설정
    pnh_.param("stability_threshold", stability_threshold_, 0.5);
    pnh_.param("confidence_threshold", confidence_threshold_, 70.0);
    
    // 보정값 로드 (옵션)
    pnh_.param("accel_offset_x", accel_offset_[0], 0.0);
    pnh_.param("accel_offset_y", accel_offset_[1], 0.0);
    pnh_.param("accel_offset_z", accel_offset_[2], 0.0);
    pnh_.param("gyro_offset_x", gyro_offset_[0], 0.0);
    pnh_.param("gyro_offset_y", gyro_offset_[1], 0.0);
    pnh_.param("gyro_offset_z", gyro_offset_[2], 0.0);
    
    ROS_INFO("Loaded I2C sensor parameters:");
    ROS_INFO("  Filter: enabled=%s, alpha=%.3f", filter_enabled_ ? "true" : "false", filter_alpha_);
    ROS_INFO("  Complementary filter: enabled=%s, alpha=%.3f", 
             use_complementary_filter_ ? "true" : "false", complementary_alpha_);
    ROS_INFO("  Stability: threshold=%.3f, confidence=%.1f", stability_threshold_, confidence_threshold_);
}

bool I2CSensorNode::initializeSensor() {
    ROS_INFO("Initializing I2C sensor...");
    
    // I2C 컨트롤러 초기화
    if (!i2c_controller_.initI2C()) {
        ROS_ERROR("Failed to initialize I2C");
        return false;
    }
    
    if (!i2c_controller_.initMPU6050()) {
        ROS_ERROR("Failed to initialize MPU6050");
        return false;
    }
    
    // 센서 보정 수행
    calibrateSensor();
    
    ROS_INFO("I2C sensor initialization complete");
    return true;
}

bool I2CSensorNode::performSelfTest() {
    // 기본적인 통신 테스트
    SensorData test_data;
    bool result = i2c_controller_.readSensorData(test_data);
    
    if (!result) {
        ROS_ERROR("I2C communication test failed");
        return false;
    }
    
    ROS_INFO("I2C sensor self-test passed");
    return true;
}

void I2CSensorNode::calibrateSensor() {
    ROS_INFO("Starting sensor calibration...");
    
    // 실제 센서 보정 수행
    i2c_controller_.calibrateSensor();
    
    // 보정값 가져오기
    accel_offset_[0] = i2c_controller_.getAccelOffsetX();
    accel_offset_[1] = i2c_controller_.getAccelOffsetY();
    accel_offset_[2] = i2c_controller_.getAccelOffsetZ();
    gyro_offset_[0] = i2c_controller_.getGyroOffsetX();
    gyro_offset_[1] = i2c_controller_.getGyroOffsetY();
    gyro_offset_[2] = i2c_controller_.getGyroOffsetZ();
    
    sensor_calibrated_ = true;
    
    ROS_INFO("Sensor calibration complete:");
    ROS_INFO("  Gyro offsets: X=%.4f, Y=%.4f, Z=%.4f", 
             gyro_offset_[0], gyro_offset_[1], gyro_offset_[2]);
}

void I2CSensorNode::sensorTimerCallback(const ros::TimerEvent& event) {
    if (!sensor_initialized_) return;
    
    // 센서 데이터 읽기
    if (!readSensorData()) {
        setError(HardwareError::I2C_READ_FAILED, "센서 데이터 읽기 실패");
        return;
    }
    
    // 데이터 필터링
    if (filter_enabled_) {
        filterSensorData();
    }
    
    // 각도 계산
    calculateAngles();
    
    // 안정성 평가
    evaluateStability();
    
    // 신뢰도 계산
    calculateConfidence();
    
    // 온도 보상 적용
    applyTemperatureCompensation();
    
    // 데이터 발행
    publishSensorData();
    publishIMUMessage();
    publishIndividualTopics();
}

bool I2CSensorNode::readSensorData() {
    // 실제 I2C에서 센서 데이터 읽기
    SensorData raw_data;
    bool result = i2c_controller_.readSensorData(raw_data);
    
    if (!result) {
        current_sensor_data_.i2c_communication_ok = false;
        return false;
    }
    
    current_sensor_data_.i2c_communication_ok = true;
    
    // 원시 데이터를 메시지에 복사 (변환 및 보정 적용)
    current_sensor_data_.accel_x = i2c_controller_.convertAccelData(raw_data.accel_x - accel_offset_[0]);
    current_sensor_data_.accel_y = i2c_controller_.convertAccelData(raw_data.accel_y - accel_offset_[1]);
    current_sensor_data_.accel_z = i2c_controller_.convertAccelData(raw_data.accel_z - accel_offset_[2]);
    current_sensor_data_.gyro_x = i2c_controller_.convertGyroData(raw_data.gyro_x - gyro_offset_[0]);
    current_sensor_data_.gyro_y = i2c_controller_.convertGyroData(raw_data.gyro_y - gyro_offset_[1]);
    current_sensor_data_.gyro_z = i2c_controller_.convertGyroData(raw_data.gyro_z - gyro_offset_[2]);
    current_sensor_data_.temperature = i2c_controller_.convertTempData(raw_data.temp);
    
    return true;
}

void I2CSensorNode::filterSensorData() {
    // 저역 통과 필터 적용
    static bool first_run = true;
    static double prev_accel[3] = {0.0, 0.0, 0.0};
    static double prev_gyro[3] = {0.0, 0.0, 0.0};
    
    if (first_run) {
        prev_accel[0] = current_sensor_data_.accel_x;
        prev_accel[1] = current_sensor_data_.accel_y;
        prev_accel[2] = current_sensor_data_.accel_z;
        prev_gyro[0] = current_sensor_data_.gyro_x;
        prev_gyro[1] = current_sensor_data_.gyro_y;
        prev_gyro[2] = current_sensor_data_.gyro_z;
        first_run = false;
        return;
    }
    
    // 가속도계 필터링
    current_sensor_data_.accel_x = lowPassFilter(current_sensor_data_.accel_x, prev_accel[0], filter_alpha_);
    current_sensor_data_.accel_y = lowPassFilter(current_sensor_data_.accel_y, prev_accel[1], filter_alpha_);
    current_sensor_data_.accel_z = lowPassFilter(current_sensor_data_.accel_z, prev_accel[2], filter_alpha_);
    
    // 자이로스코프 필터링
    current_sensor_data_.gyro_x = lowPassFilter(current_sensor_data_.gyro_x, prev_gyro[0], filter_alpha_);
    current_sensor_data_.gyro_y = lowPassFilter(current_sensor_data_.gyro_y, prev_gyro[1], filter_alpha_);
    current_sensor_data_.gyro_z = lowPassFilter(current_sensor_data_.gyro_z, prev_gyro[2], filter_alpha_);
    
    // 이전 값 업데이트
    prev_accel[0] = current_sensor_data_.accel_x;
    prev_accel[1] = current_sensor_data_.accel_y;
    prev_accel[2] = current_sensor_data_.accel_z;
    prev_gyro[0] = current_sensor_data_.gyro_x;
    prev_gyro[1] = current_sensor_data_.gyro_y;
    prev_gyro[2] = current_sensor_data_.gyro_z;
}

void I2CSensorNode::calculateAngles() {
    // 가속도계에서 Roll, Pitch 계산
    double accel_roll = atan2(current_sensor_data_.accel_y, current_sensor_data_.accel_z) * 180.0 / M_PI;
    double accel_pitch = atan2(-current_sensor_data_.accel_x, 
                              sqrt(current_sensor_data_.accel_y * current_sensor_data_.accel_y + 
                                   current_sensor_data_.accel_z * current_sensor_data_.accel_z)) * 180.0 / M_PI;
    
    // 즉시 응답 각도 (가속도계 기반)
    current_sensor_data_.roll_angle = accel_roll;
    current_sensor_data_.pitch_angle = accel_pitch;
    
    // 상보 필터 적용 (안정화된 각도)
    if (use_complementary_filter_) {
        static bool first_run = true;
        static double prev_stable_roll = 0.0;
        static double prev_stable_pitch = 0.0;
        static ros::Time prev_time = ros::Time::now();
        
        ros::Time current_time = ros::Time::now();
        double dt = (current_time - prev_time).toSec();
        
        if (first_run || dt > 1.0) {
            current_sensor_data_.stable_roll_angle = accel_roll;
            current_sensor_data_.stable_pitch_angle = accel_pitch;
            first_run = false;
        } else {
            double temp_roll = current_sensor_data_.stable_roll_angle;
            double temp_pitch = current_sensor_data_.stable_pitch_angle;
            applyComplementaryFilter(temp_roll, accel_roll, current_sensor_data_.gyro_x, dt);
            applyComplementaryFilter(temp_pitch, accel_pitch, current_sensor_data_.gyro_y, dt);
            current_sensor_data_.stable_roll_angle = temp_roll;
            current_sensor_data_.stable_pitch_angle = temp_pitch;
        }
        
        prev_time = current_time;
    } else {
        current_sensor_data_.stable_roll_angle = accel_roll;
        current_sensor_data_.stable_pitch_angle = accel_pitch;
    }
    
    // Yaw는 자력계 없이는 계산하기 어려움 (일단 0으로 설정)
    current_sensor_data_.yaw_angle = 0.0;
    
    // 각도 정규화
    current_sensor_data_.roll_angle = normalizeAngle(current_sensor_data_.roll_angle);
    current_sensor_data_.pitch_angle = normalizeAngle(current_sensor_data_.pitch_angle);
    current_sensor_data_.stable_roll_angle = normalizeAngle(current_sensor_data_.stable_roll_angle);
    current_sensor_data_.stable_pitch_angle = normalizeAngle(current_sensor_data_.stable_pitch_angle);
}

void I2CSensorNode::applyComplementaryFilter(double& angle, double accel_angle, double gyro_rate, double dt) {
    // 상보 필터: angle = α * (이전각도 + 자이로적분) + (1-α) * 가속도각도
    angle = complementary_alpha_ * (angle + gyro_rate * dt) + (1.0 - complementary_alpha_) * accel_angle;
}

void I2CSensorNode::evaluateStability() {
    // 자이로스코프 벡터 크기로 안정성 판단
    double gyro_magnitude = vectorMagnitude(current_sensor_data_.gyro_x, 
                                          current_sensor_data_.gyro_y, 
                                          current_sensor_data_.gyro_z);
    
    current_sensor_data_.is_stable = (gyro_magnitude < stability_threshold_);
}

void I2CSensorNode::calculateConfidence() {
    // 가속도계 벡터 크기 (중력 가속도에 가까울수록 신뢰도 높음)
    double accel_magnitude = vectorMagnitude(current_sensor_data_.accel_x,
                                           current_sensor_data_.accel_y,
                                           current_sensor_data_.accel_z);
    
    // 1G(9.81)에 가까울수록 높은 신뢰도
    double gravity_error = std::abs(accel_magnitude - 9.81) / 9.81;
    double confidence = std::max(0.0, (1.0 - gravity_error * 2.0)) * 100.0;
    
    // 자이로스코프 안정성도 고려
    double gyro_magnitude = vectorMagnitude(current_sensor_data_.gyro_x,
                                          current_sensor_data_.gyro_y,
                                          current_sensor_data_.gyro_z);
    double gyro_stability = std::max(0.0, (1.0 - gyro_magnitude / 5.0)) * 100.0;
    
    // 두 신뢰도의 평균
    current_sensor_data_.tilt_confidence = (confidence + gyro_stability) / 2.0;
    
    // 0-100% 범위로 제한 (float 타입으로 캐스팅)
    current_sensor_data_.tilt_confidence = std::max(0.0f, std::min(100.0f, current_sensor_data_.tilt_confidence));
}

void I2CSensorNode::applyTemperatureCompensation() {
    // 온도 보상은 센서 특성에 따라 구현
    // 현재는 기본 구현만 제공
    if (current_sensor_data_.temperature < -40.0 || current_sensor_data_.temperature > 85.0) {
        ROS_WARN_THROTTLE(5.0, "Temperature out of normal range: %.1f°C", current_sensor_data_.temperature);
    }
}

void I2CSensorNode::publishSensorData() {
    current_sensor_data_.header.stamp = ros::Time::now();
    current_sensor_data_.sensor_error_code = static_cast<uint8_t>(last_error_);
    current_sensor_data_.sensor_error_message = hardwareErrorToString(last_error_);
    
    sensor_data_pub_.publish(current_sensor_data_);
}

void I2CSensorNode::publishIMUMessage() {
    imu_msg_.header.stamp = ros::Time::now();
    
    // 가속도 데이터 (m/s²)
    imu_msg_.linear_acceleration.x = current_sensor_data_.accel_x;
    imu_msg_.linear_acceleration.y = current_sensor_data_.accel_y;
    imu_msg_.linear_acceleration.z = current_sensor_data_.accel_z;
    
    // 각속도 데이터 (rad/s)
    imu_msg_.angular_velocity.x = current_sensor_data_.gyro_x * M_PI / 180.0;
    imu_msg_.angular_velocity.y = current_sensor_data_.gyro_y * M_PI / 180.0;
    imu_msg_.angular_velocity.z = current_sensor_data_.gyro_z * M_PI / 180.0;
    
    // 자세 데이터 (간단한 Roll, Pitch만)
    double roll_rad = current_sensor_data_.stable_roll_angle * M_PI / 180.0;
    double pitch_rad = current_sensor_data_.stable_pitch_angle * M_PI / 180.0;
    double yaw_rad = 0.0;
    
    // 쿼터니언 변환 (Roll, Pitch, Yaw에서)
    double cy = cos(yaw_rad * 0.5);
    double sy = sin(yaw_rad * 0.5);
    double cp = cos(pitch_rad * 0.5);
    double sp = sin(pitch_rad * 0.5);
    double cr = cos(roll_rad * 0.5);
    double sr = sin(roll_rad * 0.5);
    
    imu_msg_.orientation.w = cr * cp * cy + sr * sp * sy;
    imu_msg_.orientation.x = sr * cp * cy - cr * sp * sy;
    imu_msg_.orientation.y = cr * sp * cy + sr * cp * sy;
    imu_msg_.orientation.z = cr * cp * sy - sr * sp * cy;
    
    imu_pub_.publish(imu_msg_);
    
    // 온도 메시지
    temp_msg_.header.stamp = ros::Time::now();
    temp_msg_.temperature = current_sensor_data_.temperature;
    temperature_pub_.publish(temp_msg_);
}

void I2CSensorNode::publishIndividualTopics() {
    std_msgs::Float32 roll_msg, pitch_msg, confidence_msg;
    
    // 안정 상태에 따라 다른 각도 사용
    if (current_sensor_data_.is_stable) {
        roll_msg.data = current_sensor_data_.stable_roll_angle;
        pitch_msg.data = current_sensor_data_.stable_pitch_angle;
    } else {
        roll_msg.data = current_sensor_data_.roll_angle;
        pitch_msg.data = current_sensor_data_.pitch_angle;
    }
    
    confidence_msg.data = current_sensor_data_.tilt_confidence;
    
    roll_pub_.publish(roll_msg);
    pitch_pub_.publish(pitch_msg);
    confidence_pub_.publish(confidence_msg);
}

void I2CSensorNode::setError(HardwareError error, const std::string& message) {
    last_error_ = error;
    if (error != HardwareError::NONE) {
        ROS_ERROR("I2C Sensor Error: %s - %s", hardwareErrorToString(error).c_str(), message.c_str());
    }
}

double I2CSensorNode::normalizeAngle(double angle) {
    while (angle > 180.0) angle -= 360.0;
    while (angle < -180.0) angle += 360.0;
    return angle;
}

double I2CSensorNode::lowPassFilter(double input, double prev_output, double alpha) {
    return alpha * input + (1.0 - alpha) * prev_output;
}

double I2CSensorNode::vectorMagnitude(double x, double y, double z) {
    return sqrt(x*x + y*y + z*z);
}

// 메인 함수
int main(int argc, char** argv) {
    ros::init(argc, argv, "i2c_sensor_node");
    ros::NodeHandle nh;
    ros::NodeHandle pnh("~");
    
    I2CSensorNode node(nh, pnh);
    
    if (!node.initialize()) {
        ROS_ERROR("Failed to initialize I2C Sensor Node");
        return -1;
    }
    
    ROS_INFO("I2C Sensor Node started successfully");
    ros::spin();
    
    return 0;
} 