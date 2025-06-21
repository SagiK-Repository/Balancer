#include "balance_robot_nodes/i2c_sensor_node.h"
#include <ros/ros.h>
#include <std_msgs/Float32.h>
#include <cmath>

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
    i2c_controller_.init();
    
    // 자가 진단 수행
    if (!performSelfTest()) {
        ROS_ERROR("I2C sensor self-test failed");
        return false;
    }
    
    // 센서 보정 수행
    calibrateSensor();
    
    ROS_INFO("I2C sensor initialization complete");
    return true;
}

bool I2CSensorNode::performSelfTest() {
    // 기본적인 통신 테스트
    double test_data[7];
    bool result = i2c_controller_.getData(test_data, test_data + 6);
    
    if (!result) {
        ROS_ERROR("I2C communication test failed");
        return false;
    }
    
    // 데이터 유효성 검사
    for (int i = 0; i < 6; i++) {
        if (std::isnan(test_data[i]) || std::isinf(test_data[i])) {
            ROS_ERROR("Invalid sensor data detected");
            return false;
        }
    }
    
    ROS_INFO("I2C sensor self-test passed");
    return true;
}

void I2CSensorNode::calibrateSensor() {
    ROS_INFO("Starting sensor calibration...");
    
    const int calibration_samples = 100;
    double gyro_sum[3] = {0.0, 0.0, 0.0};
    
    // 자이로스코프 오프셋 계산 (정지 상태에서)
    for (int i = 0; i < calibration_samples; i++) {
        double data[7];
        if (i2c_controller_.getData(data, data + 6)) {
            gyro_sum[0] += data[3];  // gyro_x
            gyro_sum[1] += data[4];  // gyro_y
            gyro_sum[2] += data[5];  // gyro_z
        }
        ros::Duration(0.01).sleep();  // 10ms 대기
    }
    
    gyro_offset_[0] = gyro_sum[0] / calibration_samples;
    gyro_offset_[1] = gyro_sum[1] / calibration_samples;
    gyro_offset_[2] = gyro_sum[2] / calibration_samples;
    
    sensor_calibrated_ = true;
    
    ROS_INFO("Sensor calibration complete:");
    ROS_INFO("  Gyro offsets: X=%.4f, Y=%.4f, Z=%.4f", 
             gyro_offset_[0], gyro_offset_[1], gyro_offset_[2]);
}

void I2CSensorNode::sensorTimerCallback(const ros::TimerEvent& event) {
    if (!sensor_initialized_) return;
    
    // 센서 데이터 읽기
    if (!readSensorData()) {
        setError(HardwareError::SENSOR_READ_FAILED, "센서 데이터 읽기 실패");
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
    
    // 온도 보상 적용 (옵션)
    applyTemperatureCompensation();
    
    // 데이터 발행
    publishSensorData();
    publishIMUMessage();
    publishIndividualTopics();
}

bool I2CSensorNode::readSensorData() {
    // I2C에서 센서 데이터 읽기
    bool result = i2c_controller_.getData(sensor_data_buffer_, sensor_data_buffer_ + 6);
    
    if (!result) {
        current_sensor_data_.i2c_communication_ok = false;
        return false;
    }
    
    current_sensor_data_.i2c_communication_ok = true;
    
    // 원시 데이터를 메시지에 복사 (보정 적용)
    current_sensor_data_.accel_x = sensor_data_buffer_[0] - accel_offset_[0];
    current_sensor_data_.accel_y = sensor_data_buffer_[1] - accel_offset_[1];
    current_sensor_data_.accel_z = sensor_data_buffer_[2] - accel_offset_[2];
    current_sensor_data_.gyro_x = sensor_data_buffer_[3] - gyro_offset_[0];
    current_sensor_data_.gyro_y = sensor_data_buffer_[4] - gyro_offset_[1];
    current_sensor_data_.gyro_z = sensor_data_buffer_[5] - gyro_offset_[2];
    current_sensor_data_.temperature = sensor_data_buffer_[6];
    
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
    return sqrt(x * x + y * y + z * z);
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