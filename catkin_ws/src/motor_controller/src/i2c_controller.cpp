#include "motor_controller/i2c_controller.h"
#include <iostream>
#include <cmath>
#include <fstream>
#include <sstream>
#include <iomanip>

I2CController::I2CController() : 
    i2c_file(-1),
    i2c_device("/dev/i2c-1"),  // Jetson Nano의 기본 I2C 버스
    accel_offset_x(0), accel_offset_y(0), accel_offset_z(0),
    gyro_offset_x(0), gyro_offset_y(0), gyro_offset_z(0),
    calibration_file_path_("/tmp/gy521_calibration.dat"),
    calibration_loaded_(false),
    force_recalibration_(false),
    accel_scale(8192.0),   // ±4g 설정 시 (16384/2)
    gyro_scale(65.5),      // ±500°/s 설정 시 (131/2)
    current_confidence_(100.0),
    buffer_index_(0),
    buffer_full_(false),
    stable_roll_(0.0),
    stable_pitch_(0.0),
    prev_accel_magnitude_(9.81),
    stable_count_(0),
    is_stable_(true),
    last_time_(ros::Time::now())
{
    // 이동 평균 버퍼 초기화
    for(int i = 0; i < FILTER_SIZE; i++) {
        roll_buffer_[i] = 0.0;
        pitch_buffer_[i] = 0.0;
    }
    
    ROS_INFO("I2C Controller initialized with optimized tilt measurement");
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
    
    // 6. 캘리브레이션 (필요시에만)
    performCalibrationIfNeeded();
    
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
    ROS_INFO("Calibrating GY-521 sensor... Keep the sensor still for 5 seconds!");
    
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
    accel_offset_z = (accel_z_sum / valid_samples) - accel_scale; // 중력 보정 (±4g 스케일)
    gyro_offset_x = gyro_x_sum / valid_samples;
    gyro_offset_y = gyro_y_sum / valid_samples;
    gyro_offset_z = gyro_z_sum / valid_samples;
    
    ROS_INFO("GY-521 calibration completed!");
    ROS_INFO("Accel offsets (±4g): X=%.1f, Y=%.1f, Z=%.1f", 
             accel_offset_x, accel_offset_y, accel_offset_z);
    ROS_INFO("Gyro offsets (±500°/s): X=%.1f, Y=%.1f, Z=%.1f", 
             gyro_offset_x, gyro_offset_y, gyro_offset_z);
    
    // 캘리브레이션 품질 검사
    double accel_magnitude = sqrt(pow(accel_offset_x/accel_scale, 2) + 
                                 pow(accel_offset_y/accel_scale, 2) + 
                                 pow((accel_offset_z + accel_scale)/accel_scale, 2));
    ROS_INFO("Gravity magnitude check: %.3f g (should be ~1.0)", accel_magnitude);
}

double I2CController::convertAccelData(int16_t raw_data) {
    return (raw_data) / accel_scale * 9.81; // m/s²로 변환 (±4g 스케일)
}

double I2CController::convertGyroData(int16_t raw_data) {
    return (raw_data - gyro_offset_x) / gyro_scale * M_PI / 180.0; // rad/s로 변환 (±500°/s 스케일)
}

double I2CController::convertTempData(int16_t raw_data) {
    return raw_data / 340.0 + 36.53; // 섭씨 온도로 변환
}

bool I2CController::getData(double* p_accel_gyro_xyz, double* p_temperature) {
    SensorData data;
    
    if (!readSensorData(data)) {
        ROS_WARN("Failed to read sensor data");
        return false;
    }
    
    // 데이터 변환
    double accel_x = convertAccelData(data.accel_x);
    double accel_y = (data.accel_y - accel_offset_y) / accel_scale * 9.81;
    double accel_z = (data.accel_z - accel_offset_z) / accel_scale * 9.81;
    
    double gyro_x = (data.gyro_x - gyro_offset_x) / 131.0;//gyro_scale * 180.0 / M_PI;
    double gyro_y = (data.gyro_y - gyro_offset_y) / 131.0;//gyro_scale * 180.0 / M_PI;
    double gyro_z = (data.gyro_z - gyro_offset_z) / 131.0;//gyro_scale * 180.0 / M_PI;
    
    double temperature = convertTempData(data.temp);
    
    // 움직임 상태 분석
    double motion_confidence = analyzeMotionConfidence(accel_x, accel_y, accel_z, gyro_x, gyro_y, gyro_z);
    current_confidence_ = motion_confidence;
    
    ros::Time current_time = ros::Time::now();
    double dt;
    double dt_microseconds;
    
    // 첫 번째 호출인지 확인 (last_time_이 0인 경우)
    if (last_time_.toSec() == 0.0) {
        last_time_ = current_time;
        dt = 0.002; // 기본값 1ms
        dt_microseconds = 2000.0;
    } else {
        ros::Duration elapsed = current_time - last_time_;
        dt = elapsed.toSec();
        
        // 마이크로초 단위로도 계산 (디버깅용)
        dt_microseconds = (double) elapsed.toNSec() / 1000.0;  // 나노초를 마이크로초로 변환
        
        last_time_ = current_time;
    }
    /*ROS_INFO("last_time %4.1f |dt %4.1f |dt_us %4.1f"
        ,last_time_.toSec(),dt,dt_microseconds);*/
    
    // 최적화된 기울기 계산
    calculateOptimalTilt(accel_x, accel_y, accel_z);
    

    float alpha = 0.96;

    static double gyro_x_filtered;
    static double gyro_y_filtered;
    static double gyro_z_filtered;
    gyro_x_filtered = alpha * (gyro_x_filtered+gyro_x*dt) + (1.f - alpha) * current_angles_.roll;
    gyro_y_filtered = alpha * (gyro_y_filtered+gyro_y*dt) + (1.f - alpha) * current_angles_.pitch;
    gyro_z_filtered = alpha * (gyro_z_filtered+gyro_z*dt) + (1.f - alpha) * current_angles_.yaw;
     

    p_accel_gyro_xyz[0] = accel_x;
    p_accel_gyro_xyz[1] = accel_y;
    p_accel_gyro_xyz[2] = accel_z;
    p_accel_gyro_xyz[3] = gyro_x;
    p_accel_gyro_xyz[4] = gyro_y;
    p_accel_gyro_xyz[5] = gyro_z;
    p_accel_gyro_xyz[6] = gyro_x_filtered;
    p_accel_gyro_xyz[7] = gyro_y_filtered;
    p_accel_gyro_xyz[8] = gyro_z_filtered;
    p_accel_gyro_xyz[9] = dt;  // 초 단위
    p_accel_gyro_xyz[10] = dt_microseconds;  // 마이크로초 단위
    
    p_temperature[0] = temperature;

    return true;
}

void I2CController::calculateOptimalTilt(double accel_x, double accel_y, double accel_z) {
    // 1. 안정 상태 감지
    bool currently_stable = detectStableState(accel_x, accel_y, accel_z);
    
    // 2. 기본 기울기 계산 (순수 가속도계)
    double gravity_magnitude = sqrt(accel_x*accel_x + accel_y*accel_y + accel_z*accel_z);
    
    // 중력 크기가 너무 비정상적이면 이전 값 유지
    if (gravity_magnitude < 5.0 || gravity_magnitude > 15.0) {
        ROS_WARN("Abnormal gravity reading: %.2f m/s² - keeping previous values", gravity_magnitude);
        return;
    }
    
    double norm_x = accel_x / gravity_magnitude;
    double norm_y = accel_y / gravity_magnitude; 
    double norm_z = accel_z / gravity_magnitude;
    
    double raw_roll = atan2(norm_y, norm_z) * 180.0 / M_PI;
    double raw_pitch = atan2(-norm_x, sqrt(norm_y*norm_y + norm_z*norm_z)) * 180.0 / M_PI;
    
    // 각도 범위 정규화
    if (raw_roll > 180.0) raw_roll -= 360.0;
    if (raw_roll < -180.0) raw_roll += 360.0;
    raw_pitch = std::max(-90.0, std::min(90.0, raw_pitch));
    
    // 3. 현재 각도 업데이트 (즉시 응답용)
    current_angles_.roll = raw_roll;
    current_angles_.pitch = raw_pitch;
    current_angles_.yaw = 0.0;
    
    // 4. 이동 평균 필터 업데이트
    updateMovingAverage(raw_roll, raw_pitch);
    
    // 5. 안정 상태에 따른 최적 각도 결정
    if (currently_stable && stable_count_ > 5) {
        // 안정 상태: 이동 평균 사용 (더 정확)
        stable_roll_ = getMovingAverage(roll_buffer_, buffer_full_ ? FILTER_SIZE : buffer_index_);
        stable_pitch_ = getMovingAverage(pitch_buffer_, buffer_full_ ? FILTER_SIZE : buffer_index_);
    } else {
        // 불안정 상태: 즉시 값 사용하되 급격한 변화 제한
        double max_change = 5.0;  // 최대 5도 변화 제한
        
        double roll_diff = raw_roll - stable_roll_;
        double pitch_diff = raw_pitch - stable_pitch_;
        
        if (std::abs(roll_diff) > max_change) {
            stable_roll_ += (roll_diff > 0) ? max_change : -max_change;
        } else {
            stable_roll_ = raw_roll;
        }
        
        if (std::abs(pitch_diff) > max_change) {
            stable_pitch_ += (pitch_diff > 0) ? max_change : -max_change;
        } else {
            stable_pitch_ = raw_pitch;
        }
    }
}

bool I2CController::detectStableState(double accel_x, double accel_y, double accel_z) {
    double current_magnitude = sqrt(accel_x*accel_x + accel_y*accel_y + accel_z*accel_z);
    double magnitude_change = std::abs(current_magnitude - prev_accel_magnitude_);
    
    // 안정 상태 판단 기준
    const double STABLE_THRESHOLD = 0.3;  // 0.3 m/s² 이하 변화
    
    if (magnitude_change < STABLE_THRESHOLD) {
        stable_count_++;
        if (stable_count_ > 10) {  // 10회 연속 안정
            is_stable_ = true;
        }
    } else {
        stable_count_ = 0;
        is_stable_ = false;
    }
    
    prev_accel_magnitude_ = current_magnitude;
    return is_stable_;
}

void I2CController::updateMovingAverage(double roll, double pitch) {
    roll_buffer_[buffer_index_] = roll;
    pitch_buffer_[buffer_index_] = pitch;
    
    buffer_index_++;
    if (buffer_index_ >= FILTER_SIZE) {
        buffer_index_ = 0;
        buffer_full_ = true;
    }
}

double I2CController::getMovingAverage(const double* buffer, int size) const {
    double sum = 0.0;
    for (int i = 0; i < size; i++) {
        sum += buffer[i];
    }
    return sum / size;
}

double I2CController::analyzeMotionConfidence(double accel_x, double accel_y, double accel_z, 
                                            double gyro_x, double gyro_y, double gyro_z) {
    // 중력 크기 분석
    double gravity_magnitude = sqrt(accel_x*accel_x + accel_y*accel_y + accel_z*accel_z);
    double gravity_deviation = std::abs(gravity_magnitude - 9.81);
    
    // 자이로 움직임 분석
    double gyro_motion = sqrt(gyro_x*gyro_x + gyro_y*gyro_y + gyro_z*gyro_z);
    
    // 신뢰도 계산 (0-100%)
    double confidence = 100.0;
    
    // 중력 편차에 따른 신뢰도 감소
    if (gravity_deviation > 0.5) {
        confidence -= (gravity_deviation - 0.5) * 20.0;  // 0.5 m/s² 초과시 감소
    }
    
    // 자이로 움직임에 따른 신뢰도 감소
    if (gyro_motion > 5.0) {  // 5 deg/s 초과시
        confidence -= (gyro_motion - 5.0) * 2.0;
    }
    
    // 신뢰도 범위 제한
    confidence = std::max(0.0, std::min(100.0, confidence));
    
    return confidence;
}

// 신뢰도 접근자 추가
double I2CController::getTiltConfidence() const {
    return current_confidence_;
}

bool I2CController::init() {
    if (!initI2C()) {
        ROS_ERROR("Failed to initialize I2C");
        return false;
    }
    
    if (!initMPU6050()) {
        ROS_ERROR("Failed to initialize MPU6050");
        closeI2C();
        return false;
    }
    
    // 초기 각도 설정
    current_angles_.roll = 0.0;
    current_angles_.pitch = 0.0;
    current_angles_.yaw = 0.0;
    
    return true;
}

// 캘리브레이션 파일 로드
bool I2CController::loadCalibration() {
    std::ifstream file(calibration_file_path_);
    if (!file.is_open()) {
        ROS_INFO("캘리브레이션 파일이 없습니다: %s", calibration_file_path_.c_str());
        return false;
    }
    
    std::string line;
    int line_count = 0;
    
    try {
        // 파일 형식: 각 줄에 하나의 오프셋 값
        if (std::getline(file, line)) {
            accel_offset_x = std::stod(line);
            line_count++;
        }
        if (std::getline(file, line)) {
            accel_offset_y = std::stod(line);
            line_count++;
        }
        if (std::getline(file, line)) {
            accel_offset_z = std::stod(line);
            line_count++;
        }
        if (std::getline(file, line)) {
            gyro_offset_x = std::stod(line);
            line_count++;
        }
        if (std::getline(file, line)) {
            gyro_offset_y = std::stod(line);
            line_count++;
        }
        if (std::getline(file, line)) {
            gyro_offset_z = std::stod(line);
            line_count++;
        }
        
        file.close();
        
        if (line_count != 6) {
            ROS_WARN("캘리브레이션 파일 형식이 잘못되었습니다 (라인 수: %d)", line_count);
            return false;
        }
        
        ROS_INFO("캘리브레이션 데이터 로드 완료:");
        ROS_INFO("  Accel offsets: X=%.1f, Y=%.1f, Z=%.1f", 
                 accel_offset_x, accel_offset_y, accel_offset_z);
        ROS_INFO("  Gyro offsets: X=%.1f, Y=%.1f, Z=%.1f", 
                 gyro_offset_x, gyro_offset_y, gyro_offset_z);
        
        calibration_loaded_ = true;
        return true;
        
    } catch (const std::exception& e) {
        ROS_ERROR("캘리브레이션 파일 읽기 오류: %s", e.what());
        file.close();
        return false;
    }
}

// 캘리브레이션 파일 저장
bool I2CController::saveCalibration() {
    std::ofstream file(calibration_file_path_);
    if (!file.is_open()) {
        ROS_ERROR("캘리브레이션 파일 생성 실패: %s", calibration_file_path_.c_str());
        return false;
    }
    
    // 고정소수점 형식으로 저장
    file << std::fixed << std::setprecision(3);
    file << accel_offset_x << std::endl;
    file << accel_offset_y << std::endl;
    file << accel_offset_z << std::endl;
    file << gyro_offset_x << std::endl;
    file << gyro_offset_y << std::endl;
    file << gyro_offset_z << std::endl;
    
    file.close();
    
    ROS_INFO("캘리브레이션 데이터 저장 완료: %s", calibration_file_path_.c_str());
    return true;
}

// 캘리브레이션 유효성 검사
bool I2CController::validateCalibration() {
    if (!calibration_loaded_) {
        ROS_INFO("캘리브레이션이 로드되지 않았습니다");
        return false;
    }
    
    // 1. 오프셋 값 범위 검사
    const double MAX_ACCEL_OFFSET = accel_scale * 0.5;  // ±2g 범위
    const double MAX_GYRO_OFFSET = gyro_scale * 10.0;   // ±10°/s 범위
    
    if (std::abs(accel_offset_x) > MAX_ACCEL_OFFSET ||
        std::abs(accel_offset_y) > MAX_ACCEL_OFFSET ||
        std::abs(accel_offset_z) > MAX_ACCEL_OFFSET) {
        ROS_WARN("가속도계 오프셋이 비정상적입니다 (X:%.1f, Y:%.1f, Z:%.1f)", 
                 accel_offset_x, accel_offset_y, accel_offset_z);
        return false;
    }
    
    if (std::abs(gyro_offset_x) > MAX_GYRO_OFFSET ||
        std::abs(gyro_offset_y) > MAX_GYRO_OFFSET ||
        std::abs(gyro_offset_z) > MAX_GYRO_OFFSET) {
        ROS_WARN("자이로스코프 오프셋이 비정상적입니다 (X:%.1f, Y:%.1f, Z:%.1f)", 
                 gyro_offset_x, gyro_offset_y, gyro_offset_z);
        return false;
    }
    
    // 2. 실제 센서 데이터로 검증
    SensorData test_data;
    double test_samples = 10;
    double accel_sum = 0.0;
    
    for (int i = 0; i < test_samples; i++) {
        if (readSensorData(test_data)) {
            double accel_x = (test_data.accel_x - accel_offset_x) / accel_scale * 9.81;
            double accel_y = (test_data.accel_y - accel_offset_y) / accel_scale * 9.81;
            double accel_z = (test_data.accel_z - accel_offset_z) / accel_scale * 9.81;
            
            double magnitude = sqrt(accel_x*accel_x + accel_y*accel_y + accel_z*accel_z);
            accel_sum += magnitude;
        }
        usleep(10000); // 10ms 대기
    }
    
    double avg_gravity = accel_sum / test_samples;
    double gravity_error = std::abs(avg_gravity - 9.81);
    
    if (gravity_error > 2.0) {  // 2 m/s² 이상 오차
        ROS_WARN("캘리브레이션 검증 실패: 중력 크기 오차 %.2f m/s² (측정값: %.2f)", 
                 gravity_error, avg_gravity);
        return false;
    }
    
    ROS_INFO("캘리브레이션 검증 성공: 중력 크기 %.2f m/s² (오차: %.2f)", 
             avg_gravity, gravity_error);
    return true;
}

// 필요시에만 캘리브레이션 수행
void I2CController::performCalibrationIfNeeded() {
    ROS_INFO("캘리브레이션 상태 확인 중...");
    
    // ROS 파라미터에서 설정 읽기
    ros::NodeHandle nh;
    std::string param_calibration_path;
    bool param_force_recalibration = false;
    
    if (nh.getParam("calibration_file_path", param_calibration_path)) {
        calibration_file_path_ = param_calibration_path;
        ROS_INFO("캘리브레이션 파일 경로: %s", calibration_file_path_.c_str());
    }
    
    if (nh.getParam("force_recalibration", param_force_recalibration)) {
        force_recalibration_ = param_force_recalibration;
        if (force_recalibration_) {
            ROS_WARN("강제 재캘리브레이션 모드가 활성화되었습니다!");
        }
    }
    
    // 강제 재캘리브레이션이 요청된 경우
    if (force_recalibration_) {
        ROS_INFO("강제 재캘리브레이션을 수행합니다...");
        calibrateSensor();
        
        if (saveCalibration()) {
            ROS_INFO("강제 재캘리브레이션이 완료되고 저장되었습니다.");
        } else {
            ROS_WARN("강제 재캘리브레이션 저장에 실패했습니다.");
        }
        return;
    }
    
    // 1. 기존 캘리브레이션 로드 시도
    bool loaded = loadCalibration();
    
    // 2. 로드된 캘리브레이션 검증
    bool valid = false;
    if (loaded) {
        valid = validateCalibration();
    }
    
    // 3. 캘리브레이션이 필요한 경우
    if (!loaded || !valid) {
        if (!loaded) {
            ROS_INFO("기존 캘리브레이션 파일이 없습니다. 새로 캘리브레이션을 수행합니다.");
        } else {
            ROS_WARN("기존 캘리브레이션이 유효하지 않습니다. 재캘리브레이션을 수행합니다.");
        }
        
        // 새로운 캘리브레이션 수행
        calibrateSensor();
        
        // 캘리브레이션 결과 저장
        if (saveCalibration()) {
            ROS_INFO("새로운 캘리브레이션이 저장되었습니다.");
        } else {
            ROS_WARN("캘리브레이션 저장에 실패했습니다.");
        }
    } else {
        ROS_INFO("기존 캘리브레이션이 유효합니다. 재캘리브레이션을 건너뜁니다.");
    }
}



