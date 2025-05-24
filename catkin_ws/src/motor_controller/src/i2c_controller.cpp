#include "motor_controller/i2c_controller.h"
#include <iostream>
#include <cmath>

I2CController::I2CController() : 
    i2c_file(-1),
    i2c_device("/dev/i2c-1"),  // Jetson Nano의 기본 I2C 버스
    accel_offset_x(0), accel_offset_y(0), accel_offset_z(0),
    gyro_offset_x(0), gyro_offset_y(0), gyro_offset_z(0),
    accel_scale(16384.0),  // ±2g 설정 시
    gyro_scale(131.0)      // ±250°/s 설정 시
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
    ROS_INFO("Initializing MPU6050...");
    
    // MPU6050 깨우기 (파워 관리 레지스터 초기화)
    if (!writeRegister(PWR_MGMT_1, 0x00)) {
        ROS_ERROR("Failed to wake up MPU6050");
        return false;
    }
    
    usleep(100000); // 100ms 대기
    
    // 샘플레이트 설정 (1kHz / (1 + SMPLRT_DIV))
    if (!writeRegister(SMPLRT_DIV, 0x07)) { // 125Hz
        ROS_ERROR("Failed to set sample rate");
        return false;
    }
    
    // 디지털 로우패스 필터 설정
    if (!writeRegister(CONFIG, 0x06)) {
        ROS_ERROR("Failed to set config");
        return false;
    }
    
    // 자이로스코프 설정 (±250°/s)
    if (!writeRegister(GYRO_CONFIG, 0x00)) {
        ROS_ERROR("Failed to set gyro config");
        return false;
    }
    
    // 가속도계 설정 (±2g)
    if (!writeRegister(ACCEL_CONFIG, 0x00)) {
        ROS_ERROR("Failed to set accel config");
        return false;
    }
    
    usleep(100000); // 100ms 대기
    
    // 센서 캘리브레이션
    calibrateSensor();
    
    ROS_INFO("MPU6050 initialized successfully");
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
    ROS_INFO("Calibrating sensor... Keep the sensor still!");
    
    const int samples = 100;
    double accel_x_sum = 0, accel_y_sum = 0, accel_z_sum = 0;
    double gyro_x_sum = 0, gyro_y_sum = 0, gyro_z_sum = 0;
    
    SensorData data;
    
    for (int i = 0; i < samples; i++) {
        if (readSensorData(data)) {
            accel_x_sum += data.accel_x;
            accel_y_sum += data.accel_y;
            accel_z_sum += data.accel_z;
            gyro_x_sum += data.gyro_x;
            gyro_y_sum += data.gyro_y;
            gyro_z_sum += data.gyro_z;
        }
        usleep(5000); // 5ms 대기
    }
    
    // 오프셋 계산
    accel_offset_x = accel_x_sum / samples;
    accel_offset_y = accel_y_sum / samples;
    accel_offset_z = (accel_z_sum / samples) - accel_scale; // 중력 보정
    gyro_offset_x = gyro_x_sum / samples;
    gyro_offset_y = gyro_y_sum / samples;
    gyro_offset_z = gyro_z_sum / samples;
    
    ROS_INFO("Calibration completed!");
    ROS_INFO("Accel offsets: X=%.2f, Y=%.2f, Z=%.2f", 
             accel_offset_x, accel_offset_y, accel_offset_z);
    ROS_INFO("Gyro offsets: X=%.2f, Y=%.2f, Z=%.2f", 
             gyro_offset_x, gyro_offset_y, gyro_offset_z);
}

double I2CController::convertAccelData(int16_t raw_data) {
    return (raw_data - accel_offset_x) / accel_scale * 9.81; // m/s²로 변환
}

double I2CController::convertGyroData(int16_t raw_data) {
    return raw_data / gyro_scale * M_PI / 180.0; // rad/s로 변환
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
    double accel_y = convertAccelData(data.accel_y);
    double accel_z = (data.accel_z - accel_offset_z) / accel_scale * 9.81;
    
    double gyro_x = (data.gyro_x - gyro_offset_x) / gyro_scale * M_PI / 180.0;
    double gyro_y = (data.gyro_y - gyro_offset_y) / gyro_scale * M_PI / 180.0;
    double gyro_z = (data.gyro_z - gyro_offset_z) / gyro_scale * M_PI / 180.0;
    
    double temperature = convertTempData(data.temp);
    
    p_accel_gyro_xyz[0] = accel_x;
    p_accel_gyro_xyz[1] = accel_y;
    p_accel_gyro_xyz[2] = accel_z;
    p_accel_gyro_xyz[3] = gyro_x;
    p_accel_gyro_xyz[4] = gyro_y;
    p_accel_gyro_xyz[5] = gyro_z;
    
    p_temperature[0] = temperature;

    return true;
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
    return true;
}