#include "integrated_balancer/i2c_controller.h"
#include <cstring>
#include <unistd.h>

I2CController::I2CController(const std::string& device, uint8_t address)
    : i2c_file(-1), i2c_device(device), i2c_address(address),
      accel_offset_x(0), accel_offset_y(0), accel_offset_z(0),
      gyro_offset_x(0), gyro_offset_y(0), gyro_offset_z(0),
      accel_scale(8192.0),   // ±4g 설정 시 (16384/2)
      gyro_scale(65.5)       // ±500°/s 설정 시 (131/2)
{
}

I2CController::~I2CController() {
    closeI2C();
}

bool I2CController::initI2C() {
    // I2C 디바이스 파일 열기
    i2c_file = open(i2c_device.c_str(), O_RDWR);
    if (i2c_file < 0) {
        return false;
    }
    
    // I2C 슬레이브 주소 설정
    if (ioctl(i2c_file, I2C_SLAVE, i2c_address) < 0) {
        close(i2c_file);
        i2c_file = -1;
        return false;
    }
    
    return true;
}

void I2CController::closeI2C() {
    if (i2c_file >= 0) {
        close(i2c_file);
        i2c_file = -1;
    }
}

bool I2CController::initMPU6050() {
    // 1. MPU6050 리셋 및 깨우기
    if (!writeRegister(PWR_MGMT_1, 0x80)) {  // 디바이스 리셋
        return false;
    }
    usleep(100000); // 100ms 대기 (리셋 완료)
    
    if (!writeRegister(PWR_MGMT_1, 0x01)) {  // X 자이로 클럭 사용, 슬립 해제
        return false;
    }
    usleep(50000); // 50ms 대기
    
    // 2. 샘플레이트 설정 (1kHz / (1 + SMPLRT_DIV))
    if (!writeRegister(SMPLRT_DIV, 0x09)) { // 100Hz (밸런싱에 최적)
        return false;
    }
    
    // 3. 디지털 로우패스 필터 설정
    if (!writeRegister(CONFIG, 0x03)) {  // 44Hz LPF (노이즈 감소)
        return false;
    }
    
    // 4. 자이로스코프 설정 (±500°/s - 밸런싱에 적합)
    if (!writeRegister(GYRO_CONFIG, 0x08)) {  // ±500°/s
        return false;
    }
    
    // 5. 가속도계 설정 (±4g - 더 넓은 범위)
    if (!writeRegister(ACCEL_CONFIG, 0x08)) {  // ±4g
        return false;
    }
    
    usleep(100000); // 100ms 대기 (설정 안정화)
    
    return true;
}

bool I2CController::writeRegister(uint8_t reg, uint8_t value) {
    uint8_t buffer[2] = {reg, value};
    
    if (write(i2c_file, buffer, 2) != 2) {
        return false;
    }
    
    return true;
}

bool I2CController::readRegister(uint8_t reg, uint8_t* data, int length) {
    // 레지스터 주소 쓰기
    if (write(i2c_file, &reg, 1) != 1) {
        return false;
    }
    
    // 데이터 읽기
    if (read(i2c_file, data, length) != length) {
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
        
        usleep(25000); // 25ms 대기 (100Hz에 맞춤)
    }
    
    if (valid_samples > 0) {
        // 오프셋 계산
        accel_offset_x = accel_x_sum / valid_samples;
        accel_offset_y = accel_y_sum / valid_samples;
        accel_offset_z = accel_z_sum / valid_samples;
        gyro_offset_x = gyro_x_sum / valid_samples;
        gyro_offset_y = gyro_y_sum / valid_samples;
        gyro_offset_z = gyro_z_sum / valid_samples;
    }
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




