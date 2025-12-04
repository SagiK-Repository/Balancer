#ifndef I2C_CONTROLLER_H
#define I2C_CONTROLLER_H

#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <linux/i2c-dev.h>
#include <cstdint>
#include <string>

// MPU6050 레지스터 주소
#define MPU6050_ADDR 0x68
#define PWR_MGMT_1 0x6B
#define SMPLRT_DIV 0x19
#define CONFIG 0x1A
#define GYRO_CONFIG 0x1B
#define ACCEL_CONFIG 0x1C
#define ACCEL_XOUT_H 0x3B

/**
 * @brief 원시 센서 데이터 구조체
 */
struct SensorData {
    int16_t accel_x, accel_y, accel_z;
    int16_t temp;
    int16_t gyro_x, gyro_y, gyro_z;
};

/**
 * @brief I2C 컨트롤러 클래스 (ROS 의존성 없음)
 * 
 * 순수 하드웨어 제어를 위한 I2C 통신 클래스
 */
class I2CController {
public:
    /**
     * @brief 생성자
     * @param device I2C 디바이스 경로 (예: "/dev/i2c-1")
     * @param address I2C 슬레이브 주소
     */
    I2CController(const std::string& device = "/dev/i2c-1", uint8_t address = MPU6050_ADDR);
    
    /**
     * @brief 소멸자
     */
    ~I2CController();
    
    /**
     * @brief I2C 초기화
     * @return 성공 여부
     */
    bool initI2C();
    
    /**
     * @brief I2C 종료
     */
    void closeI2C();
    
    /**
     * @brief MPU6050 초기화
     * @return 성공 여부
     */
    bool initMPU6050();
    
    /**
     * @brief 센서 데이터 읽기
     * @param data 센서 데이터 구조체 참조
     * @return 성공 여부
     */
    bool readSensorData(SensorData& data);
    
    /**
     * @brief 센서 보정 수행
     */
    void calibrateSensor();
    
    /**
     * @brief 가속도 데이터 변환 (m/s²)
     */
    double convertAccelData(int16_t raw_data);
    
    /**
     * @brief 자이로 데이터 변환 (°/s)
     */
    double convertGyroData(int16_t raw_data);
    
    /**
     * @brief 온도 데이터 변환 (°C)
     */
    double convertTempData(int16_t raw_data);
    
    // 보정값 접근자
    double getAccelOffsetX() const { return accel_offset_x; }
    double getAccelOffsetY() const { return accel_offset_y; }
    double getAccelOffsetZ() const { return accel_offset_z; }
    double getGyroOffsetX() const { return gyro_offset_x; }
    double getGyroOffsetY() const { return gyro_offset_y; }
    double getGyroOffsetZ() const { return gyro_offset_z; }

private:
    bool writeRegister(uint8_t reg, uint8_t value);
    bool readRegister(uint8_t reg, uint8_t* data, int length);
    
    int i2c_file;
    std::string i2c_device;
    uint8_t i2c_address;
    
    // 보정값
    double accel_offset_x, accel_offset_y, accel_offset_z;
    double gyro_offset_x, gyro_offset_y, gyro_offset_z;
    
    // 스케일 팩터
    double accel_scale;  // ±4g 설정
    double gyro_scale;   // ±500°/s 설정
};

#endif // I2C_CONTROLLER_H




