#ifndef I2C_CONTROLLER_H
#define I2C_CONTROLLER_H

#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <geometry_msgs/Vector3.h>
#include <std_msgs/Float64MultiArray.h>
#include <linux/i2c-dev.h>
#include <sys/ioctl.h>
#include <fcntl.h>
#include <unistd.h>
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
#define ACCEL_XOUT_L 0x3C
#define ACCEL_YOUT_H 0x3D
#define ACCEL_YOUT_L 0x3E
#define ACCEL_ZOUT_H 0x3F
#define ACCEL_ZOUT_L 0x40
#define TEMP_OUT_H 0x41
#define TEMP_OUT_L 0x42
#define GYRO_XOUT_H 0x43
#define GYRO_XOUT_L 0x44
#define GYRO_YOUT_H 0x45
#define GYRO_YOUT_L 0x46
#define GYRO_ZOUT_H 0x47
#define GYRO_ZOUT_L 0x48

class I2CController {
private:
    int i2c_file;
    std::string i2c_device;
    
    // MPU6050 데이터
    struct SensorData {
        int16_t accel_x, accel_y, accel_z;
        int16_t temp;
        int16_t gyro_x, gyro_y, gyro_z;
    };
    
    // 캘리브레이션 오프셋
    double accel_offset_x, accel_offset_y, accel_offset_z;
    double gyro_offset_x, gyro_offset_y, gyro_offset_z;
    
    // 스케일 팩터
    double accel_scale;
    double gyro_scale;
    
public:
    I2CController();
    ~I2CController();
    
    // I2C 초기화 및 해제
    bool initI2C();
    void closeI2C();
    
    // MPU6050 초기화
    bool initMPU6050();
    
    // I2C 읽기/쓰기 함수
    bool writeRegister(uint8_t reg, uint8_t value);
    bool readRegister(uint8_t reg, uint8_t* data, int length = 1);
    
    // MPU6050 데이터 읽기
    bool readSensorData(SensorData& data);
    
    // 캘리브레이션
    void calibrateSensor();
    
    // 데이터 변환
    double convertAccelData(int16_t raw_data);
    double convertGyroData(int16_t raw_data);
    double convertTempData(int16_t raw_data);
    
    // ROS 콜백
    bool getData(double*, double*);
    
    // 메인 실행 함수
    bool init();
};

#endif // I2C_CONTROLLER_H
