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
#include <cmath>

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
    
    // 각도 저장 구조체
    struct AngleData {
        double roll;    // X축 회전 (좌우 기울기)
        double pitch;   // Y축 회전 (앞뒤 기울기)
        double yaw;     // Z축 회전
        
        AngleData() : roll(0.0), pitch(0.0), yaw(0.0) {}
    };
    
    // 캘리브레이션 오프셋
    double accel_offset_x, accel_offset_y, accel_offset_z;
    double gyro_offset_x, gyro_offset_y, gyro_offset_z;
    
    // 캘리브레이션 파일 관리
    std::string calibration_file_path_;
    bool calibration_loaded_;
    bool force_recalibration_;
    
    // 스케일 팩터
    double accel_scale;
    double gyro_scale;
    
    // 각도 계산 관련
    AngleData current_angles_;
    
    // 신뢰도 정보
    double current_confidence_;           // 현재 측정 신뢰도 (0-100%)
    
    // 최적화된 기울기 측정을 위한 변수들
    static const int FILTER_SIZE = 10;    // 이동 평균 필터 크기
    double roll_buffer_[FILTER_SIZE];     // Roll 각도 버퍼
    double pitch_buffer_[FILTER_SIZE];    // Pitch 각도 버퍼
    int buffer_index_;                    // 현재 버퍼 인덱스
    bool buffer_full_;                    // 버퍼가 가득 찼는지 여부
    
    double stable_roll_;                  // 안정화된 Roll 각도
    double stable_pitch_;                 // 안정화된 Pitch 각도
    
    // 움직임 상태 추적
    double prev_accel_magnitude_;         // 이전 가속도 크기
    int stable_count_;                    // 안정 상태 카운터
    bool is_stable_;                      // 현재 안정 상태 여부
    
    // 시간 추적
    ros::Time last_time_;                 // 이전 측정 시간
    
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
    bool loadCalibration();
    bool saveCalibration();
    bool validateCalibration();
    void performCalibrationIfNeeded();
    
    // 데이터 변환
    double convertAccelData(int16_t raw_data);
    double convertGyroData(int16_t raw_data);
    double convertTempData(int16_t raw_data);
    
    // 신뢰도 분석
    double analyzeMotionConfidence(double accel_x, double accel_y, double accel_z, 
                                 double gyro_x, double gyro_y, double gyro_z);
    
    // 각도 접근자
    double getRoll() const { return current_angles_.roll; }
    double getPitch() const { return current_angles_.pitch; }
    double getYaw() const { return current_angles_.yaw; }
    
    // 신뢰도 접근자
    double getTiltConfidence() const;
    
    // 최적화된 각도 접근자
    double getStableRoll() const { return stable_roll_; }
    double getStablePitch() const { return stable_pitch_; }
    bool isStable() const { return is_stable_; }
    
    // ROS 콜백
    bool getData(double*, double*);
    
    // 메인 실행 함수
    bool init();
    

    
    // 최적화된 기울기 측정 함수들
    void updateMovingAverage(double roll, double pitch);
    bool detectStableState(double accel_x, double accel_y, double accel_z);
    void calculateOptimalTilt(double accel_x, double accel_y, double accel_z);
    double getMovingAverage(const double* buffer, int size) const;
};

#endif // I2C_CONTROLLER_H
