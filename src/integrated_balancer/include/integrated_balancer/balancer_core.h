#ifndef BALANCER_CORE_H
#define BALANCER_CORE_H

#include "integrated_balancer/i2c_controller.h"
#include "integrated_balancer/hardware_spi.h"
#include "integrated_balancer/mcp4921.h"
#include "integrated_balancer/pid_controller.h"
#include <atomic>
#include <mutex>
#include <thread>
#include <pthread.h>
#include <time.h>
#include <cmath>

/**
 * @brief BalancerCore 클래스
 * 
 * ROS 헤더 없이 동작하는 순수 제어 루프 매니저
 * Real-Time 스레드에서 100Hz 고정 주기로 실행
 */
class BalancerCore {
public:
    /**
     * @brief 생성자
     */
    BalancerCore();
    
    /**
     * @brief 소멸자
     */
    ~BalancerCore();
    
    /**
     * @brief 초기화
     * @return 성공 여부
     */
    bool initialize();
    
    /**
     * @brief 제어 루프 시작
     */
    void start();
    
    /**
     * @brief 제어 루프 중지
     */
    void stop();
    
    // 공유 변수 접근자 (ROS Bridge에서 사용)
    float getCurrentAngle() const {
        std::lock_guard<std::mutex> lock(state_mutex_);
        return current_angle_;
    }
    
    float getTargetSpeed() const {
        return target_speed_.load();
    }
    
    void setTargetSpeed(float speed) {
        target_speed_.store(speed);
    }
    
    float getTargetAngle() const {
        return target_angle_.load();
    }
    
    void setTargetAngle(float angle) {
        target_angle_.store(angle);
    }
    
    bool isRunning() const {
        return running_.load();
    }
    
    // PID 게인 설정
    void setBalancePIDGains(double kp, double ki, double kd) {
        std::lock_guard<std::mutex> lock(control_mutex_);
        pid_balance_.setGains(kp, ki, kd);
    }

private:
    // 하드웨어 인터페이스
    I2CController i2c_controller_;
    HardwareSPI* spi_;
    MCP4921* dac0_;
    MCP4921* dac1_;
    PIDController pid_balance_;
    
    // 제어 루프 스레드
    std::thread control_thread_;
    std::atomic<bool> running_;
    
    // 공유 상태 변수 (ROS Bridge와 공유)
    std::atomic<float> target_speed_;
    std::atomic<float> target_angle_;
    mutable std::mutex state_mutex_;
    float current_angle_;
    float current_gyro_rate_;
    
    // 제어 변수
    mutable std::mutex control_mutex_;
    float motor_output_;
    
    // 센서 보정값
    double accel_offset_[3];
    double gyro_offset_[3];
    
    /**
     * @brief 제어 루프 함수 (Real-Time Thread)
     */
    void controlLoop();
    
    /**
     * @brief 제어 로직 실행
     */
    void processControl();
    
    /**
     * @brief 센서 데이터 읽기
     */
    bool readSensorData();
    
    /**
     * @brief 각도 계산
     */
    void calculateAngle();
    
    /**
     * @brief 모터 출력 설정
     */
    void setMotorOutput(float output);
    
    /**
     * @brief Real-Time 스레드 설정
     */
    bool setRealTimePriority();
    
    // 상보 필터 변수
    double stable_angle_;
    double prev_stable_angle_;
    bool first_angle_calc_;
    
    // 센서 데이터
    SensorData sensor_data_;
    double accel_x_, accel_y_, accel_z_;
    double gyro_x_, gyro_y_, gyro_z_;
};

#endif // BALANCER_CORE_H




