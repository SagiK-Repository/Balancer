#ifndef I2C_SENSOR_NODE_H
#define I2C_SENSOR_NODE_H

#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/Temperature.h>
#include "balance_robot_nodes/SensorData.h"
#include "balance_robot_nodes/hardware_interface.h"

// 임시 I2C 컨트롤러 클래스 선언 (실제 하드웨어에서는 기존 motor_controller 파일 사용)
class I2CController {
public:
    I2CController() {}
    void init() { /* 실제 구현에서는 기존 코드 사용 */ }
    bool getData(double* accel_data, double* gyro_data) { 
        // 임시 더미 데이터
        for(int i = 0; i < 3; i++) {
            accel_data[i] = 0.0;
            gyro_data[i] = 0.0;
        }
        return true; 
    }
};

#include <string>

/**
 * @brief I2C 센서 제어 노드
 * 
 * 담당 기능:
 * - IMU 센서 데이터 읽기 (가속도, 자이로, 자력계)
 * - 센서 데이터 필터링 및 안정화
 * - Roll/Pitch/Yaw 각도 계산
 * - 온도 데이터 읽기
 * - 센서 상태 모니터링
 */
class I2CSensorNode {
public:
    /**
     * @brief 생성자
     * @param nh ROS 노드 핸들
     * @param pnh 프라이빗 노드 핸들
     */
    I2CSensorNode(ros::NodeHandle& nh, ros::NodeHandle& pnh);
    
    /**
     * @brief 소멸자
     */
    ~I2CSensorNode();
    
    /**
     * @brief 노드 초기화
     * @return 초기화 성공 여부
     */
    bool initialize();

private:
    // ROS 관련
    ros::NodeHandle& nh_;
    ros::NodeHandle& pnh_;
    
    // 발행자
    ros::Publisher sensor_data_pub_;        // 커스텀 센서 데이터
    ros::Publisher imu_pub_;               // 표준 IMU 메시지
    ros::Publisher temperature_pub_;       // 표준 온도 메시지
    ros::Publisher roll_pub_;              // 개별 각도 토픽 (호환성)
    ros::Publisher pitch_pub_;
    ros::Publisher confidence_pub_;
    
    // 타이머
    ros::Timer sensor_timer_;
    
    // I2C 컨트롤러
    I2CController i2c_controller_;
    
    // 센서 데이터
    balance_robot_nodes::SensorData current_sensor_data_;
    sensor_msgs::Imu imu_msg_;
    sensor_msgs::Temperature temp_msg_;
    
    // 센서 상태
    bool sensor_initialized_;
    bool sensor_calibrated_;
    HardwareError last_error_;
    
    // 필터링 및 안정화 변수
    double sensor_data_buffer_[7];  // 원시 센서 데이터 버퍼
    bool filter_enabled_;
    double filter_alpha_;           // 저역 통과 필터 계수
    
    // 각도 계산 관련
    bool use_complementary_filter_;
    double complementary_alpha_;
    
    // 안정성 판단 관련
    double stability_threshold_;    // 안정성 판단 임계값
    double confidence_threshold_;   // 신뢰도 임계값
    
    // 센서 보정 관련
    double accel_offset_[3];       // 가속도계 오프셋
    double gyro_offset_[3];        // 자이로스코프 오프셋
    double mag_offset_[3];         // 자력계 오프셋
    
    // 콜백 함수들
    /**
     * @brief 센서 읽기 타이머 콜백
     */
    void sensorTimerCallback(const ros::TimerEvent& event);
    
    // 센서 관련 함수들
    /**
     * @brief I2C 센서 초기화
     */
    bool initializeSensor();
    
    /**
     * @brief 센서 데이터 읽기
     */
    bool readSensorData();
    
    /**
     * @brief 센서 데이터 필터링
     */
    void filterSensorData();
    
    /**
     * @brief Roll/Pitch/Yaw 각도 계산
     */
    void calculateAngles();
    
    /**
     * @brief 상보 필터 적용
     */
    void applyComplementaryFilter(double& angle, double accel_angle, double gyro_rate, double dt);
    
    /**
     * @brief 센서 안정성 평가
     */
    void evaluateStability();
    
    /**
     * @brief 신뢰도 계산
     */
    void calculateConfidence();
    
    /**
     * @brief 센서 보정
     */
    void calibrateSensor();
    
    /**
     * @brief 센서 데이터 발행
     */
    void publishSensorData();
    
    /**
     * @brief 표준 IMU 메시지 발행
     */
    void publishIMUMessage();
    
    /**
     * @brief 개별 토픽 발행 (호환성)
     */
    void publishIndividualTopics();
    
    /**
     * @brief 매개변수 로드
     */
    void loadParameters();
    
    /**
     * @brief 에러 상태 설정
     */
    void setError(HardwareError error, const std::string& message = "");
    
    /**
     * @brief 센서 자가 진단
     */
    bool performSelfTest();
    
    /**
     * @brief 온도 보상 적용
     */
    void applyTemperatureCompensation();
    
    // 유틸리티 함수들
    /**
     * @brief 각도를 -180~180도 범위로 정규화
     */
    double normalizeAngle(double angle);
    
    /**
     * @brief 저역 통과 필터 적용
     */
    double lowPassFilter(double input, double prev_output, double alpha);
    
    /**
     * @brief 벡터 크기 계산
     */
    double vectorMagnitude(double x, double y, double z);
};

#endif // I2C_SENSOR_NODE_H 