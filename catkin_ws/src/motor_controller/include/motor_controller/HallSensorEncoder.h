#ifndef HALL_SENSOR_ENCODER_H
#define HALL_SENSOR_ENCODER_H

#include <vector>
#include <chrono>
#include <cmath>
#include <string>
#include <bitset>     // std::bitset 사용을 위해 필수!

/**
 * @brief 3상 모터 홀 센서를 이용한 엔코더 클래스
 * 
 * 이 클래스는 3상 모터의 홀 센서 신호를 이용하여 바퀴의 위치와 속도를 계산합니다.
 * ROS에 독립적이며, 다른 프로젝트에서 쉽게 사용할 수 있습니다.
 */
class HallSensorEncoder {
public:
    /**
     * @brief 바퀴 정보 구조체
     */
    struct WheelInfo {
        double position;        // 바퀴 위치 (라디안)
        double velocity;        // 바퀴 속도 (라디안/초)
        long encoder_count;     // 엔코더 카운트
        bool direction_forward; // 회전 방향 (true: 정방향, false: 역방향)
        bool is_valid;          // 데이터 유효성
        int current_hall_state; // 현재 홀 센서 상태 (3비트)
        
        WheelInfo() : position(0.0), velocity(0.0), encoder_count(0), 
                     direction_forward(true), is_valid(false), current_hall_state(0) {}
    };

    /**
     * @brief 로봇 오도메트리 정보 구조체
     */
    struct RobotOdometry {
        double x;               // X 위치 (미터)
        double y;               // Y 위치 (미터)
        double theta;           // 방향각 (라디안)
        double linear_velocity; // 선속도 (미터/초)
        double angular_velocity;// 각속도 (라디안/초)
        
        RobotOdometry() : x(0.0), y(0.0), theta(0.0), linear_velocity(0.0), angular_velocity(0.0) {}
    };

private:
    // 홀 센서 시퀀스
    std::vector<int> wheel0_sequence_;
    std::vector<int> wheel0_sequence_inverse;
    std::vector<int> wheel1_sequence_;
    std::vector<int> wheel1_sequence_inverse;
    
    // 바퀴 상태
    WheelInfo wheel0_info_;
    WheelInfo wheel1_info_;
    
    // 내부 상태
    struct WheelState {
        int last_hall_state;
        int sequence_index;
        std::chrono::high_resolution_clock::time_point last_update_time;
        long last_encoder_count;
        std::chrono::high_resolution_clock::time_point last_velocity_time;
        
        WheelState() : last_hall_state(-1), sequence_index(0), last_encoder_count(0) {
            last_update_time = std::chrono::high_resolution_clock::now();
            last_velocity_time = last_update_time;
        }
    };
    
    WheelState wheel0_state_;
    WheelState wheel1_state_;
    
    // 로봇 오도메트리
    RobotOdometry robot_odom_;
    std::chrono::high_resolution_clock::time_point last_odom_time_;
    
    // 파라미터
    double wheel_radius_;           // 바퀴 반지름 (미터)
    double wheel_base_;             // 바퀴 간격 (미터)
    int steps_per_revolution_;      // 한 바퀴당 스텝 수
    double max_velocity_;           // 최대 속도 (라디안/초)
    double velocity_timeout_;       // 속도 계산 타임아웃 (초)
    
    // 내부 함수들
    void initHallSequences();
    int findSequenceIndex(const std::vector<int>& inverse, int hall_value);
    int getNextSequenceIndex(const std::vector<int>& sequence, int current_index);
    int getPrevSequenceIndex(const std::vector<int>& sequence, int current_index);
    bool updateWheelState(WheelInfo& wheel_info, WheelState& wheel_state, 
        const std::vector<int>& sequence, const std::vector<int>& inverse, int new_hall_state);
    void calculatePosition(WheelInfo& wheel_info, const WheelState& wheel_state);
    void calculateVelocity(WheelInfo& wheel_info, WheelState& wheel_state);
    void updateRobotOdometry();

public:
    /**
     * @brief 생성자
     * @param wheel_radius 바퀴 반지름 (미터, 기본값: 0.05m)
     * @param wheel_base 바퀴 간격 (미터, 기본값: 0.2m)
     * @param max_velocity 최대 허용 속도 (라디안/초, 기본값: 10.0)
     */
    HallSensorEncoder(double wheel_radius = 0.05, double wheel_base = 0.2, double max_velocity = 10.0);
    
    /**
     * @brief 소멸자
     */
    ~HallSensorEncoder();
    
    /**
     * @brief 엔코더 초기화
     * @return 초기화 성공 여부
     */
    bool initialize();
    
    /**
     * @brief 홀 센서 값 업데이트 (타이머 루프에서 호출)
     * @param wheel0_bit0 바퀴 0의 홀 센서 비트 0
     * @param wheel0_bit1 바퀴 0의 홀 센서 비트 1  
     * @param wheel0_bit2 바퀴 0의 홀 센서 비트 2
     * @param wheel1_bit0 바퀴 1의 홀 센서 비트 0
     * @param wheel1_bit1 바퀴 1의 홀 센서 비트 1
     * @param wheel1_bit2 바퀴 1의 홀 센서 비트 2
     */
    void updateHallSensors(bool wheel0_bit0, bool wheel0_bit1, bool wheel0_bit2,
                          bool wheel1_bit0, bool wheel1_bit1, bool wheel1_bit2);
    
    /**
     * @brief 홀 센서 값 업데이트 (3비트 정수 버전)
     * @param wheel0_hall 바퀴 0의 홀 센서 값 (0-7)
     * @param wheel1_hall 바퀴 1의 홀 센서 값 (0-7)
     */
    void updateHallSensors(int wheel0_hall, int wheel1_hall);
    
    /**
     * @brief 바퀴 0 정보 가져오기
     * @return 바퀴 0의 WheelInfo 구조체
     */
    const WheelInfo& getWheel0Info() const { return wheel0_info_; }
    
    /**
     * @brief 바퀴 1 정보 가져오기
     * @return 바퀴 1의 WheelInfo 구조체
     */
    const WheelInfo& getWheel1Info() const { return wheel1_info_; }
    
    /**
     * @brief 로봇 오도메트리 정보 가져오기
     * @return 로봇의 RobotOdometry 구조체
     */
    const RobotOdometry& getRobotOdometry() const { return robot_odom_; }
    
    /**
     * @brief 바퀴 0 위치 가져오기 (라디안)
     */
    double getWheel0Position() const { return wheel0_info_.position; }
    
    /**
     * @brief 바퀴 1 위치 가져오기 (라디안)
     */
    double getWheel1Position() const { return wheel1_info_.position; }
    
    /**
     * @brief 바퀴 0 속도 가져오기 (라디안/초)
     */
    double getWheel0Velocity() const { return wheel0_info_.velocity; }
    
    /**
     * @brief 바퀴 1 속도 가져오기 (라디안/초)
     */
    double getWheel1Velocity() const { return wheel1_info_.velocity; }
    
    /**
     * @brief 로봇 X 위치 가져오기 (미터)
     */
    double getRobotX() const { return robot_odom_.x; }
    
    /**
     * @brief 로봇 Y 위치 가져오기 (미터)
     */
    double getRobotY() const { return robot_odom_.y; }
    
    /**
     * @brief 로봇 방향각 가져오기 (라디안)
     */
    double getRobotTheta() const { return robot_odom_.theta; }
    
    /**
     * @brief 로봇 선속도 가져오기 (미터/초)
     */
    double getRobotLinearVelocity() const { return robot_odom_.linear_velocity; }
    
    /**
     * @brief 로봇 각속도 가져오기 (라디안/초)
     */
    double getRobotAngularVelocity() const { return robot_odom_.angular_velocity; }
    
    /**
     * @brief 엔코더 리셋
     */
    void reset();
    
    /**
     * @brief 파라미터 설정
     */
    void setWheelRadius(double radius) { wheel_radius_ = radius; }
    void setWheelBase(double base) { wheel_base_ = base; }
    void setMaxVelocity(double max_vel) { max_velocity_ = max_vel; }
    
    /**
     * @brief 파라미터 가져오기
     */
    double getWheelRadius() const { return wheel_radius_; }
    double getWheelBase() const { return wheel_base_; }
    int getStepsPerRevolution() const { return steps_per_revolution_; }
    
    /**
     * @brief 디버그 정보 출력용 문자열 생성
     */
    std::string getDebugString();
};

#endif // HALL_SENSOR_ENCODER_H