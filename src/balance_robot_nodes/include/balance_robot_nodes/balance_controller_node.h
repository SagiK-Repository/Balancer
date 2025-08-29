#ifndef BALANCE_CONTROLLER_NODE_H
#define BALANCE_CONTROLLER_NODE_H

#include <ros/ros.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Bool.h>
#include <geometry_msgs/Twist.h>

#include "balance_robot_nodes/MotorCommand.h"
#include "balance_robot_nodes/MotorStatus.h"
#include "balance_robot_nodes/SensorData.h"
#include "balance_robot_nodes/ControllerStatus.h"
#include "balance_robot_nodes/pid_controller.h"
#include "balance_robot_nodes/smooth_controller.h"
#include "balance_robot_nodes/hardware_interface.h"

#include <string>

/**
 * @brief 밸런스 컨트롤러 노드
 * 
 * 담당 기능:
 * - PID 제어 (속도 제어, 밸런싱 제어)
 * - 부드러운 모터 제어
 * - 데드존 보상
 * - 방향 전환 제어
 * - 사용자 명령 처리
 * - 시스템 상태 모니터링
 */
class BalanceControllerNode {
public:
    /**
     * @brief 생성자
     * @param nh ROS 노드 핸들
     * @param pnh 프라이빗 노드 핸들
     */
    BalanceControllerNode(ros::NodeHandle& nh, ros::NodeHandle& pnh);
    
    /**
     * @brief 소멸자
     */
    ~BalanceControllerNode();
    
    /**
     * @brief 노드 초기화
     * @return 초기화 성공 여부
     */
    bool initialize();

private:
    // ROS 관련
    ros::NodeHandle& nh_;
    ros::NodeHandle& pnh_;
    
    // 구독자
    ros::Subscriber motor_status_sub_;
    ros::Subscriber sensor_data_sub_;
    ros::Subscriber target_speed0_sub_;     // 호환성을 위한 개별 토픽
    ros::Subscriber target_speed1_sub_;
    ros::Subscriber cmd_vel_sub_;           // 표준 속도 명령
    ros::Subscriber emergency_stop_sub_;
    
    // 발행자
    ros::Publisher motor_command_pub_;
    ros::Publisher controller_status_pub_;
    ros::Publisher system_status_pub_;      // 시스템 전체 상태
    
    // 타이머
    ros::Timer control_timer_;
    ros::Timer status_timer_;
    
    // 제어기들
    PIDController pid_motor0_;
    PIDController pid_motor1_;
    PIDController pid_balance_;
    SmoothMotorController smooth_motor0_;
    SmoothMotorController smooth_motor1_;
    DeadzoneCompensator deadzone_compensator_;
    DirectionChangeController direction_controller_;
    
    // 현재 상태
    balance_robot_nodes::MotorStatus current_motor_status_;
    balance_robot_nodes::SensorData current_sensor_data_;
    balance_robot_nodes::MotorCommand current_motor_command_;
    balance_robot_nodes::ControllerStatus controller_status_;
    
    // 제어 변수
    double target_speed_0_;
    double target_speed_1_;
    double actual_speed_0_;
    double actual_speed_1_;
    
    double pid_output_0_;
    double pid_output_1_;
    double balance_output_;
    
    double target_roll_angle_;
    double current_roll_angle_;
    
    // 제어 모드
    enum ControlMode {
        DIRECT_MODE = 0,
        PID_MODE = 1,
        BALANCE_MODE = 2
    };
    ControlMode current_control_mode_;
    
    // 제어 플래그
    bool use_pid_control_;
    bool use_balance_control_;
    bool use_smooth_control_;
    bool emergency_stop_active_;
    bool system_ready_;
    
    // 상태 플래그
    bool motor_status_received_;
    bool sensor_data_received_;
    ros::Time last_motor_status_time_;
    ros::Time last_sensor_data_time_;
    ros::Time last_control_time_;
    
    // 제어 파라미터
    double max_speed_;
    double control_frequency_;
    double status_frequency_;
    
    // 타임아웃 설정
    double motor_status_timeout_;
    double sensor_data_timeout_;
    
    // 콜백 함수들
    /**
     * @brief 모터 상태 콜백
     */
    void motorStatusCallback(const balance_robot_nodes::MotorStatus::ConstPtr& msg);
    
    /**
     * @brief 센서 데이터 콜백
     */
    void sensorDataCallback(const balance_robot_nodes::SensorData::ConstPtr& msg);
    
    /**
     * @brief 목표 속도 콜백 (개별)
     */
    void targetSpeed0Callback(const std_msgs::Float32::ConstPtr& msg);
    void targetSpeed1Callback(const std_msgs::Float32::ConstPtr& msg);
    
    /**
     * @brief cmd_vel 콜백 (표준)
     */
    void cmdVelCallback(const geometry_msgs::Twist::ConstPtr& msg);
    
    /**
     * @brief 비상 정지 콜백
     */
    void emergencyStopCallback(const std_msgs::Bool::ConstPtr& msg);
    
    /**
     * @brief 제어 루프 타이머 콜백
     */
    void controlTimerCallback(const ros::TimerEvent& event);
    
    /**
     * @brief 상태 발행 타이머 콜백
     */
    void statusTimerCallback(const ros::TimerEvent& event);
    
    // 제어 함수들
    /**
     * @brief 직접 제어 모드
     */
    void executeDirectControl();
    
    /**
     * @brief PID 속도 제어 모드
     */
    void executePIDControl();
    
    /**
     * @brief 밸런싱 제어 모드
     */
    void executeBalanceControl();
    
    /**
     * @brief PID 제어 업데이트
     */
    void updatePIDControl();
    
    /**
     * @brief 밸런싱 제어 업데이트
     */
    void updateBalanceControl();
    
    /**
     * @brief 부드러운 제어 적용
     */
    void applySmoothControl();
    
    /**
     * @brief 모터 명령 발행
     */
    void publishMotorCommand();
    
    /**
     * @brief 컨트롤러 상태 발행
     */
    void publishControllerStatus();
    
    /**
     * @brief 시스템 상태 확인
     */
    void checkSystemStatus();
    
    /**
     * @brief PID 제어기 리셋
     */
    void resetPIDControllers();
    
    /**
     * @brief 제어 모드 변경
     */
    void changeControlMode(ControlMode new_mode);
    
    /**
     * @brief 비상 정지 처리
     */
    void handleEmergencyStop();
    
    /**
     * @brief 안전 체크
     */
    bool performSafetyCheck();
    
    // 유틸리티 함수들
    /**
     * @brief 매개변수 로드
     */
    void loadParameters();
    
    /**
     * @brief PID 게인 로드
     */
    void loadPIDGains();
    
    /**
     * @brief 부드러운 제어 파라미터 로드
     */
    void loadSmoothControlParameters();
    
    /**
     * @brief 데드존 보상 파라미터 로드
     */
    void loadDeadzoneParameters();
    
    /**
     * @brief 방향 제어 파라미터 로드
     */
    void loadDirectionControlParameters();
    
    /**
     * @brief cmd_vel을 개별 바퀴 속도로 변환
     */
    void convertCmdVelToWheelSpeeds(const geometry_msgs::Twist& cmd_vel, 
                                    double& left_speed, double& right_speed);
    
    /**
     * @brief 속도 제한 적용
     */
    double clampSpeed(double speed);
    
    /**
     * @brief 시스템 진단
     */
    void performSystemDiagnostics();
    
    /**
     * @brief 로그 출력
     */
    void printControlStatus();
    
    /**
     * @brief 제어 모드를 문자열로 변환
     */
    std::string getControlModeString() const;
};

#endif // BALANCE_CONTROLLER_NODE_H 