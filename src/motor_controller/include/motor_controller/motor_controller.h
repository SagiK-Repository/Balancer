// include/motor_controller/motor_controller.h
#ifndef MOTOR_CONTROLLER_H
#define MOTOR_CONTROLLER_H
#include <ros/ros.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Bool.h>
#include "motor_controller/mcp4921.h"
#include "motor_controller/i2c_controller.h"
#include "motor_controller/HallSensorEncoder.h"
#include <string>
#include <gpiod.h>

#define WHEEL_RADIUS 0.142
#define WHEEL_DISTANCE 0.375

// PID 컨트롤러 클래스
class PIDController {
public:
    PIDController(double kp = 1.0, double ki = 0.0, double kd = 0.0, 
                  double min_output = -100.0, double max_output = 100.0);
    
    void setGains(double kp, double ki, double kd);
    void setOutputLimits(double min_output, double max_output);
    void reset();
    double compute(double setpoint, double measured_value, double gyro_x, double dt);
    
    // PID 상태 접근자
    double getError() const { return error_; }
    double getIntegral() const { return integral_; }
    double getDerivative() const { return derivative_; }
    double getOutput() const { return output_; }

    // PID 게인
    double kp_, ki_, kd_;
private:
    
    // PID 상태 변수
    double error_;
    double prev_error_;
    double integral_;
    double derivative_;
    double output_;
    
    // 출력 제한
    double min_output_, max_output_;
    
    // 적분 와인드업 방지
    double integral_max_;
    
    bool first_run_;
};

class MotorController { public:
    // 생성자 및 소멸자
    MotorController(ros::NodeHandle& nh, ros::NodeHandle& pnh);
    ~MotorController();
    
    // 속도 명령 콜백
    void speed0Callback(const std_msgs::Float32::ConstPtr& msg);
    void speed1Callback(const std_msgs::Float32::ConstPtr& msg);
    
    // 타이머 콜백 - 주기적 실행
    void timerCallback(const ros::TimerEvent& event);
    
    // 홀센서 속도 감지 - 주기적 실행
    void checkHallSensor(const ros::TimerEvent& event);
    
    // 버튼 입력 처리
    void checkButtons();
    
    // 모터 제어 메서드
    void setMotor0Direction(bool forward);
    void setMotor0Speed(float speed_percent);
    void setMotor1Direction(bool forward);
    void setMotor1Speed(float speed_percent);
    void updateActualSpeed();
    
    // 밸런싱 제어 함수들
    void updateBalanceControl(double gyro_x);
    void applyBalanceOutput();
    
    // GPIO 초기화 및 정리
    bool setOutputLine(int pin,struct gpiod_line** pline);
    bool setInputLine(int pin,struct gpiod_line** pline);
    void setupGPIO();
    void cleanupGPIO();
    
    // ROS 관련 멤버
    ros::NodeHandle& nh_;
    ros::NodeHandle& pnh_;
    ros::Subscriber speed0_sub_;
    ros::Subscriber speed1_sub_;
    ros::Publisher actual_speed0_pub_;
    ros::Publisher aroll_pub_;
    ros::Publisher groll_pub_;

    ros::Publisher target_roll_pub_;
    ros::Publisher actual_roll_pub_;
    ros::Publisher balance_output_pub_;
    ros::Publisher balance_P_pub_;
    ros::Publisher balance_I_pub_;
    ros::Publisher balance_D_pub_;

    ros::Timer timer0_;
    ros::Timer timer1_;
    
    // 하드웨어 제어 멤버
    MCP4921* dac0_;
    MCP4921* dac1_;
    I2CController i2c;
    BitBangSPI spi;
    HallSensorEncoder encoder;

    // gpiod 관련 멤버
    struct gpiod_chip* chip_;
    struct gpiod_line* reverse0_line_;
    struct gpiod_line* reverse1_line_;
    struct gpiod_line* inc_button_line_;
    struct gpiod_line* dec_button_line_;
    
    // GPIO 핀 번호
    int mosi_pin_;
    int miso_pin_;
    int clk_pin_;
    int ss0_pin_;
    int ss1_pin_;
    int reverse0_pin_;
    int reverse1_pin_;
    //int hall_sensor_pin_;
    int inc_button_pin_;
    int dec_button_pin_;

    int latch_pin_;
    struct gpiod_chip *chip;
    struct gpiod_line* latch_line;
    
    // 상태 및 설정 멤버
    float target_speed_0;
    float actual_speed_0;

    float target_speed_1;
    float actual_speed_1;

    uint8_t hall_state_0;
    uint8_t hall_state_1;

    float max_speed_;
    bool prev_hall_state_;
    int hall_count_;
    ros::Time last_hall_time_;
    
    // 디바운싱을 위한 이전 버튼 상태
    bool prev_inc_button_state_;
    bool prev_dec_button_state_;
    ros::Time last_button_time_;
    
    // 설정 매개변수
    std::string spi_device_;
    int spi_speed_;
    std::string gpio_chip_name_; // 추가: gpiod 칩 이름

    // PID 컨트롤러
    PIDController pid_motor0_;
    PIDController pid_motor1_;
    PIDController pid_balance_;  // Roll 각도 밸런싱 PID
    
    // PID 제어 변수
    double pid_output_0_;  // PID 출력값 (-100 ~ 100)
    double pid_output_1_;
    double balance_output_; // 밸런싱 PID 출력값
    ros::Time last_pid_time_;
    
    // 밸런싱 제어 변수
    double target_roll_;    // 목표 Roll 각도 (보통 0.0)
    double current_roll_;   // 현재 Roll 각도
    
    // 실제 모터 출력값 (로그용)
    double actual_motor_output_0_;
    double actual_motor_output_1_;
    
    // 데드존 보상 변수
    double motor_deadzone_;     // 모터 최소 동작 속도 (%)
    double deadzone_compensation_;  // 데드존 보상값 (%)
    
    // 방향 전환 제어 변수
    double brake_zone_threshold_;   // 브레이크 존 임계값 (%)
    double brake_duration_;         // 브레이크 지속 시간 (초)
    ros::Time brake_start_time_0_;  // 모터 0 브레이크 시작 시간
    ros::Time brake_start_time_1_;  // 모터 1 브레이크 시작 시간
    double prev_output_0_;          // 이전 출력값 (방향 감지용)
    double prev_output_1_;
    bool is_braking_0_;             // 모터 0 브레이킹 상태
    bool is_braking_1_;             // 모터 1 브레이킹 상태
    
};
#endif // MOTOR_CONTROLLER_H
