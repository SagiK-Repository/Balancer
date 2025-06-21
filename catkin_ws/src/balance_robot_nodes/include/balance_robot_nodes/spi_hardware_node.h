#ifndef SPI_HARDWARE_NODE_H
#define SPI_HARDWARE_NODE_H

#include <ros/ros.h>
#include <std_msgs/Float32.h>
#include "balance_robot_nodes/MotorCommand.h"
#include "balance_robot_nodes/MotorStatus.h"
#include "balance_robot_nodes/hardware_interface.h"

// 임시 클래스 선언 (실제 하드웨어에서는 기존 motor_controller 파일 사용)
class BitBangSPI {
public:
    enum Mode { MODE0 = 0 };
    BitBangSPI() {}
    BitBangSPI(int clk_pin, int mosi_pin, int miso_pin, Mode mode, int speed, bool lsb_first) 
        : clk_pin_(clk_pin), mosi_pin_(mosi_pin), miso_pin_(miso_pin), 
          mode_(mode), speed_(speed), lsb_first_(lsb_first) {}
    void init() { /* 실제 구현에서는 기존 코드 사용 */ }
    void init(int mosi, int miso, int clk, Mode mode, int speed, bool lsb_first) { /* 실제 구현 */ }
    uint8_t transfer(uint8_t data) { return 0; /* 실제 구현에서는 기존 코드 사용 */ }
private:
    int clk_pin_, mosi_pin_, miso_pin_;
    Mode mode_;
    int speed_;
    bool lsb_first_;
};

class MCP4921 {
public:
    MCP4921(int channel) : channel_(channel) {}
    MCP4921(BitBangSPI* spi, int cs_pin) : channel_(cs_pin), spi_(spi) {}
    void setOutputPercent(float percent) { /* 실제 구현에서는 기존 코드 사용 */ }
    bool isInitialized() { return true; /* 임시로 항상 true */ }
private:
    int channel_;
    BitBangSPI* spi_;
};

class HallSensorEncoder {
public:
    HallSensorEncoder() {}
    HallSensorEncoder(double wheel_radius, double wheel_distance, int hall_states) 
        : wheel_radius_(wheel_radius), wheel_distance_(wheel_distance), hall_states_(hall_states) {}
    void initialize() { /* 실제 구현에서는 기존 코드 사용 */ }
    void updateHallSensors(uint8_t state0, uint8_t state1) { /* 실제 구현에서는 기존 코드 사용 */ }
    double getWheel0Velocity() { return 0.0; /* 실제 구현에서는 기존 코드 사용 */ }
    double getWheel1Velocity() { return 0.0; /* 실제 구현에서는 기존 코드 사용 */ }
private:
    double wheel_radius_;
    double wheel_distance_;
    int hall_states_;
};

#include <gpiod.h>
#include <string>

/**
 * @brief SPI 하드웨어 제어 노드
 * 
 * 담당 기능:
 * - DAC를 통한 모터 출력 제어
 * - SPI 통신을 통한 홀센서 데이터 읽기
 * - GPIO를 통한 모터 방향 제어
 * - 버튼 입력 처리
 */
class SPIHardwareNode {
public:
    /**
     * @brief 생성자
     * @param nh ROS 노드 핸들
     * @param pnh 프라이빗 노드 핸들
     */
    SPIHardwareNode(ros::NodeHandle& nh, ros::NodeHandle& pnh);
    
    /**
     * @brief 소멸자
     */
    ~SPIHardwareNode();
    
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
    ros::Subscriber motor_command_sub_;
    
    // 발행자
    ros::Publisher motor_status_pub_;
    ros::Publisher motor0_speed_pub_;  // 호환성을 위한 개별 토픽
    ros::Publisher motor1_speed_pub_;
    
    // 타이머
    ros::Timer control_timer_;
    ros::Timer sensor_timer_;
    ros::Timer button_timer_;
    
    // 하드웨어 인터페이스
    GPIOPinConfig pin_config_;
    MCP4921* dac0_;
    MCP4921* dac1_;
    BitBangSPI spi_;
    HallSensorEncoder encoder_;
    
    // GPIO 관련
    struct gpiod_chip* chip_;
    struct gpiod_line* reverse0_line_;
    struct gpiod_line* reverse1_line_;
    struct gpiod_line* inc_button_line_;
    struct gpiod_line* dec_button_line_;
    struct gpiod_line* latch_line_;
    
    // 상태 변수
    balance_robot_nodes::MotorCommand current_command_;
    balance_robot_nodes::MotorStatus current_status_;
    
    // 홀센서 상태
    uint8_t hall_state_0_;
    uint8_t hall_state_1_;
    int32_t hall_count_0_;
    int32_t hall_count_1_;
    
    // 모터 속도 (RPM)
    float motor0_speed_;
    float motor1_speed_;
    
    // 버튼 상태
    bool prev_inc_button_state_;
    bool prev_dec_button_state_;
    ros::Time last_button_time_;
    
    // 하드웨어 상태
    bool hardware_initialized_;
    HardwareError last_error_;
    
    // 콜백 함수들
    /**
     * @brief 모터 명령 콜백
     */
    void motorCommandCallback(const balance_robot_nodes::MotorCommand::ConstPtr& msg);
    
    /**
     * @brief 제어 루프 타이머 콜백
     */
    void controlTimerCallback(const ros::TimerEvent& event);
    
    /**
     * @brief 센서 읽기 타이머 콜백
     */
    void sensorTimerCallback(const ros::TimerEvent& event);
    
    /**
     * @brief 버튼 체크 타이머 콜백
     */
    void buttonTimerCallback(const ros::TimerEvent& event);
    
    // 하드웨어 제어 함수들
    /**
     * @brief GPIO 초기화
     */
    bool initializeGPIO();
    
    /**
     * @brief SPI 초기화
     */
    bool initializeSPI();
    
    /**
     * @brief DAC 초기화
     */
    bool initializeDAC();
    
    /**
     * @brief GPIO 정리
     */
    void cleanupGPIO();
    
    /**
     * @brief GPIO 라인 설정 (출력)
     */
    bool setOutputLine(int pin, struct gpiod_line** pline);
    
    /**
     * @brief GPIO 라인 설정 (입력)
     */
    bool setInputLine(int pin, struct gpiod_line** pline);
    
    /**
     * @brief 모터 제어 적용
     */
    void applyMotorCommand();
    
    /**
     * @brief 모터 방향 설정
     */
    void setMotorDirection(int motor_id, bool forward);
    
    /**
     * @brief 모터 속도 설정
     */
    void setMotorSpeed(int motor_id, float speed_percent);
    
    /**
     * @brief 홀센서 상태 읽기
     */
    void readHallSensors();
    
    /**
     * @brief 모터 속도 계산
     */
    void calculateMotorSpeeds();
    
    /**
     * @brief 버튼 상태 확인
     */
    void checkButtons();
    
    /**
     * @brief 모터 상태 발행
     */
    void publishMotorStatus();
    
    /**
     * @brief 매개변수 로드
     */
    void loadParameters();
    
    /**
     * @brief 에러 상태 설정
     */
    void setError(HardwareError error, const std::string& message = "");
};

#endif // SPI_HARDWARE_NODE_H 