#ifndef SPI_HARDWARE_NODE_H
#define SPI_HARDWARE_NODE_H

#include <ros/ros.h>
#include <std_msgs/Float32.h>
#include "balance_robot_nodes/MotorCommand.h"
#include "balance_robot_nodes/MotorStatus.h"
#include "balance_robot_nodes/hardware_interface.h"

// 실제 BitBangSPI 구현 (motor_controller에서 가져옴)
#include <gpiod.h>
#include <chrono>
#include <thread>

class BitBangSPI {
public:
    enum SPIMode { MODE0 = 0, MODE1 = 1, MODE2 = 2, MODE3 = 3 };
    
    BitBangSPI(int mosi_pin, int miso_pin, int sclk_pin, int cs_pin,
               SPIMode mode, uint32_t max_speed_hz, bool bit_order_lsb);
    ~BitBangSPI();
    
    void initGPIO();
    void cleanupGPIO();
    void delayMicroseconds(unsigned int usec);
    uint8_t transfer(uint8_t data_out);
    uint16_t transfer16(uint16_t data_out);
    void setMode(SPIMode mode);
    void setMaxSpeed(uint32_t speed_hz);
    void setCS(bool level);

private:
    int mosi_pin_, miso_pin_, sclk_pin_, cs_pin_;
    SPIMode mode_;
    uint32_t max_speed_hz_;
    bool bit_order_lsb_;
    bool cpol_, cpha_;
    bool gpio_initialized_;
    
    struct gpiod_chip* chip_;
    struct gpiod_line* mosi_line_;
    struct gpiod_line* miso_line_;
    struct gpiod_line* sclk_line_;
    struct gpiod_line* cs_line_;
};

class MCP4921 {
public:
    MCP4921(BitBangSPI* spi, int cs_pin);
    ~MCP4921();
    
    bool isInitialized() const;
    bool setOutput(uint16_t value, bool buffered = true, bool gain_1x = true, bool active = true);
    bool setOutputPercent(float percent, bool buffered = true, bool gain_1x = true);
    bool setOutputVoltage(float voltage, float vref = 3.3f, bool buffered = true, bool gain_1x = true);

private:
    BitBangSPI* spi_;
    int cs_pin_;
    bool initialized_;
    
    struct gpiod_chip* chip_;
    struct gpiod_line* cs_line_;
};

class HallSensorEncoder {
public:
    struct WheelInfo {
        long encoder_count = 0;
        double position = 0.0;
        double velocity = 0.0;
        int current_hall_state = -1;
        bool direction_forward = true;
        bool is_valid = false;
    };
    
    struct RobotOdometry {
        double x = 0.0;
        double y = 0.0;
        double theta = 0.0;
        double linear_velocity = 0.0;
        double angular_velocity = 0.0;
    };
    
    HallSensorEncoder(double wheel_radius, double wheel_base, double max_velocity = 10.0);
    ~HallSensorEncoder();
    
    bool initialize();
    void reset();
    void updateHallSensors(int wheel0_hall, int wheel1_hall);
    void updateHallSensors(bool wheel0_bit0, bool wheel0_bit1, bool wheel0_bit2,
                          bool wheel1_bit0, bool wheel1_bit1, bool wheel1_bit2);
    
    // 접근자
    const WheelInfo& getWheel0Info() const { return wheel0_info_; }
    const WheelInfo& getWheel1Info() const { return wheel1_info_; }
    const RobotOdometry& getRobotOdometry() const { return robot_odom_; }
    
    double getWheel0Velocity() const { return wheel0_info_.velocity; }
    double getWheel1Velocity() const { return wheel1_info_.velocity; }
    double getWheel0Position() const { return wheel0_info_.position; }
    double getWheel1Position() const { return wheel1_info_.position; }

private:
    struct WheelState {
        int sequence_index = 0;
        int last_hall_state = -1;
        std::chrono::high_resolution_clock::time_point last_update_time;
        std::chrono::high_resolution_clock::time_point last_velocity_time;
        long last_encoder_count = 0;
    };
    
    // 하드웨어 파라미터
    double wheel_radius_;
    double wheel_base_;
    int steps_per_revolution_;
    double max_velocity_;
    double velocity_timeout_;
    
    // 상태
    WheelInfo wheel0_info_;
    WheelInfo wheel1_info_;
    WheelState wheel0_state_;
    WheelState wheel1_state_;
    RobotOdometry robot_odom_;
    std::chrono::high_resolution_clock::time_point last_odom_time_;
    
    // 홀센서 시퀀스
    std::vector<int> wheel0_sequence_;
    std::vector<int> wheel1_sequence_;
    std::vector<int> wheel0_sequence_inverse;
    std::vector<int> wheel1_sequence_inverse;
    
    void initHallSequences();
    int findSequenceIndex(const std::vector<int>& inverse, int hall_value);
    int getNextSequenceIndex(const std::vector<int>& sequence, int current_index);
    int getPrevSequenceIndex(const std::vector<int>& sequence, int current_index);
    bool updateWheelState(WheelInfo& wheel_info, WheelState& wheel_state,
                         const std::vector<int>& sequence, const std::vector<int>& inverse, int new_hall_state);
    void calculatePosition(WheelInfo& wheel_info, const WheelState& wheel_state);
    void calculateVelocity(WheelInfo& wheel_info, WheelState& wheel_state);
    void updateRobotOdometry();
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