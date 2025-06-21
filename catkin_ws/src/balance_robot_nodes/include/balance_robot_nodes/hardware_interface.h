#ifndef HARDWARE_INTERFACE_H
#define HARDWARE_INTERFACE_H

#include <string>
#include <cstdint>

/**
 * @brief 하드웨어 상수 정의
 */
namespace HardwareConstants {
    // 로봇 물리적 파라미터
    const double WHEEL_RADIUS = 0.142;      // 바퀴 반지름 (m)
    const double WHEEL_DISTANCE = 0.375;    // 바퀴 간 거리 (m)
    
    // 홀센서 관련
    const int HALL_STATES_PER_REVOLUTION = 21;  // 한 바퀴당 홀센서 상태 변화 수
    
    // SPI 통신 설정
    const int DEFAULT_SPI_SPEED = 100000000;    // 100MHz
    
    // 제어 루프 주기
    const double CONTROL_LOOP_PERIOD = 0.002;   // 2ms (500Hz)
    const double SENSOR_READ_PERIOD = 0.0001;   // 0.1ms (10kHz)
}

/**
 * @brief 데드존 보상 유틸리티 클래스
 */
class DeadzoneCompensator {
public:
    DeadzoneCompensator(double deadzone_threshold = 5.0, double compensation_value = 8.0);
    
    /**
     * @brief 데드존 보상 적용
     * @param speed 입력 속도 (%)
     * @return 보상된 출력 속도 (%)
     */
    double compensate(double speed);
    
    void setParameters(double deadzone_threshold, double compensation_value);
    
private:
    double deadzone_threshold_;    // 데드존 임계값 (%)
    double compensation_value_;    // 보상값 (%)
};

/**
 * @brief 방향 전환 제어 유틸리티 클래스
 */
class DirectionChangeController {
public:
    DirectionChangeController(double brake_threshold = 10.0, double brake_duration = 0.1);
    
    /**
     * @brief 방향 전환 제어 적용
     * @param current_output 현재 출력값
     * @param motor_id 모터 ID (0 또는 1)
     * @param dt 시간 간격 (초)
     * @return 제어된 출력값
     */
    double applyControl(double current_output, int motor_id, double dt);
    
    /**
     * @brief 제어기 리셋
     */
    void reset();
    
    void setParameters(double brake_threshold, double brake_duration);

private:
    double brake_threshold_;    // 브레이크 존 임계값 (%)
    double brake_duration_;     // 브레이크 지속 시간 (초)
    
    // 모터별 상태 (최대 2개 모터)
    double prev_output_[2];
    bool is_braking_[2];
    double brake_start_time_[2];
    
    /**
     * @brief 방향 전환 감지
     */
    bool isDirectionChange(double current_output, double prev_output);
};

/**
 * @brief GPIO 핀 설정 구조체
 */
struct GPIOPinConfig {
    // SPI 핀
    int mosi_pin = 16;
    int miso_pin = 17;
    int clk_pin = 18;
    int ss0_pin = 19;    // DAC0 CS
    int ss1_pin = 20;    // DAC1 CS
    int latch_pin = 38;  // 홀센서 래치
    
    // 모터 제어 핀
    int reverse0_pin = 168;
    int reverse1_pin = 13;
    
    // 버튼 핀
    int inc_button_pin = 232;
    int dec_button_pin = 15;
    
    // 기타 설정
    std::string gpio_chip_name = "gpiochip0";
    int spi_speed = HardwareConstants::DEFAULT_SPI_SPEED;
};

/**
 * @brief 에러 코드 정의
 */
enum class HardwareError {
    NONE = 0,
    SPI_INIT_FAILED = 1,
    GPIO_INIT_FAILED = 2,
    DAC_INIT_FAILED = 3,
    I2C_INIT_FAILED = 4,
    SENSOR_READ_FAILED = 5,
    COMMUNICATION_TIMEOUT = 6
};

/**
 * @brief 에러 코드를 문자열로 변환
 */
std::string hardwareErrorToString(HardwareError error);

#endif // HARDWARE_INTERFACE_H 