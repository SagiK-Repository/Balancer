#include "balance_robot_nodes/hardware_interface.h"
#include <algorithm>
#include <cmath>

// DeadzoneCompensator 구현
DeadzoneCompensator::DeadzoneCompensator(double deadzone_threshold, double compensation_value)
    : deadzone_threshold_(deadzone_threshold), compensation_value_(compensation_value) {
}

double DeadzoneCompensator::compensate(double speed) {
    if (speed == 0.0) {
        return 0.0;  // 완전 정지는 그대로
    }
    
    double abs_speed = std::abs(speed);
    double sign = (speed > 0) ? 1.0 : -1.0;
    
    // 작은 속도값에 대해 데드존 보상 적용
    if (abs_speed < deadzone_threshold_) {
        // 데드존 이하의 속도는 최소 동작 속도로 보상
        return sign * compensation_value_;
    } else {
        // 데드존 이상의 속도는 선형 스케일링
        // 입력: deadzone_threshold_ ~ 100%
        // 출력: compensation_value_ ~ 100%
        double scaled_speed = compensation_value_ + 
                             (abs_speed - deadzone_threshold_) * 
                             (100.0 - compensation_value_) / 
                             (100.0 - deadzone_threshold_);
        
        return sign * std::min(scaled_speed, 100.0);
    }
}

void DeadzoneCompensator::setParameters(double deadzone_threshold, double compensation_value) {
    deadzone_threshold_ = std::abs(deadzone_threshold);
    compensation_value_ = std::abs(compensation_value);
}

// DirectionChangeController 구현
DirectionChangeController::DirectionChangeController(double brake_threshold, double brake_duration)
    : brake_threshold_(brake_threshold), brake_duration_(brake_duration) {
    
    // 배열 초기화
    for (int i = 0; i < 2; i++) {
        prev_output_[i] = 0.0;
        is_braking_[i] = false;
        brake_start_time_[i] = 0.0;
    }
}

double DirectionChangeController::applyControl(double current_output, int motor_id, double dt) {
    if (motor_id < 0 || motor_id >= 2) {
        return current_output;  // 잘못된 모터 ID
    }
    
    static double accumulated_time[2] = {0.0, 0.0};
    accumulated_time[motor_id] += dt;
    
    // 방향 전환 감지
    if (!is_braking_[motor_id] && isDirectionChange(current_output, prev_output_[motor_id])) {
        // 브레이킹 시작
        is_braking_[motor_id] = true;
        brake_start_time_[motor_id] = 0.0;
        
        // 이전 출력값 업데이트
        prev_output_[motor_id] = current_output;
        
        return 0.0;  // 브레이크 시작 - 정지
    }
    
    // 브레이킹 중인 경우
    if (is_braking_[motor_id]) {
        brake_start_time_[motor_id] += dt;
        
        if (brake_start_time_[motor_id] < brake_duration_) {
            // 브레이킹 지속
            return 0.0;
        } else {
            // 브레이킹 종료
            is_braking_[motor_id] = false;
        }
    }
    
    // 이전 출력값 업데이트
    prev_output_[motor_id] = current_output;
    
    return current_output;  // 정상 출력
}

void DirectionChangeController::reset() {
    for (int i = 0; i < 2; i++) {
        prev_output_[i] = 0.0;
        is_braking_[i] = false;
        brake_start_time_[i] = 0.0;
    }
}

void DirectionChangeController::setParameters(double brake_threshold, double brake_duration) {
    brake_threshold_ = std::abs(brake_threshold);
    brake_duration_ = std::abs(brake_duration);
}

bool DirectionChangeController::isDirectionChange(double current_output, double prev_output) {
    // 부호가 다르고, 둘 다 브레이크 존 임계값 이하인 경우
    return (current_output * prev_output < 0) && 
           (std::abs(current_output) < brake_threshold_) && 
           (std::abs(prev_output) < brake_threshold_);
}

// 에러 코드를 문자열로 변환
std::string hardwareErrorToString(HardwareError error) {
    switch (error) {
        case HardwareError::NONE:
            return "없음";
        case HardwareError::SPI_INIT_FAILED:
            return "SPI 초기화 실패";
        case HardwareError::GPIO_INIT_FAILED:
            return "GPIO 초기화 실패";
        case HardwareError::DAC_INIT_FAILED:
            return "DAC 초기화 실패";
        case HardwareError::I2C_INIT_FAILED:
            return "I2C 초기화 실패";
        case HardwareError::I2C_READ_FAILED:
            return "I2C 읽기 실패";
        case HardwareError::SENSOR_READ_FAILED:
            return "센서 읽기 실패";
        case HardwareError::COMMUNICATION_TIMEOUT:
            return "통신 타임아웃";
        default:
            return "알 수 없는 오류";
    }
} 