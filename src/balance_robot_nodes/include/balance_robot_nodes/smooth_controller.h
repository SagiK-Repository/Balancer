#ifndef SMOOTH_CONTROLLER_H
#define SMOOTH_CONTROLLER_H

#include <algorithm>
#include <cmath>

/**
 * @brief 부드러운 모터 제어 클래스
 * 
 * 급격한 속도 변화를 방지하여 부드러운 모터 제어를 제공합니다.
 * 가속도 제한, 감속도 제한, 필터링 기능을 포함합니다.
 */
class SmoothMotorController {
public:
    /**
     * @brief 생성자
     */
    SmoothMotorController();
    
    /**
     * @brief 제어 파라미터 설정
     * @param max_acceleration 최대 가속도 (%/초)
     * @param max_deceleration 최대 감속도 (%/초)
     * @param velocity_filter_coeff 속도 필터 계수 (0~1)
     * @param output_filter_coeff 출력 필터 계수 (0~1)
     */
    void setParameters(double max_acceleration, double max_deceleration,
                      double velocity_filter_coeff, double output_filter_coeff);
    
    /**
     * @brief 목표 속도 설정
     * @param target_velocity 목표 속도 (%)
     */
    void setTargetVelocity(double target_velocity);
    
    /**
     * @brief 제어 업데이트
     * @param dt 시간 간격 (초)
     * @return 부드럽게 제어된 출력값
     */
    double update(double dt);
    
    /**
     * @brief 제어 업데이트 (출력값 기반)
     * @param output 입력 출력값
     * @return 부드럽게 제어된 출력값
     */
    double updateWithOutput(double output);
    
    /**
     * @brief 제어기 리셋
     */
    void reset();
    
    // 상태 접근자
    double getCurrentVelocity() const { return current_velocity_; }
    double getFilteredOutput() const { return filtered_output_; }
    double getTargetVelocity() const { return target_velocity_; }

private:
    // 제어 파라미터
    double max_acceleration_;    // 최대 가속도 (%/초)
    double max_deceleration_;    // 최대 감속도 (%/초)
    double velocity_filter_coeff_;   // 속도 필터 계수
    double output_filter_coeff_;     // 출력 필터 계수
    
    // 상태 변수
    double target_velocity_;     // 목표 속도
    double current_velocity_;    // 현재 속도
    double filtered_output_;     // 필터링된 출력
    
    /**
     * @brief 값을 범위로 제한
     * @param value 제한할 값
     * @param min_val 최소값
     * @param max_val 최대값
     * @return 제한된 값
     */
    double clamp(double value, double min_val, double max_val);
    
    /**
     * @brief 저역 통과 필터 적용
     * @param input 입력값
     * @param prev_output 이전 출력값
     * @param filter_coeff 필터 계수
     * @return 필터링된 값
     */
    double lowPassFilter(double input, double prev_output, double filter_coeff);
};

#endif // SMOOTH_CONTROLLER_H 