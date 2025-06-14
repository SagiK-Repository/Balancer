#ifndef SMOOTH_MOTOR_CONTROLLER_H
#define SMOOTH_MOTOR_CONTROLLER_H

#include <ros/ros.h>
#include <vector>

/**
 * @brief 부드러운 모터 제어를 위한 클래스
 * 
 * 이 클래스는 다음 기능들을 제공합니다:
 * - 가속도 제한 (Acceleration Limiting)
 * - 속도 램프 (Velocity Ramping) 
 * - 저역통과 필터링 (Low-pass Filtering)
 * - S-Curve 가속도 프로파일
 * - 진동 억제 필터
 * - 긴급 정지 기능
 */
class SmoothMotorController {
public:
    /**
     * @brief 생성자
     */
    SmoothMotorController();
    
    /**
     * @brief 제어 매개변수 설정
     * @param max_accel 최대 가속도 (%/초)
     * @param max_decel 최대 감속도 (%/초)
     * @param vel_filter 속도 필터 계수 (0.1~1.0)
     * @param out_filter 출력 필터 계수 (0.1~1.0)
     */
    void setParameters(double max_accel, double max_decel, 
                      double vel_filter, double out_filter);
    
    /**
     * @brief 목표 속도 설정
     * @param target 목표 속도 (-100 ~ +100%)
     */
    void setTargetVelocity(double target);
    
    /**
     * @brief 제어 업데이트 (주기적 호출)
     * @param dt 시간 간격 (초)
     * @return 필터링된 출력값 (-100 ~ +100%)
     */
    double update(double dt);
    
    /**
     * @brief 긴급 정지 활성화
     */
    void emergencyStop();
    
    /**
     * @brief 긴급 정지 해제
     */
    void resume();
    
    /**
     * @brief 컨트롤러 리셋
     */
    void reset();
    
    /**
     * @brief 가속도 램프 활성화/비활성화
     * @param enable true: 활성화, false: 비활성화
     */
    void enableRamp(bool enable);
    
    /**
     * @brief 필터링 활성화/비활성화
     * @param enable true: 활성화, false: 비활성화
     */
    void enableFilter(bool enable);
    
    /**
     * @brief 상태 정보 출력
     */
    void printStatus() const;
    
    // 접근자 함수들
    double getCurrentVelocity() const { return current_velocity_; }
    double getFilteredVelocity() const { return filtered_velocity_; }
    double getCurrentOutput() const { return current_output_; }
    double getFilteredOutput() const { return filtered_output_; }
    double getTargetVelocity() const { return target_velocity_; }
    bool isEmergencyStop() const { return emergency_stop_; }
    bool isRampEnabled() const { return ramp_enabled_; }
    bool isFilterEnabled() const { return filter_enabled_; }

private:
    // 제어 매개변수
    double max_acceleration_;      // 최대 가속도 (%/초)
    double max_deceleration_;      // 최대 감속도 (%/초)
    double velocity_filter_alpha_; // 속도 필터 계수
    double output_filter_alpha_;   // 출력 필터 계수
    
    // 상태 변수
    double current_velocity_;      // 현재 속도
    double target_velocity_;       // 목표 속도
    double filtered_velocity_;     // 필터링된 속도
    double current_output_;        // 현재 출력
    double filtered_output_;       // 필터링된 출력
    double last_update_time_;      // 마지막 업데이트 시간
    
    // 제어 플래그
    bool emergency_stop_;          // 긴급 정지 상태
    bool ramp_enabled_;           // 가속도 램프 활성화
    bool filter_enabled_;         // 필터링 활성화
    
    /**
     * @brief 가속도 제한 적용
     * @param dt 시간 간격
     */
    void applyAccelerationLimits(double dt);
    
    /**
     * @brief 저역통과 필터 적용
     * @param input 입력값
     * @param prev_output 이전 출력값
     * @param alpha 필터 계수
     * @return 필터링된 출력값
     */
    double applyLowPassFilter(double input, double prev_output, double alpha);
    
    /**
     * @brief S-Curve 가속도 프로파일 적용 (더 부드러운 제어)
     * @param target 목표값
     * @param current 현재값
     * @param dt 시간 간격
     * @return 프로파일 적용된 값
     */
    double applySCurveProfile(double target, double current, double dt);
    
    /**
     * @brief 진동 억제 필터 (노치 필터)
     * @param input 입력값
     * @param frequency 억제할 주파수
     * @return 필터링된 출력값
     */
    double applyNotchFilter(double input, double frequency);
};

#endif // SMOOTH_MOTOR_CONTROLLER_H 