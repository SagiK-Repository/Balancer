#include "balance_robot_nodes/smooth_controller.h"
#include <algorithm>
#include <cmath>

SmoothMotorController::SmoothMotorController()
    : max_acceleration_(30.0),      // 기본 30%/초
      max_deceleration_(60.0),      // 기본 60%/초
      velocity_filter_coeff_(0.4),  // 기본 속도 필터 계수
      output_filter_coeff_(0.6),    // 기본 출력 필터 계수
      target_velocity_(0.0),
      current_velocity_(0.0),
      filtered_output_(0.0) {
}

void SmoothMotorController::setParameters(double max_acceleration, double max_deceleration,
                                         double velocity_filter_coeff, double output_filter_coeff) {
    max_acceleration_ = std::abs(max_acceleration);
    max_deceleration_ = std::abs(max_deceleration);
    velocity_filter_coeff_ = clamp(velocity_filter_coeff, 0.0, 1.0);
    output_filter_coeff_ = clamp(output_filter_coeff, 0.0, 1.0);
}

void SmoothMotorController::setTargetVelocity(double target_velocity) {
    target_velocity_ = target_velocity;
}

double SmoothMotorController::update(double dt) {
    if (dt <= 0.0) {
        return filtered_output_;
    }
    
    // 목표 속도와 현재 속도의 차이 계산
    double velocity_error = target_velocity_ - current_velocity_;
    
    // 가속도/감속도 제한 적용
    double max_change;
    if (std::abs(velocity_error) < 0.1) {
        // 목표에 가까우면 더 부드럽게
        max_change = std::min(max_acceleration_, max_deceleration_) * dt * 0.5;
    } else if ((velocity_error > 0 && current_velocity_ >= 0) || 
               (velocity_error < 0 && current_velocity_ <= 0)) {
        // 같은 방향으로 가속
        max_change = max_acceleration_ * dt;
    } else {
        // 반대 방향 또는 감속
        max_change = max_deceleration_ * dt;
    }
    
    // 속도 변화량 제한
    double velocity_change = clamp(velocity_error, -max_change, max_change);
    
    // 현재 속도 업데이트
    current_velocity_ += velocity_change;
    
    // 속도 필터링 적용
    double filtered_velocity = lowPassFilter(current_velocity_, filtered_output_, velocity_filter_coeff_);
    
    // 출력 필터링 적용
    filtered_output_ = lowPassFilter(filtered_velocity, filtered_output_, output_filter_coeff_);
    
    return filtered_output_;
}

void SmoothMotorController::reset() {
    target_velocity_ = 0.0;
    current_velocity_ = 0.0;
    filtered_output_ = 0.0;
}

double SmoothMotorController::clamp(double value, double min_val, double max_val) {
    return std::max(min_val, std::min(value, max_val));
}

double SmoothMotorController::updateWithOutput(double output) {
    // 출력값 기반 업데이트 - 간단한 저역 통과 필터 적용
    filtered_output_ = lowPassFilter(output, filtered_output_, output_filter_coeff_);
    return filtered_output_;
}

double SmoothMotorController::lowPassFilter(double input, double prev_output, double filter_coeff) {
    return filter_coeff * input + (1.0 - filter_coeff) * prev_output;
} 