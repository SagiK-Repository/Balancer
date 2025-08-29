#include "motor_controller/smooth_motor_controller.h"
#include <algorithm>
#include <cmath>

SmoothMotorController::SmoothMotorController() 
    : max_acceleration_(50.0),      // 50%/초 최대 가속도
      max_deceleration_(100.0),     // 100%/초 최대 감속도  
      velocity_filter_alpha_(0.3),  // 속도 필터 계수
      output_filter_alpha_(0.5),    // 출력 필터 계수
      current_velocity_(0.0),
      target_velocity_(0.0),
      filtered_velocity_(0.0),
      current_output_(0.0),
      filtered_output_(0.0),
      last_update_time_(0.0),
      emergency_stop_(false),
      ramp_enabled_(true),
      filter_enabled_(true) {
    
    ROS_INFO("SmoothMotorController initialized");
    ROS_INFO("Max acceleration: %.1f%%/s, Max deceleration: %.1f%%/s", 
             max_acceleration_, max_deceleration_);
}

void SmoothMotorController::setParameters(double max_accel, double max_decel, 
                                        double vel_filter, double out_filter) {
    max_acceleration_ = std::max(1.0, std::min(max_accel, 200.0));
    max_deceleration_ = std::max(1.0, std::min(max_decel, 500.0));
    velocity_filter_alpha_ = std::max(0.1, std::min(vel_filter, 1.0));
    output_filter_alpha_ = std::max(0.1, std::min(out_filter, 1.0));
    
    ROS_INFO("Smooth control parameters updated:");
    ROS_INFO("  Max accel/decel: %.1f/%.1f %%/s", max_acceleration_, max_deceleration_);
    ROS_INFO("  Filter alphas: vel=%.2f, out=%.2f", velocity_filter_alpha_, output_filter_alpha_);
}

void SmoothMotorController::setTargetVelocity(double target) {
    // 목표 속도 범위 제한
    target_velocity_ = std::max(-100.0, std::min(target, 100.0));
}

double SmoothMotorController::update(double dt) {
    if (dt <= 0.0 || dt > 1.0) {
        ROS_WARN("Invalid dt: %.6f, skipping update", dt);
        return filtered_output_;
    }
    
    // 1. 긴급 정지 처리
    if (emergency_stop_) {
        target_velocity_ = 0.0;
        // 긴급 정지 시 더 빠른 감속
        max_deceleration_ = 200.0;
    }
    
    // 2. 가속도 제한 적용 (램프 제어)
    if (ramp_enabled_) {
        applyAccelerationLimits(dt);
    } else {
        current_velocity_ = target_velocity_;
    }
    
    // 3. 속도 필터링
    if (filter_enabled_) {
        filtered_velocity_ = applyLowPassFilter(current_velocity_, filtered_velocity_, 
                                              velocity_filter_alpha_);
    } else {
        filtered_velocity_ = current_velocity_;
    }
    
    // 4. 출력 계산 (현재는 속도와 동일, 추후 PID 등 추가 가능)
    current_output_ = filtered_velocity_;
    
    // 5. 출력 필터링
    if (filter_enabled_) {
        filtered_output_ = applyLowPassFilter(current_output_, filtered_output_, 
                                            output_filter_alpha_);
    } else {
        filtered_output_ = current_output_;
    }
    
    // 6. 최종 출력 범위 제한
    filtered_output_ = std::max(-100.0, std::min(filtered_output_, 100.0));
    
    return filtered_output_;
}

void SmoothMotorController::applyAccelerationLimits(double dt) {
    double velocity_diff = target_velocity_ - current_velocity_;
    double max_change;
    
    if (std::abs(velocity_diff) < 0.1) {
        // 목표에 거의 도달한 경우
        current_velocity_ = target_velocity_;
        return;
    }
    
    if (velocity_diff > 0) {
        // 가속 중
        max_change = max_acceleration_ * dt;
    } else {
        // 감속 중 (방향 전환 포함)
        max_change = max_deceleration_ * dt;
    }
    
    // 변화량 제한 적용
    if (std::abs(velocity_diff) <= max_change) {
        current_velocity_ = target_velocity_;
    } else {
        current_velocity_ += (velocity_diff > 0) ? max_change : -max_change;
    }
}

double SmoothMotorController::applyLowPassFilter(double input, double prev_output, double alpha) {
    // 1차 저역통과 필터: output = α * input + (1-α) * prev_output
    return alpha * input + (1.0 - alpha) * prev_output;
}

void SmoothMotorController::emergencyStop() {
    emergency_stop_ = true;
    target_velocity_ = 0.0;
    ROS_WARN("Emergency stop activated!");
}

void SmoothMotorController::resume() {
    emergency_stop_ = false;
    ROS_INFO("Emergency stop released");
}

void SmoothMotorController::reset() {
    current_velocity_ = 0.0;
    target_velocity_ = 0.0;
    filtered_velocity_ = 0.0;
    current_output_ = 0.0;
    filtered_output_ = 0.0;
    emergency_stop_ = false;
    ROS_INFO("SmoothMotorController reset");
}

void SmoothMotorController::enableRamp(bool enable) {
    ramp_enabled_ = enable;
    ROS_INFO("Acceleration ramp %s", enable ? "ENABLED" : "DISABLED");
}

void SmoothMotorController::enableFilter(bool enable) {
    filter_enabled_ = enable;
    ROS_INFO("Output filtering %s", enable ? "ENABLED" : "DISABLED");
}

// S-Curve 가속도 프로파일 (더 부드러운 제어)
double SmoothMotorController::applySCurveProfile(double target, double current, double dt) {
    double diff = target - current;
    
    if (std::abs(diff) < 0.1) {
        return target;
    }
    
    // S-Curve 매개변수
    double max_vel = (diff > 0) ? max_acceleration_ : max_deceleration_;
    double accel_time = 0.5;  // 가속 구간 시간
    
    // 정규화된 시간 (0~1)
    static double profile_time = 0.0;
    profile_time += dt / accel_time;
    profile_time = std::min(profile_time, 1.0);
    
    // S-Curve 함수 (3차 다항식)
    double s_factor;
    if (profile_time < 0.5) {
        // 가속 구간
        double t = profile_time * 2.0;
        s_factor = 2.0 * t * t * t - 3.0 * t * t + t;
    } else {
        // 감속 구간  
        double t = (profile_time - 0.5) * 2.0;
        s_factor = 0.5 + 0.5 * (3.0 * t * t - 2.0 * t * t * t);
    }
    
    double change = diff * s_factor * dt * max_vel;
    
    // 목표에 도달했으면 프로파일 시간 리셋
    if (std::abs(target - (current + change)) < 0.1) {
        profile_time = 0.0;
    }
    
    return current + change;
}

// 진동 억제 필터 (노치 필터)
double SmoothMotorController::applyNotchFilter(double input, double frequency) {
    // 간단한 이동평균 기반 노치 필터
    static std::vector<double> buffer(5, 0.0);
    static int index = 0;
    
    buffer[index] = input;
    index = (index + 1) % buffer.size();
    
    double sum = 0.0;
    for (double val : buffer) {
        sum += val;
    }
    
    return sum / buffer.size();
}

// 상태 정보 출력
void SmoothMotorController::printStatus() const {
    ROS_INFO("=== Smooth Motor Controller Status ===");
    ROS_INFO("Target: %.2f%%, Current: %.2f%%, Filtered: %.2f%%", 
             target_velocity_, current_velocity_, filtered_velocity_);
    ROS_INFO("Output: %.2f%%, Filtered Output: %.2f%%", 
             current_output_, filtered_output_);
    ROS_INFO("Emergency Stop: %s, Ramp: %s, Filter: %s",
             emergency_stop_ ? "ON" : "OFF",
             ramp_enabled_ ? "ON" : "OFF", 
             filter_enabled_ ? "ON" : "OFF");
    ROS_INFO("Max Accel/Decel: %.1f/%.1f %%/s", max_acceleration_, max_deceleration_);
} 