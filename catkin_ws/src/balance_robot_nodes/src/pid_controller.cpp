#include "balance_robot_nodes/pid_controller.h"
#include <algorithm>
#include <cmath>

PIDController::PIDController(double kp, double ki, double kd, double min_output, double max_output)
    : kp_(kp), ki_(ki), kd_(kd),
      error_(0.0), prev_error_(0.0), integral_(0.0), derivative_(0.0), output_(0.0),
      min_output_(min_output), max_output_(max_output),
      integral_max_(std::abs(max_output) * 0.8), // 적분 제한을 최대 출력의 80%로 설정
      first_run_(true) {
}

void PIDController::setGains(double kp, double ki, double kd) {
    kp_ = kp;
    ki_ = ki;
    kd_ = kd;
}

void PIDController::setOutputLimits(double min_output, double max_output) {
    min_output_ = min_output;
    max_output_ = max_output;
    integral_max_ = std::abs(max_output) * 0.8;
}

void PIDController::reset() {
    error_ = 0.0;
    prev_error_ = 0.0;
    integral_ = 0.0;
    derivative_ = 0.0;
    output_ = 0.0;
    first_run_ = true;
}

double PIDController::compute(double setpoint, double measured_value, double dt) {
    if (dt <= 0.0) {
        return output_;
    }
    
    // 오차 계산
    error_ = setpoint - measured_value;
    
    // 적분 계산 (와인드업 방지)
    integral_ += error_ * dt;
    
    // 적분 제한 적용
    if (integral_ > integral_max_) {
        integral_ = integral_max_;
    } else if (integral_ < -integral_max_) {
        integral_ = -integral_max_;
    }
    
    // 미분 계산 (첫 실행 시 미분은 0)
    if (first_run_) {
        derivative_ = 0.0;
        first_run_ = false;
    } else {
        derivative_ = (error_ - prev_error_) / dt;
    }
    
    // PID 출력 계산
    output_ = kp_ * error_ + ki_ * integral_ + kd_ * derivative_;
    
    // 출력 제한 적용
    if (output_ > max_output_) {
        output_ = max_output_;
    } else if (output_ < min_output_) {
        output_ = min_output_;
    }
    
    // 다음 계산을 위해 현재 오차 저장
    prev_error_ = error_;
    
    return output_;
} 