#include "motor_controller/HallSensorEncoder.h"
#include <sstream>
#include <iomanip>
#include <algorithm>
#include <cmath>

#include <ros/ros.h>

HallSensorEncoder::HallSensorEncoder(double wheel_radius, double wheel_base, double max_velocity)
    : wheel_radius_(wheel_radius)
    , wheel_base_(wheel_base)
    , steps_per_revolution_(20)
    , max_velocity_(max_velocity)
    , velocity_timeout_(0.1)  // 100ms 타임아웃
{
    last_odom_time_ = std::chrono::high_resolution_clock::now();
}

HallSensorEncoder::~HallSensorEncoder() {
}

bool HallSensorEncoder::initialize() {
    // 홀 센서 시퀀스 초기화
    initHallSequences();
    
    // 상태 리셋
    reset();
    
    return true;
}

void HallSensorEncoder::initHallSequences() {
    // Wheel 0 시퀀스: 001 -> 101 -> 111 -> 110 -> 010 -> 000 -> 001
    wheel0_sequence_ = {
        0b001,  // 1 : [0]
        0b101,  // 5 : [1]
        0b111,  // 7 : [2]
        0b110,  // 6 : [3]
        0b010,  // 2 : [4]
        0b000   // 0 : [5]
    };
    wheel0_sequence_inverse = {
        5,0,4,-1,
        -1,1,3,2
    };
    
    // Wheel 1 시퀀스: 110 -> 100 -> 101 -> 001 -> 011 -> 010 -> 110
    wheel1_sequence_ = {
        0b110,  // 6 : [0]
        0b100,  // 4 : [1]
        0b101,  // 5 : [2]
        0b001,  // 1 : [3]
        0b011,  // 3 : [4]
        0b010   // 2 : [5]
    };

    wheel1_sequence_inverse = {
        -1,3,5,4,
        1,2,0,-1
    };
}

int HallSensorEncoder::findSequenceIndex(const std::vector<int>& inverse, int hall_value) {
    return inverse[hall_value];
}

int HallSensorEncoder::getNextSequenceIndex(const std::vector<int>& sequence, int current_index) {
    return (current_index + 1) % static_cast<int>(sequence.size());
}

int HallSensorEncoder::getPrevSequenceIndex(const std::vector<int>& sequence, int current_index) {
    return (current_index - 1 + static_cast<int>(sequence.size())) % static_cast<int>(sequence.size());
}

bool HallSensorEncoder::updateWheelState(WheelInfo& wheel_info, WheelState& wheel_state,
                                        const std::vector<int>& sequence, const std::vector<int>& inverse, int new_hall_state) {
    if (new_hall_state == wheel_info.current_hall_state) {
        return false; // 변화 없음
    }
    
    auto current_time = std::chrono::high_resolution_clock::now();
    
    wheel_state.last_hall_state = wheel_info.current_hall_state;
    wheel_info.current_hall_state = new_hall_state;
    wheel_state.last_update_time = current_time;
    
    // 첫 번째 업데이트인 경우
    if (wheel_state.last_hall_state == -1) {
        int index = findSequenceIndex(inverse, new_hall_state);
        if (index != -1) {
            wheel_state.sequence_index = index;
            wheel_info.is_valid = true;
        }
        return false;
    }
    
    // 현재 시퀀스 인덱스 찾기
    int current_index = findSequenceIndex(inverse, wheel_state.last_hall_state);
    int new_index = findSequenceIndex(inverse, new_hall_state);
    
    if (current_index == -1 || new_index == -1) {
        wheel_info.is_valid = false;
        return false;
    }
    
    std::vector<int>  wheel_next_next = {2,3,4,5,0,1};
    std::vector<int>  wheel_next = {1,2,3,4,5,0};
    std::vector<int>  wheel_prev = {5,0,1,2,3,4};
    std::vector<int>  wheel_prev_prev = {4,5,0,1,2,3};
    // 방향 판단
    int expected_next = wheel_next[current_index];//getNextSequenceIndex(sequence, current_index);
    int expected_next_next = wheel_next_next[current_index];//getNextSequenceIndex(sequence, expected_next);
    int expected_prev = wheel_prev[current_index];//getPrevSequenceIndex(sequence, current_index);
    int expected_prev_prev = wheel_prev_prev[current_index];//getPrevSequenceIndex(sequence, expected_prev);
    
    if (new_index == expected_next) {
        // 정방향
        wheel_info.direction_forward = true;
        wheel_info.encoder_count++;
        wheel_state.sequence_index = new_index;
        wheel_info.is_valid = true;
    } else if (new_index == expected_next_next) {
        // 정방향
        wheel_info.direction_forward = true;
        wheel_info.encoder_count+=2;
        wheel_state.sequence_index = new_index;
        wheel_info.is_valid = true;
    } else if (new_index == expected_prev) {
        // 역방향
        wheel_info.direction_forward = false;
        wheel_info.encoder_count--;
        wheel_state.sequence_index = new_index;
        wheel_info.is_valid = true;
    } else if (new_index == expected_prev_prev) {
        // 역방향
        wheel_info.direction_forward = false;
        wheel_info.encoder_count-=2;
        wheel_state.sequence_index = new_index;
        wheel_info.is_valid = true;
    } else {
        // 시퀀스 오류 - 스텝을 건너뛰었거나 노이즈
        // 새로운 인덱스로 강제 업데이트하지만 카운트는 증가시키지 않음
        ROS_ERROR("step");
        wheel_state.sequence_index = new_index;
        wheel_info.is_valid = false;
        return false;
    }
    
    return true;
}

void HallSensorEncoder::calculatePosition(WheelInfo& wheel_info, const WheelState& wheel_state) {
    // 엔코더 카운트를 라디안으로 변환
    wheel_info.position = (2.0 * M_PI * wheel_info.encoder_count) / steps_per_revolution_;
}

void HallSensorEncoder::calculateVelocity(WheelInfo& wheel_info, WheelState& wheel_state) {
    auto current_time = std::chrono::high_resolution_clock::now();
    auto time_diff = std::chrono::duration_cast<std::chrono::microseconds>(
        current_time - wheel_state.last_velocity_time);
    double dt = time_diff.count() / 1000000.0; // 초로 변환
    
    long count_diff = wheel_info.encoder_count - wheel_state.last_encoder_count;  
    if (dt > 0.1 || count_diff > 100 || count_diff < -100) { // 1ms 간격 이상 또는 10ticks 이상
        double angle_diff = (2.0 * M_PI * count_diff) / steps_per_revolution_;
        
        double new_velocity = angle_diff / dt;
        
        // 속도 필터링 (급격한 변화 제한)
        if (std::abs(new_velocity) <= max_velocity_) {
            wheel_info.velocity = new_velocity;
        } else {
            wheel_info.velocity = (new_velocity > 0) ? max_velocity_ : -max_velocity_;
        }
        
        wheel_state.last_velocity_time = current_time;
        wheel_state.last_encoder_count = wheel_info.encoder_count;
    }
    
    // 타임아웃 체크 - 일정 시간 동안 업데이트가 없으면 속도를 0으로 설정
    auto time_since_update = std::chrono::duration_cast<std::chrono::microseconds>(
        current_time - wheel_state.last_update_time);
    if (time_since_update.count() / 1000000.0 > velocity_timeout_) {
        wheel_info.velocity = 0.0;
    }
}

void HallSensorEncoder::updateRobotOdometry() {
    auto current_time = std::chrono::high_resolution_clock::now();
    auto time_diff = std::chrono::duration_cast<std::chrono::microseconds>(
        current_time - last_odom_time_);
    double dt = time_diff.count() / 1000000.0; // 초로 변환
    
    if (dt > 0.001) { // 최소 1ms 간격
        // 바퀴 선속도 계산
        double v_left = wheel0_info_.velocity * wheel_radius_;
        double v_right = wheel1_info_.velocity * wheel_radius_;
        
        // 로봇 선속도 및 각속도 계산
        robot_odom_.linear_velocity = (v_left + v_right) / 2.0;
        robot_odom_.angular_velocity = (v_right - v_left) / wheel_base_;
        
        // 로봇 위치 업데이트 (오일러 적분)
        robot_odom_.x += robot_odom_.linear_velocity * cos(robot_odom_.theta) * dt;
        robot_odom_.y += robot_odom_.linear_velocity * sin(robot_odom_.theta) * dt;
        robot_odom_.theta += robot_odom_.angular_velocity * dt;
        
        // 각도 정규화 (-π ~ π)
        robot_odom_.theta = atan2(sin(robot_odom_.theta), cos(robot_odom_.theta));
        
        last_odom_time_ = current_time;
    }
}

void HallSensorEncoder::updateHallSensors(bool wheel0_bit0, bool wheel0_bit1, bool wheel0_bit2,
                                         bool wheel1_bit0, bool wheel1_bit1, bool wheel1_bit2) {
    // 비트를 3비트 정수로 변환
    int wheel0_hall = (wheel0_bit2 << 2) | (wheel0_bit1 << 1) | wheel0_bit0;
    int wheel1_hall = (wheel1_bit2 << 2) | (wheel1_bit1 << 1) | wheel1_bit0;
    
    updateHallSensors(wheel0_hall, wheel1_hall);
}

void HallSensorEncoder::updateHallSensors(int wheel0_hall, int wheel1_hall) {
    bool updated0 = updateWheelState(wheel0_info_, wheel0_state_, wheel0_sequence_, wheel0_sequence_inverse, wheel0_hall);
    bool updated1 = updateWheelState(wheel1_info_, wheel1_state_, wheel1_sequence_, wheel1_sequence_inverse, wheel1_hall);
    static int cnt;
    cnt++;

    // 위치 계산 (항상 수행)
    calculatePosition(wheel0_info_, wheel0_state_);
    calculatePosition(wheel1_info_, wheel1_state_);
    
    // 속도 계산 (업데이트가 있을 때만)
    if (updated0) {
        calculateVelocity(wheel0_info_, wheel0_state_);
    }
    if (updated1) {
        calculateVelocity(wheel1_info_, wheel1_state_);
    }
    if(cnt&0xF == 0x1F){
        // 오도메트리 업데이트
        updateRobotOdometry();
    
        // 타임아웃된 속도 체크 (업데이트가 없어도 수행)
        calculateVelocity(wheel0_info_, wheel0_state_);
        calculateVelocity(wheel1_info_, wheel1_state_);

    }
}

void HallSensorEncoder::reset() {
    // 바퀴 정보 리셋
    wheel0_info_ = WheelInfo();
    wheel1_info_ = WheelInfo();
    
    // 바퀴 상태 리셋
    wheel0_state_ = WheelState();
    wheel1_state_ = WheelState();
    
    // 로봇 오도메트리 리셋
    robot_odom_ = RobotOdometry();
    last_odom_time_ = std::chrono::high_resolution_clock::now();
    
    // 시퀀스 인덱스 초기화 (첫 번째 값으로)
    if (!wheel0_sequence_.empty() && !wheel1_sequence_.empty()) {
        wheel0_info_.current_hall_state = wheel0_sequence_[0];
        wheel1_info_.current_hall_state = wheel1_sequence_[0];
        wheel0_state_.sequence_index = 0;
        wheel1_state_.sequence_index = 0;
    }
}

std::string HallSensorEncoder::getDebugString() {
    std::stringstream ss;
    
    ss << "=== Hall Sensor Encoder Debug Info ===\n";
    ss << std::fixed << std::setprecision(3);
    
    // 바퀴 0 정보
    ss << "Wheel 0:\n";
    ss << "  Hall State: " << std::bitset<3>(wheel0_info_.current_hall_state) 
       << " (" << wheel0_info_.current_hall_state << ")\n";
    ss << "  Count: " << wheel0_info_.encoder_count << "\n";
    ss << "  Position: " << wheel0_info_.position << " rad\n";
    ss << "  Velocity: " << wheel0_info_.velocity << " rad/s\n";
    ss << "  Direction: " << (wheel0_info_.direction_forward ? "Forward" : "Reverse") << "\n";
    ss << "  Valid: " << (wheel0_info_.is_valid ? "Yes" : "No") << "\n";
    
    // 바퀴 1 정보
    ss << "Wheel 1:\n";
    ss << "  Hall State: " << std::bitset<3>(wheel1_info_.current_hall_state) 
       << " (" << wheel1_info_.current_hall_state << ")\n";
    ss << "  Count: " << wheel1_info_.encoder_count << "\n";
    ss << "  Position: " << wheel1_info_.position << " rad\n";
    ss << "  Velocity: " << wheel1_info_.velocity << " rad/s\n";
    ss << "  Direction: " << (wheel1_info_.direction_forward ? "Forward" : "Reverse") << "\n";
    ss << "  Valid: " << (wheel1_info_.is_valid ? "Yes" : "No") << "\n";
    
    // 로봇 오도메트리
    ss << "Robot Odometry:\n";
    ss << "  Position: (" << robot_odom_.x << ", " << robot_odom_.y << ") m\n";
    ss << "  Orientation: " << robot_odom_.theta << " rad (" 
       << (robot_odom_.theta * 180.0 / M_PI) << " deg)\n";
    ss << "  Linear Vel: " << robot_odom_.linear_velocity << " m/s\n";
    ss << "  Angular Vel: " << robot_odom_.angular_velocity << " rad/s\n";
    
    return ss.str();
}