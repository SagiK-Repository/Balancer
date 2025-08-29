#include "balance_robot_nodes/balance_controller_node.h"
#include <ros/ros.h>
#include <cmath>

BalanceControllerNode::BalanceControllerNode(ros::NodeHandle& nh, ros::NodeHandle& pnh)
    : nh_(nh), pnh_(pnh),
      target_speed_0_(0.0), target_speed_1_(0.0),
      actual_speed_0_(0.0), actual_speed_1_(0.0),
      pid_output_0_(0.0), pid_output_1_(0.0), balance_output_(0.0),
      target_roll_angle_(0.0), current_roll_angle_(0.0),
      current_control_mode_(PID_MODE),
      use_pid_control_(true), use_balance_control_(false), use_smooth_control_(true),
      emergency_stop_active_(false), system_ready_(false),
      motor_status_received_(false), sensor_data_received_(false),
      max_speed_(100.0), control_frequency_(500.0), status_frequency_(10.0),
      motor_status_timeout_(0.1), sensor_data_timeout_(0.1),
      pid_motor0_(2.0, 0.5, 0.1, -100.0, 100.0),
      pid_motor1_(2.0, 0.5, 0.1, -100.0, 100.0),
      pid_balance_(15.0, 2.0, 0.5, -100.0, 100.0),
      deadzone_compensator_(5.0, 8.0),
      direction_controller_(10.0, 0.1) {
    
    // 모터 명령 초기화
    current_motor_command_.header.frame_id = "base_link";
    current_motor_command_.motor0_output = 0.0;
    current_motor_command_.motor1_output = 0.0;
    current_motor_command_.motor0_direction = true;
    current_motor_command_.motor1_direction = true;
    current_motor_command_.control_mode = static_cast<uint8_t>(current_control_mode_);
    current_motor_command_.emergency_stop = false;
    
    // 컨트롤러 상태 초기화
    controller_status_.header.frame_id = "base_link";
    controller_status_.current_control_mode = static_cast<uint8_t>(current_control_mode_);
    controller_status_.target_speed_0 = 0.0;
    controller_status_.target_speed_1 = 0.0;
    controller_status_.actual_speed_0 = 0.0;
    controller_status_.actual_speed_1 = 0.0;
    controller_status_.pid_output_0 = 0.0;
    controller_status_.pid_output_1 = 0.0;
    controller_status_.balance_output = 0.0;
    controller_status_.pid_error_0 = 0.0;
    controller_status_.pid_error_1 = 0.0;
    controller_status_.balance_error = 0.0;
    controller_status_.target_roll_angle = 0.0;
    controller_status_.current_roll_angle = 0.0;
    controller_status_.use_pid_control = true;
    controller_status_.use_balance_control = false;
    controller_status_.use_smooth_control = true;
    controller_status_.emergency_stop_active = false;
    controller_status_.system_ready = false;
    controller_status_.sensors_ok = false;
    controller_status_.motors_ok = false;
    controller_status_.control_loop_frequency = 0;
}

BalanceControllerNode::~BalanceControllerNode() {
    // 비상 정지 및 정리
    handleEmergencyStop();
    ROS_INFO("Balance Controller Node shutdown complete");
}

bool BalanceControllerNode::initialize() {
    ROS_INFO("Initializing Balance Controller Node...");
    
    // 매개변수 로드
    loadParameters();
    loadPIDGains();
    loadSmoothControlParameters();
    loadDeadzoneParameters();
    loadDirectionControlParameters();
    
    // ROS 토픽 설정
    // 구독자
    motor_status_sub_ = nh_.subscribe("motor_status", 10, 
                                     &BalanceControllerNode::motorStatusCallback, this);
    sensor_data_sub_ = nh_.subscribe("sensor_data", 10,
                                    &BalanceControllerNode::sensorDataCallback, this);
    target_speed0_sub_ = nh_.subscribe("target_speed0", 10,
                                      &BalanceControllerNode::targetSpeed0Callback, this);
    target_speed1_sub_ = nh_.subscribe("target_speed1", 10,
                                      &BalanceControllerNode::targetSpeed1Callback, this);
    cmd_vel_sub_ = nh_.subscribe("cmd_vel", 10,
                                &BalanceControllerNode::cmdVelCallback, this);
    emergency_stop_sub_ = nh_.subscribe("emergency_stop", 10,
                                       &BalanceControllerNode::emergencyStopCallback, this);
    
    // 발행자
    motor_command_pub_ = nh_.advertise<balance_robot_nodes::MotorCommand>("motor_command", 10);
    controller_status_pub_ = nh_.advertise<balance_robot_nodes::ControllerStatus>("controller_status", 10);
    system_status_pub_ = nh_.advertise<std_msgs::Bool>("system_ready", 10);
    
    // 타이머 설정
    control_timer_ = nh_.createTimer(ros::Duration(1.0 / control_frequency_),
                                    &BalanceControllerNode::controlTimerCallback, this);
    status_timer_ = nh_.createTimer(ros::Duration(1.0 / status_frequency_),
                                   &BalanceControllerNode::statusTimerCallback, this);
    
    // 초기 시간 설정
    last_control_time_ = ros::Time::now();
    last_motor_status_time_ = ros::Time::now();
    last_sensor_data_time_ = ros::Time::now();
    
    ROS_INFO("Balance Controller Node initialization complete");
    ROS_INFO("Control modes: PID=%s, Balance=%s, Smooth=%s",
             use_pid_control_ ? "ON" : "OFF",
             use_balance_control_ ? "ON" : "OFF", 
             use_smooth_control_ ? "ON" : "OFF");
    
    return true;
}

void BalanceControllerNode::loadParameters() {
    // 제어 모드 설정
    pnh_.param("use_pid_control", use_pid_control_, true);
    pnh_.param("use_balance_control", use_balance_control_, false);
    pnh_.param("use_smooth_control", use_smooth_control_, true);
    
    // 제어 파라미터
    pnh_.param("max_speed", max_speed_, 100.0);
    pnh_.param("control_frequency", control_frequency_, 500.0);
    pnh_.param("status_frequency", status_frequency_, 10.0);
    pnh_.param("target_roll_angle", target_roll_angle_, 0.0);
    
    // 타임아웃 설정
    pnh_.param("motor_status_timeout", motor_status_timeout_, 0.1);
    pnh_.param("sensor_data_timeout", sensor_data_timeout_, 0.1);
    
    // 초기 제어 모드 결정
    if (use_balance_control_) {
        current_control_mode_ = BALANCE_MODE;
    } else if (use_pid_control_) {
        current_control_mode_ = PID_MODE;
    } else {
        current_control_mode_ = DIRECT_MODE;
    }
    
    ROS_INFO("Loaded controller parameters:");
    ROS_INFO("  Max speed: %.1f, Control freq: %.1fHz", max_speed_, control_frequency_);
    ROS_INFO("  Control mode: %d, Target roll: %.1f°", 
             static_cast<int>(current_control_mode_), target_roll_angle_);
}

void BalanceControllerNode::loadPIDGains() {
    // 모터 PID 게인
    double kp0, ki0, kd0, kp1, ki1, kd1;
    pnh_.param("pid_kp_motor0", kp0, 2.0);
    pnh_.param("pid_ki_motor0", ki0, 0.5);
    pnh_.param("pid_kd_motor0", kd0, 0.1);
    pnh_.param("pid_kp_motor1", kp1, 2.0);
    pnh_.param("pid_ki_motor1", ki1, 0.5);
    pnh_.param("pid_kd_motor1", kd1, 0.1);
    
    // 밸런싱 PID 게인
    double kp_bal, ki_bal, kd_bal;
    pnh_.param("pid_kp_balance", kp_bal, 15.0);
    pnh_.param("pid_ki_balance", ki_bal, 2.0);
    pnh_.param("pid_kd_balance", kd_bal, 0.5);
    
    // PID 컨트롤러 설정
    pid_motor0_.setGains(kp0, ki0, kd0);
    pid_motor1_.setGains(kp1, ki1, kd1);
    pid_balance_.setGains(kp_bal, ki_bal, kd_bal);
    
    pid_motor0_.setOutputLimits(-max_speed_, max_speed_);
    pid_motor1_.setOutputLimits(-max_speed_, max_speed_);
    pid_balance_.setOutputLimits(-max_speed_, max_speed_);
    
    ROS_INFO("PID Gains loaded:");
    ROS_INFO("  Motor0: Kp=%.2f, Ki=%.2f, Kd=%.2f", kp0, ki0, kd0);
    ROS_INFO("  Motor1: Kp=%.2f, Ki=%.2f, Kd=%.2f", kp1, ki1, kd1);
    ROS_INFO("  Balance: Kp=%.2f, Ki=%.2f, Kd=%.2f", kp_bal, ki_bal, kd_bal);
}

void BalanceControllerNode::loadSmoothControlParameters() {
    double max_accel, max_decel, vel_filter, out_filter;
    pnh_.param("smooth_max_acceleration", max_accel, 30.0);
    pnh_.param("smooth_max_deceleration", max_decel, 60.0);
    pnh_.param("smooth_velocity_filter", vel_filter, 0.4);
    pnh_.param("smooth_output_filter", out_filter, 0.6);
    
    smooth_motor0_.setParameters(max_accel, max_decel, vel_filter, out_filter);
    smooth_motor1_.setParameters(max_accel, max_decel, vel_filter, out_filter);
    
    ROS_INFO("Smooth control parameters:");
    ROS_INFO("  Accel/Decel: %.1f/%.1f %%/s", max_accel, max_decel);
    ROS_INFO("  Filters: vel=%.2f, out=%.2f", vel_filter, out_filter);
}

void BalanceControllerNode::loadDeadzoneParameters() {
    double deadzone_threshold, compensation_value;
    pnh_.param("motor_deadzone", deadzone_threshold, 5.0);
    pnh_.param("deadzone_compensation", compensation_value, 8.0);
    
    deadzone_compensator_.setParameters(deadzone_threshold, compensation_value);
    
    controller_status_.motor_deadzone = deadzone_threshold;
    controller_status_.deadzone_compensation = compensation_value;
    
    ROS_INFO("Deadzone compensation: threshold=%.1f%%, compensation=%.1f%%",
             deadzone_threshold, compensation_value);
}

void BalanceControllerNode::loadDirectionControlParameters() {
    double brake_threshold, brake_duration;
    pnh_.param("brake_zone_threshold", brake_threshold, 10.0);
    pnh_.param("brake_duration", brake_duration, 0.1);
    
    direction_controller_.setParameters(brake_threshold, brake_duration);
    
    controller_status_.brake_zone_threshold = brake_threshold;
    controller_status_.brake_duration = brake_duration;
    
    ROS_INFO("Direction change control: threshold=%.1f%%, duration=%.3fs",
             brake_threshold, brake_duration);
}

void BalanceControllerNode::motorStatusCallback(const balance_robot_nodes::MotorStatus::ConstPtr& msg) {
    current_motor_status_ = *msg;
    motor_status_received_ = true;
    last_motor_status_time_ = ros::Time::now();
    
    // 모터 속도 업데이트
    actual_speed_0_ = current_motor_status_.motor0_speed;
    actual_speed_1_ = current_motor_status_.motor1_speed;
    
    // 시스템 상태 업데이트
    controller_status_.motors_ok = current_motor_status_.spi_communication_ok &&
                                  current_motor_status_.dac0_ok &&
                                  current_motor_status_.dac1_ok &&
                                  current_motor_status_.gpio_ok;
}

void BalanceControllerNode::sensorDataCallback(const balance_robot_nodes::SensorData::ConstPtr& msg) {
    current_sensor_data_ = *msg;
    sensor_data_received_ = true;
    last_sensor_data_time_ = ros::Time::now();
    
    // 현재 Roll 각도 업데이트 (안정 상태에 따라 선택)
    if (current_sensor_data_.is_stable && current_sensor_data_.tilt_confidence > 70.0) {
        current_roll_angle_ = current_sensor_data_.stable_roll_angle;
    } else {
        current_roll_angle_ = current_sensor_data_.roll_angle;
    }
    
    // 시스템 상태 업데이트
    controller_status_.sensors_ok = current_sensor_data_.i2c_communication_ok;
}

void BalanceControllerNode::targetSpeed0Callback(const std_msgs::Float32::ConstPtr& msg) {
    target_speed_0_ = clampSpeed(msg->data);
    ROS_INFO("Target speed 0 set to: %.2f", target_speed_0_);
}

void BalanceControllerNode::targetSpeed1Callback(const std_msgs::Float32::ConstPtr& msg) {
    target_speed_1_ = clampSpeed(msg->data);
    ROS_INFO("Target speed 1 set to: %.2f", target_speed_1_);
}

void BalanceControllerNode::cmdVelCallback(const geometry_msgs::Twist::ConstPtr& msg) {
    // cmd_vel을 개별 바퀴 속도로 변환
    double left_speed, right_speed;
    convertCmdVelToWheelSpeeds(*msg, left_speed, right_speed);
    
    target_speed_0_ = clampSpeed(left_speed);
    target_speed_1_ = clampSpeed(right_speed);
    
    ROS_INFO("cmd_vel received: linear=%.2f, angular=%.2f -> speeds=[%.2f,%.2f]",
             msg->linear.x, msg->angular.z, target_speed_0_, target_speed_1_);
}

void BalanceControllerNode::emergencyStopCallback(const std_msgs::Bool::ConstPtr& msg) {
    if (msg->data && !emergency_stop_active_) {
        ROS_WARN("Emergency stop activated!");
        emergency_stop_active_ = true;
        handleEmergencyStop();
    } else if (!msg->data && emergency_stop_active_) {
        ROS_INFO("Emergency stop deactivated");
        emergency_stop_active_ = false;
    }
}

void BalanceControllerNode::controlTimerCallback(const ros::TimerEvent& event) {
    // 시스템 상태 확인
    checkSystemStatus();
    
    if (!system_ready_ || emergency_stop_active_) {
        // 시스템이 준비되지 않았거나 비상 정지 상태
        current_motor_command_.motor0_output = 0.0;
        current_motor_command_.motor1_output = 0.0;
        current_motor_command_.emergency_stop = emergency_stop_active_;
        publishMotorCommand();
        return;
    }
    
    // 안전 체크
    if (!performSafetyCheck()) {
        ROS_WARN_THROTTLE(1.0, "Safety check failed - stopping motors");
        handleEmergencyStop();
        return;
    }
    
    // 제어 모드에 따른 실행
    switch (current_control_mode_) {
        case DIRECT_MODE:
            executeDirectControl();
            break;
        case PID_MODE:
            executePIDControl();
            break;
        case BALANCE_MODE:
            executeBalanceControl();
            break;
        default:
            ROS_ERROR_THROTTLE(1.0, "Unknown control mode: %d", static_cast<int>(current_control_mode_));
            executeDirectControl();
            break;
    }
    
    // 모터 명령 발행
    publishMotorCommand();
    
    // 제어 상태 출력 (낮은 빈도로)
    static int print_counter = 0;
    if (++print_counter >= 100) {  // 500Hz에서 5Hz로 감소
        printControlStatus();
        print_counter = 0;
    }
    
    last_control_time_ = ros::Time::now();
}

void BalanceControllerNode::statusTimerCallback(const ros::TimerEvent& event) {
    // 컨트롤러 상태 발행
    publishControllerStatus();
    
    // 시스템 진단 수행
    performSystemDiagnostics();
}

void BalanceControllerNode::checkSystemStatus() {
    ros::Time now = ros::Time::now();
    
    // 데이터 수신 상태 체크
    bool motor_data_timeout = (now - last_motor_status_time_).toSec() > motor_status_timeout_;
    bool sensor_data_timeout = (now - last_sensor_data_time_).toSec() > sensor_data_timeout_;
    
    // 전체 시스템 준비 상태
    system_ready_ = motor_status_received_ && sensor_data_received_ && 
                   !motor_data_timeout && !sensor_data_timeout &&
                   controller_status_.motors_ok && controller_status_.sensors_ok;
    
    if (motor_data_timeout) {
        ROS_WARN_THROTTLE(1.0, "Motor status timeout - no data for %.3fs", motor_status_timeout_);
    }
    if (sensor_data_timeout) {
        ROS_WARN_THROTTLE(1.0, "Sensor data timeout - no data for %.3fs", sensor_data_timeout_);
    }
}

bool BalanceControllerNode::performSafetyCheck() {
    // 기본 안전 체크
    if (!motor_status_received_ || !sensor_data_received_) {
        return false;
    }
    
    // 각도 안전 체크 (밸런싱 모드에서)
    if (use_balance_control_) {
        double angle_limit = 45.0;  // 45도 이상 기울면 정지
        if (std::abs(current_roll_angle_) > angle_limit) {
            ROS_ERROR("Critical angle detected: %.1f°", current_roll_angle_);
            return false;
        }
    }
    
    // 속도 안전 체크
    if (std::abs(actual_speed_0_) > max_speed_ * 1.5 || std::abs(actual_speed_1_) > max_speed_ * 1.5) {
        ROS_ERROR("Motor speed exceeded safety limit");
        return false;
    }
    
    // 하드웨어 상태 체크
    if (!controller_status_.motors_ok || !controller_status_.sensors_ok) {
        return false;
    }
    
    return true;
}

void BalanceControllerNode::executeDirectControl() {
    // 직접 제어 모드 - 목표 속도를 그대로 적용
    double output0 = target_speed_0_;
    double output1 = target_speed_1_;
    
    // 부드러운 제어 적용
    if (use_smooth_control_) {
        output0 = smooth_motor0_.updateWithOutput(output0);
        output1 = smooth_motor1_.updateWithOutput(output1);
    }
    
    // 데드존 보상 적용
    output0 = deadzone_compensator_.compensate(output0);
    output1 = deadzone_compensator_.compensate(output1);
    
    // 방향 전환 제어 적용
    output0 = direction_controller_.applyControl(output0, 0, 0.002);  // 500Hz = 2ms
    output1 = direction_controller_.applyControl(output1, 1, 0.002);
    
    // 최종 출력 설정
    current_motor_command_.motor0_output = clampSpeed(output0);
    current_motor_command_.motor1_output = clampSpeed(output1);
    
    // 방향 설정
    current_motor_command_.motor0_direction = (current_motor_command_.motor0_output >= 0);
    current_motor_command_.motor1_direction = (current_motor_command_.motor1_output >= 0);
}

void BalanceControllerNode::executePIDControl() {
    // PID 제어 모드 - 속도 오차를 PID로 보정
    ros::Time now = ros::Time::now();
    double dt = (now - last_control_time_).toSec();
    
    if (dt > 0.001) {  // 최소 1ms 간격
        // PID 계산
        pid_output_0_ = pid_motor0_.compute(target_speed_0_, actual_speed_0_, dt);
        pid_output_1_ = pid_motor1_.compute(target_speed_1_, actual_speed_1_, dt);
        
        // PID 오차 저장
        controller_status_.pid_error_0 = target_speed_0_ - actual_speed_0_;
        controller_status_.pid_error_1 = target_speed_1_ - actual_speed_1_;
        
        double output0 = pid_output_0_;
        double output1 = pid_output_1_;
        
        // 부드러운 제어 적용
        if (use_smooth_control_) {
            output0 = smooth_motor0_.updateWithOutput(output0);
            output1 = smooth_motor1_.updateWithOutput(output1);
        }
        
        // 데드존 보상 적용
        output0 = deadzone_compensator_.compensate(output0);
        output1 = deadzone_compensator_.compensate(output1);
        
        // 방향 전환 제어 적용
        output0 = direction_controller_.applyControl(output0, 0, dt);
        output1 = direction_controller_.applyControl(output1, 1, dt);
        
        // 최종 출력 설정
        current_motor_command_.motor0_output = clampSpeed(output0);
        current_motor_command_.motor1_output = clampSpeed(output1);
        
        // 방향 설정
        current_motor_command_.motor0_direction = (current_motor_command_.motor0_output >= 0);
        current_motor_command_.motor1_direction = (current_motor_command_.motor1_output >= 0);
    }
}

void BalanceControllerNode::executeBalanceControl() {
    // 밸런스 제어 모드 - 자세를 유지하면서 속도 제어
    ros::Time now = ros::Time::now();
    double dt = (now - last_control_time_).toSec();
    
    if (dt > 0.001) {  // 최소 1ms 간격
        // 밸런스 PID 계산
        balance_output_ = pid_balance_.compute(target_roll_angle_, current_roll_angle_, dt);
        controller_status_.balance_error = target_roll_angle_ - current_roll_angle_;
        
        // 기본 모터 PID 계산
        pid_output_0_ = pid_motor0_.compute(target_speed_0_, actual_speed_0_, dt);
        pid_output_1_ = pid_motor1_.compute(target_speed_1_, actual_speed_1_, dt);
        
        // PID 오차 저장
        controller_status_.pid_error_0 = target_speed_0_ - actual_speed_0_;
        controller_status_.pid_error_1 = target_speed_1_ - actual_speed_1_;
        
        // 밸런스 보정을 두 모터에 동일하게 적용
        double output0 = pid_output_0_ + balance_output_;
        double output1 = pid_output_1_ + balance_output_;
        
        // 부드러운 제어 적용
        if (use_smooth_control_) {
            output0 = smooth_motor0_.updateWithOutput(output0);
            output1 = smooth_motor1_.updateWithOutput(output1);
        }
        
        // 데드존 보상 적용
        output0 = deadzone_compensator_.compensate(output0);
        output1 = deadzone_compensator_.compensate(output1);
        
        // 방향 전환 제어 적용
        output0 = direction_controller_.applyControl(output0, 0, dt);
        output1 = direction_controller_.applyControl(output1, 1, dt);
        
        // 최종 출력 설정
        current_motor_command_.motor0_output = clampSpeed(output0);
        current_motor_command_.motor1_output = clampSpeed(output1);
        
        // 방향 설정
        current_motor_command_.motor0_direction = (current_motor_command_.motor0_output >= 0);
        current_motor_command_.motor1_direction = (current_motor_command_.motor1_output >= 0);
    }
}

void BalanceControllerNode::handleEmergencyStop() {
    // 모든 출력 정지
    current_motor_command_.motor0_output = 0.0;
    current_motor_command_.motor1_output = 0.0;
    current_motor_command_.emergency_stop = true;
    
    // PID 컨트롤러 리셋
    pid_motor0_.reset();
    pid_motor1_.reset();
    pid_balance_.reset();
    
    // 부드러운 제어 리셋
    smooth_motor0_.reset();
    smooth_motor1_.reset();
    
    // 방향 전환 제어 리셋
    direction_controller_.reset();
    
    // 즉시 모터 명령 발행
    publishMotorCommand();
    
    ROS_WARN("Emergency stop executed - all controllers reset");
}

void BalanceControllerNode::publishMotorCommand() {
    current_motor_command_.header.stamp = ros::Time::now();
    current_motor_command_.control_mode = static_cast<uint8_t>(current_control_mode_);
    
    motor_command_pub_.publish(current_motor_command_);
}

void BalanceControllerNode::publishControllerStatus() {
    controller_status_.header.stamp = ros::Time::now();
    
    // 현재 상태 업데이트
    controller_status_.current_control_mode = static_cast<uint8_t>(current_control_mode_);
    controller_status_.target_speed_0 = target_speed_0_;
    controller_status_.target_speed_1 = target_speed_1_;
    controller_status_.actual_speed_0 = actual_speed_0_;
    controller_status_.actual_speed_1 = actual_speed_1_;
    controller_status_.pid_output_0 = pid_output_0_;
    controller_status_.pid_output_1 = pid_output_1_;
    controller_status_.balance_output = balance_output_;
    controller_status_.target_roll_angle = target_roll_angle_;
    controller_status_.current_roll_angle = current_roll_angle_;
    controller_status_.use_pid_control = use_pid_control_;
    controller_status_.use_balance_control = use_balance_control_;
    controller_status_.use_smooth_control = use_smooth_control_;
    controller_status_.emergency_stop_active = emergency_stop_active_;
    controller_status_.system_ready = system_ready_;
    
    // 제어 루프 주파수 계산
    static ros::Time last_status_time;
    static int status_counter = 0;
    ros::Time now = ros::Time::now();
    status_counter++;
    
    if ((now - last_status_time).toSec() >= 1.0) {
        controller_status_.control_loop_frequency = status_counter;
        status_counter = 0;
        last_status_time = now;
    }
    
    controller_status_pub_.publish(controller_status_);
    
    // 시스템 준비 상태 발행
    std_msgs::Bool system_ready_msg;
    system_ready_msg.data = system_ready_;
    system_status_pub_.publish(system_ready_msg);
}

void BalanceControllerNode::performSystemDiagnostics() {
    static int diagnostic_counter = 0;
    diagnostic_counter++;
    
    // 5초마다 진단 수행
    if (diagnostic_counter >= 50) {  // 10Hz * 5s = 50
        diagnostic_counter = 0;
        
        ROS_INFO("System Diagnostics:");
        ROS_INFO("  System ready: %s", system_ready_ ? "YES" : "NO");
        ROS_INFO("  Motors OK: %s, Sensors OK: %s", 
                 controller_status_.motors_ok ? "YES" : "NO",
                 controller_status_.sensors_ok ? "YES" : "NO");
        ROS_INFO("  Current mode: %s", getControlModeString().c_str());
        ROS_INFO("  Target speeds: [%.1f, %.1f], Actual: [%.1f, %.1f]",
                 target_speed_0_, target_speed_1_, actual_speed_0_, actual_speed_1_);
        
        if (use_balance_control_) {
            ROS_INFO("  Roll angle: target=%.1f°, current=%.1f°", 
                     target_roll_angle_, current_roll_angle_);
        }
    }
}

void BalanceControllerNode::printControlStatus() {
    const char* mode_str = getControlModeString().c_str();
    
    ROS_INFO_THROTTLE(1.0, "Control: Mode=%s, Targets=[%.1f,%.1f], Actual=[%.1f,%.1f], Outputs=[%.1f,%.1f]",
             mode_str, target_speed_0_, target_speed_1_, 
             actual_speed_0_, actual_speed_1_,
             current_motor_command_.motor0_output, current_motor_command_.motor1_output);
             
    if (use_balance_control_) {
        ROS_INFO_THROTTLE(1.0, "Balance: Target=%.1f°, Current=%.1f°, Output=%.1f",
                 target_roll_angle_, current_roll_angle_, balance_output_);
    }
}

double BalanceControllerNode::clampSpeed(double speed) {
    return std::max(-max_speed_, std::min(max_speed_, speed));
}

void BalanceControllerNode::convertCmdVelToWheelSpeeds(const geometry_msgs::Twist& cmd_vel, 
                                                      double& left_speed, double& right_speed) {
    // 차동 구동 로봇의 운동학 변환
    double linear_vel = cmd_vel.linear.x;   // m/s
    double angular_vel = cmd_vel.angular.z; // rad/s
    
    // 로봇 매개변수 (미터 단위)
    const double wheel_radius = 0.142;      // 바퀴 반지름 (m)
    const double wheel_base = 0.375;        // 바퀴 간 거리 (m)
    
    // 바퀴 속도 계산 (m/s)
    double left_vel = linear_vel - (angular_vel * wheel_base / 2.0);
    double right_vel = linear_vel + (angular_vel * wheel_base / 2.0);
    
    // RPM으로 변환
    left_speed = left_vel * 60.0 / (2.0 * M_PI * wheel_radius);
    right_speed = right_vel * 60.0 / (2.0 * M_PI * wheel_radius);
    
    // 최대 속도로 정규화 (필요시)
    double max_rpm = std::max(std::abs(left_speed), std::abs(right_speed));
    if (max_rpm > max_speed_) {
        double scale = max_speed_ / max_rpm;
        left_speed *= scale;
        right_speed *= scale;
    }
}

std::string BalanceControllerNode::getControlModeString() const {
    switch (current_control_mode_) {
        case DIRECT_MODE: return "DIRECT";
        case PID_MODE: return "PID";
        case BALANCE_MODE: return "BALANCE";
        default: return "UNKNOWN";
    }
}

// 메인 함수
int main(int argc, char** argv) {
    ros::init(argc, argv, "balance_controller_node");
    ros::NodeHandle nh;
    ros::NodeHandle pnh("~");
    
    BalanceControllerNode node(nh, pnh);
    
    if (!node.initialize()) {
        ROS_ERROR("Failed to initialize Balance Controller Node");
        return -1;
    }
    
    ROS_INFO("Balance Controller Node started successfully");
    ros::spin();
    
    return 0;
} 