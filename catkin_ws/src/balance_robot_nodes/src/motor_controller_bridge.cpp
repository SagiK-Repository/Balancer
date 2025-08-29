#include <ros/ros.h>
#include <std_msgs/Float32.h>
#include "balance_robot_nodes/MotorCommand.h"

class MotorControllerBridge {
private:
    ros::NodeHandle nh_;
    ros::NodeHandle pnh_;
    
    // 기존 motor_controller 호환 토픽
    ros::Subscriber speed0_sub_;
    ros::Subscriber speed1_sub_;
    
    // 새로운 balance_robot_nodes 토픽
    ros::Publisher motor_command_pub_;
    
    // 현재 목표 속도
    float target_speed0_;
    float target_speed1_;
    bool enable_motors_;
    
    // 타이머
    ros::Timer publish_timer_;
    
public:
    MotorControllerBridge(ros::NodeHandle& nh, ros::NodeHandle& pnh) 
        : nh_(nh), pnh_(pnh), target_speed0_(0.0), target_speed1_(0.0), enable_motors_(true) {
        
        // 기존 motor_controller 토픽 구독
        speed0_sub_ = nh_.subscribe("target_speed0", 10, &MotorControllerBridge::speed0Callback, this);
        speed1_sub_ = nh_.subscribe("target_speed1", 10, &MotorControllerBridge::speed1Callback, this);
        
        // 새로운 motor_command 토픽 발행
        motor_command_pub_ = nh_.advertise<balance_robot_nodes::MotorCommand>("motor_command", 10);
        
        // 주기적으로 motor_command 발행 (10Hz)
        publish_timer_ = nh_.createTimer(ros::Duration(0.1), &MotorControllerBridge::publishTimerCallback, this);
        
        ROS_INFO("Motor Controller Bridge started");
        ROS_INFO("Subscribing to: target_speed0, target_speed1");
        ROS_INFO("Publishing to: motor_command");
    }
    
    void speed0Callback(const std_msgs::Float32::ConstPtr& msg) {
        target_speed0_ = msg->data;
        ROS_DEBUG("Speed0: %.2f", target_speed0_);
    }
    
    void speed1Callback(const std_msgs::Float32::ConstPtr& msg) {
        target_speed1_ = msg->data;
        ROS_DEBUG("Speed1: %.2f", target_speed1_);
    }
    
    void publishTimerCallback(const ros::TimerEvent& event) {
        balance_robot_nodes::MotorCommand cmd;
        
        // 헤더 설정
        cmd.header.stamp = ros::Time::now();
        cmd.header.frame_id = "base_link";
        
        // 기존 motor_controller의 속도를 새로운 형식으로 변환
        // target_speed는 RPM, motor_output은 -100~100% 범위로 가정
        float max_speed = 100.0;  // 매개변수로 설정 가능
        
        cmd.motor0_output = std::max(-100.0f, std::min(100.0f, (target_speed0_ / max_speed) * 100.0f));
        cmd.motor1_output = std::max(-100.0f, std::min(100.0f, (target_speed1_ / max_speed) * 100.0f));
        
        cmd.motor0_direction = (cmd.motor0_output >= 0);
        cmd.motor1_direction = (cmd.motor1_output >= 0);
        
        cmd.control_mode = 0;  // 기본 제어 모드
        cmd.emergency_stop = false;
        
        motor_command_pub_.publish(cmd);
        
        ROS_DEBUG("Published motor_command: [%.2f, %.2f]", cmd.motor0_output, cmd.motor1_output);
    }
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "motor_controller_bridge");
    ros::NodeHandle nh;
    ros::NodeHandle pnh("~");
    
    MotorControllerBridge bridge(nh, pnh);
    
    ROS_INFO("Motor Controller Bridge ready");
    ros::spin();
    
    return 0;
} 