#include <ros/ros.h>
#include <std_msgs/Float32.h>
#include <iostream>
#include <string>
#include <sys/select.h>
#include <unistd.h>
#include <algorithm>

class PIDTestNode {
public:
    PIDTestNode() : nh_("~") {
        // 발행자 설정
        speed0_pub_ = nh_.advertise<std_msgs::Float32>("/target_speed0", 10);
        speed1_pub_ = nh_.advertise<std_msgs::Float32>("/target_speed1", 10);
        
        // 구독자 설정 (실제 속도 모니터링)
        actual_speed0_sub_ = nh_.subscribe("/actual_speed0", 10, &PIDTestNode::actualSpeed0Callback, this);
        actual_speed1_sub_ = nh_.subscribe("/actual_speed1", 10, &PIDTestNode::actualSpeed1Callback, this);
        
        actual_speed0_ = 0.0;
        actual_speed1_ = 0.0;
        
        ROS_INFO("PID Test Node started. Commands:");
        ROS_INFO("  w/s: Forward/Backward");
        ROS_INFO("  a/d: Left/Right turn");
        ROS_INFO("  q/e: Increase/Decrease speed");
        ROS_INFO("  x: Stop motors");
        ROS_INFO("  h: Show help");
        ROS_INFO("  ESC: Exit");
        
        current_speed_ = 20.0;  // 기본 속도 (RPM)
        showStatus();
    }
    
    void run() {
        ros::Rate rate(10);  // 10Hz
        
        while (ros::ok()) {
            processInput();
            ros::spinOnce();
            rate.sleep();
        }
    }

private:
    ros::NodeHandle nh_;
    ros::Publisher speed0_pub_;
    ros::Publisher speed1_pub_;
    ros::Subscriber actual_speed0_sub_;
    ros::Subscriber actual_speed1_sub_;
    
    float current_speed_;
    float actual_speed0_;
    float actual_speed1_;
    
    void actualSpeed0Callback(const std_msgs::Float32::ConstPtr& msg) {
        actual_speed0_ = msg->data;
    }
    
    void actualSpeed1Callback(const std_msgs::Float32::ConstPtr& msg) {
        actual_speed1_ = msg->data;
    }
    
    void publishSpeeds(float speed0, float speed1) {
        std_msgs::Float32 msg0, msg1;
        msg0.data = speed0;
        msg1.data = speed1;
        speed0_pub_.publish(msg0);
        speed1_pub_.publish(msg1);
        
        ROS_INFO("Target: [%.1f, %.1f] RPM | Actual: [%.1f, %.1f] RPM", 
                 speed0, speed1, actual_speed0_, actual_speed1_);
    }
    
    void showStatus() {
        std::cout << "\n=== PID Control Test ===" << std::endl;
        std::cout << "Current Speed: " << current_speed_ << " RPM" << std::endl;
        std::cout << "Target: [" << 0.0 << ", " << 0.0 << "] RPM" << std::endl;
        std::cout << "Actual: [" << actual_speed0_ << ", " << actual_speed1_ << "] RPM" << std::endl;
        std::cout << "Command: ";
    }
    
    void processInput() {
        // 논블로킹 입력 처리
        fd_set readfds;
        struct timeval timeout;
        FD_ZERO(&readfds);
        FD_SET(STDIN_FILENO, &readfds);
        timeout.tv_sec = 0;
        timeout.tv_usec = 0;
        
        if (select(STDIN_FILENO + 1, &readfds, NULL, NULL, &timeout) > 0) {
            char input;
            std::cin >> input;
            
            switch (input) {
                case 'w':  // 전진
                    publishSpeeds(current_speed_, current_speed_);
                    break;
                case 's':  // 후진
                    publishSpeeds(-current_speed_, -current_speed_);
                    break;
                case 'a':  // 좌회전
                    publishSpeeds(-current_speed_ * 0.5, current_speed_ * 0.5);
                    break;
                case 'd':  // 우회전
                    publishSpeeds(current_speed_ * 0.5, -current_speed_ * 0.5);
                    break;
                case 'q':  // 속도 증가
                    current_speed_ = std::min(current_speed_ + 5.0f, 100.0f);
                    ROS_INFO("Speed increased to %.1f RPM", current_speed_);
                    break;
                case 'e':  // 속도 감소
                    current_speed_ = std::max(current_speed_ - 5.0f, 5.0f);
                    ROS_INFO("Speed decreased to %.1f RPM", current_speed_);
                    break;
                case 'x':  // 정지
                    publishSpeeds(0.0, 0.0);
                    break;
                case 'h':  // 도움말
                    ROS_INFO("Commands: w(forward) s(backward) a(left) d(right) q(+speed) e(-speed) x(stop) h(help)");
                    break;
                case 27:   // ESC
                    ROS_INFO("Exiting...");
                    publishSpeeds(0.0, 0.0);
                    ros::shutdown();
                    break;
                default:
                    break;
            }
        }
    }
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "pid_test_node");
    
    try {
        PIDTestNode test_node;
        test_node.run();
    } catch (const std::exception& e) {
        ROS_ERROR("PID Test Node error: %s", e.what());
        return -1;
    }
    
    return 0;
} 