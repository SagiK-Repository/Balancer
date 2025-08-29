#include <ros/ros.h>
#include <std_msgs/Float32.h>
#include <iostream>
#include <string>

class MotorTestController {
private:
    ros::NodeHandle nh_;
    ros::Publisher motor0_pub_;
    ros::Publisher motor1_pub_;
    
public:
    MotorTestController() {
        motor0_pub_ = nh_.advertise<std_msgs::Float32>("target_speed0", 10);
        motor1_pub_ = nh_.advertise<std_msgs::Float32>("target_speed1", 10);
        
        ROS_INFO("모터 테스트 컨트롤러 시작됨");
        ROS_INFO("사용법:");
        ROS_INFO("  0 [속도] - 모터0 속도 설정 (예: 0 10.5)");
        ROS_INFO("  1 [속도] - 모터1 속도 설정 (예: 1 -5.2)");
        ROS_INFO("  both [속도] - 두 모터 동일 속도 설정 (예: both 15.0)");
        ROS_INFO("  stop - 모든 모터 정지");
        ROS_INFO("  quit - 프로그램 종료");
    }
    
    void setMotor0Speed(double speed) {
        std_msgs::Float32 msg;
        msg.data = speed;
        motor0_pub_.publish(msg);
        ROS_INFO("모터0 속도 설정: %.2f", speed);
    }
    
    void setMotor1Speed(double speed) {
        std_msgs::Float32 msg;
        msg.data = speed;
        motor1_pub_.publish(msg);
        ROS_INFO("모터1 속도 설정: %.2f", speed);
    }
    
    void setBothMotorsSpeed(double speed) {
        setMotor0Speed(speed);
        setMotor1Speed(speed);
        ROS_INFO("두 모터 속도 설정: %.2f", speed);
    }
    
    void stopAllMotors() {
        setMotor0Speed(0.0);
        setMotor1Speed(0.0);
        ROS_INFO("모든 모터 정지");
    }
    
    void run() {
        std::string input;
        
        while (ros::ok()) {
            std::cout << "\n명령 입력 (도움말: help): ";
            std::getline(std::cin, input);
            
            if (input.empty()) continue;
            
            std::istringstream iss(input);
            std::string command;
            iss >> command;
            
            if (command == "quit" || command == "exit") {
                stopAllMotors();
                break;
            } else if (command == "help") {
                ROS_INFO("사용법:");
                ROS_INFO("  0 [속도] - 모터0 속도 설정");
                ROS_INFO("  1 [속도] - 모터1 속도 설정");
                ROS_INFO("  both [속도] - 두 모터 동일 속도");
                ROS_INFO("  stop - 모든 모터 정지");
                ROS_INFO("  quit - 프로그램 종료");
            } else if (command == "stop") {
                stopAllMotors();
            } else if (command == "0") {
                double speed;
                if (iss >> speed) {
                    setMotor0Speed(speed);
                } else {
                    ROS_WARN("속도 값을 입력하세요. 예: 0 10.5");
                }
            } else if (command == "1") {
                double speed;
                if (iss >> speed) {
                    setMotor1Speed(speed);
                } else {
                    ROS_WARN("속도 값을 입력하세요. 예: 1 -5.2");
                }
            } else if (command == "both") {
                double speed;
                if (iss >> speed) {
                    setBothMotorsSpeed(speed);
                } else {
                    ROS_WARN("속도 값을 입력하세요. 예: both 15.0");
                }
            } else {
                ROS_WARN("알 수 없는 명령: %s (도움말: help)", command.c_str());
            }
            
            ros::spinOnce();
        }
    }
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "motor_test_controller");
    
    MotorTestController controller;
    controller.run();
    
    return 0;
} 