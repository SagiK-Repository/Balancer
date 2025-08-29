#include <ros/ros.h>
#include <std_msgs/Float32.h>
#include <iostream>
#include <string>
#include <thread>
#include <chrono>
#include <vector>

class DualMotorTester {
private:
    ros::NodeHandle nh_;
    ros::Publisher motor0_pub_;
    ros::Publisher motor1_pub_;
    ros::Subscriber actual_speed0_sub_;
    ros::Subscriber actual_speed1_sub_;
    ros::Subscriber sensor_confidence_sub_;
    
    double actual_speed0_;
    double actual_speed1_;
    double sensor_confidence_;
    
public:
    DualMotorTester() : actual_speed0_(0.0), actual_speed1_(0.0), sensor_confidence_(100.0) {
        motor0_pub_ = nh_.advertise<std_msgs::Float32>("target_speed0", 10);
        motor1_pub_ = nh_.advertise<std_msgs::Float32>("target_speed1", 10);
        
        actual_speed0_sub_ = nh_.subscribe("actual_speed0", 10, &DualMotorTester::actualSpeed0Callback, this);
        actual_speed1_sub_ = nh_.subscribe("actual_speed1", 10, &DualMotorTester::actualSpeed1Callback, this);
        sensor_confidence_sub_ = nh_.subscribe("sensor_confidence", 10, &DualMotorTester::sensorConfidenceCallback, this);
        
        ros::Duration(1.0).sleep(); // 초기화 대기
        
        ROS_INFO("=== 🚗 두 바퀴 모터 테스트 시스템 ===");
        printMenu();
    }
    
    void actualSpeed0Callback(const std_msgs::Float32::ConstPtr& msg) {
        actual_speed0_ = msg->data;
    }
    
    void actualSpeed1Callback(const std_msgs::Float32::ConstPtr& msg) {
        actual_speed1_ = msg->data;
    }
    
    void sensorConfidenceCallback(const std_msgs::Float32::ConstPtr& msg) {
        sensor_confidence_ = msg->data;
    }
    
    void setMotorSpeeds(double speed0, double speed1) {
        std_msgs::Float32 msg0, msg1;
        msg0.data = speed0;
        msg1.data = speed1;
        motor0_pub_.publish(msg0);
        motor1_pub_.publish(msg1);
        
        ROS_INFO("모터 속도 설정: [%.2f, %.2f]", speed0, speed1);
    }
    
    void stopAllMotors() {
        setMotorSpeeds(0.0, 0.0);
        ROS_INFO("🛑 모든 모터 정지");
    }
    
    void printStatus() {
        ROS_INFO("📊 현재 상태: 목표[%.2f, %.2f] 실제[%.2f, %.2f] 신뢰도:%.0f%%", 
                 0.0, 0.0, actual_speed0_, actual_speed1_, sensor_confidence_);
    }
    
    void printMenu() {
        std::cout << "\n=== 🎮 두 바퀴 모터 테스트 메뉴 ===" << std::endl;
        std::cout << "1. 기본 테스트 (각 모터 개별)" << std::endl;
        std::cout << "2. 동시 테스트 (두 모터 동일 속도)" << std::endl;
        std::cout << "3. 방향 테스트 (전진/후진/회전)" << std::endl;
        std::cout << "4. 부드러운 제어 테스트" << std::endl;
        std::cout << "5. 속도 단계별 테스트" << std::endl;
        std::cout << "6. 수동 제어 모드" << std::endl;
        std::cout << "7. 상태 확인" << std::endl;
        std::cout << "0. 정지 및 종료" << std::endl;
        std::cout << "선택: ";
    }
    
    void basicTest() {
        ROS_INFO("🔧 기본 테스트 시작 - 각 모터 개별 테스트");
        
        // 모터0 테스트
        ROS_INFO("1️⃣ 모터0 테스트 (5초)");
        setMotorSpeeds(5.0, 0.0);
        waitAndMonitor(5.0);
        
        stopAllMotors();
        waitAndMonitor(2.0);
        
        // 모터1 테스트
        ROS_INFO("2️⃣ 모터1 테스트 (5초)");
        setMotorSpeeds(0.0, 5.0);
        waitAndMonitor(5.0);
        
        stopAllMotors();
        ROS_INFO("✅ 기본 테스트 완료");
    }
    
    void simultaneousTest() {
        ROS_INFO("🔧 동시 테스트 시작 - 두 모터 동일 속도");
        
        std::vector<double> speeds = {3.0, 6.0, 10.0, -5.0, -10.0};
        
        for (double speed : speeds) {
            ROS_INFO("⚡ 두 모터 속도: %.1f (3초)", speed);
            setMotorSpeeds(speed, speed);
            waitAndMonitor(3.0);
        }
        
        stopAllMotors();
        ROS_INFO("✅ 동시 테스트 완료");
    }
    
    void directionTest() {
        ROS_INFO("🔧 방향 테스트 시작");
        
        // 전진
        ROS_INFO("⬆️ 전진 (5초)");
        setMotorSpeeds(8.0, 8.0);
        waitAndMonitor(5.0);
        
        stopAllMotors();
        waitAndMonitor(1.0);
        
        // 후진
        ROS_INFO("⬇️ 후진 (5초)");
        setMotorSpeeds(-8.0, -8.0);
        waitAndMonitor(5.0);
        
        stopAllMotors();
        waitAndMonitor(1.0);
        
        // 좌회전 (모터0 느리게, 모터1 빠르게)
        ROS_INFO("⬅️ 좌회전 (3초)");
        setMotorSpeeds(3.0, 8.0);
        waitAndMonitor(3.0);
        
        stopAllMotors();
        waitAndMonitor(1.0);
        
        // 우회전 (모터0 빠르게, 모터1 느리게)
        ROS_INFO("➡️ 우회전 (3초)");
        setMotorSpeeds(8.0, 3.0);
        waitAndMonitor(3.0);
        
        stopAllMotors();
        ROS_INFO("✅ 방향 테스트 완료");
    }
    
    void smoothControlTest() {
        ROS_INFO("🔧 부드러운 제어 테스트 시작");
        
        ROS_INFO("📈 점진적 가속 테스트");
        for (double speed = 0.0; speed <= 10.0; speed += 2.0) {
            ROS_INFO("속도 증가: %.1f", speed);
            setMotorSpeeds(speed, speed);
            waitAndMonitor(2.0);
        }
        
        ROS_INFO("📉 점진적 감속 테스트");
        for (double speed = 10.0; speed >= 0.0; speed -= 2.0) {
            ROS_INFO("속도 감소: %.1f", speed);
            setMotorSpeeds(speed, speed);
            waitAndMonitor(2.0);
        }
        
        ROS_INFO("🔄 방향 전환 테스트");
        setMotorSpeeds(8.0, 8.0);
        waitAndMonitor(3.0);
        
        setMotorSpeeds(-8.0, -8.0);
        waitAndMonitor(3.0);
        
        stopAllMotors();
        ROS_INFO("✅ 부드러운 제어 테스트 완료");
    }
    
    void stepSpeedTest() {
        ROS_INFO("🔧 속도 단계별 테스트 시작");
        
        std::vector<double> speeds = {1.0, 3.0, 5.0, 8.0, 12.0, 15.0};
        
        for (double speed : speeds) {
            ROS_INFO("🎯 속도 단계: %.1f (4초)", speed);
            setMotorSpeeds(speed, speed);
            waitAndMonitor(4.0);
            
            // 센서 신뢰도 체크
            if (sensor_confidence_ < 50.0) {
                ROS_WARN("⚠️ 센서 신뢰도 낮음 (%.0f%%) - 속도 조정 권장", sensor_confidence_);
            }
        }
        
        stopAllMotors();
        ROS_INFO("✅ 속도 단계별 테스트 완료");
    }
    
    void manualControlMode() {
        ROS_INFO("🎮 수동 제어 모드 시작");
        ROS_INFO("사용법: [모터0속도] [모터1속도] (예: 5.0 -3.0)");
        ROS_INFO("특수 명령: 'stop' (정지), 'back' (메뉴로 돌아가기)");
        
        std::string input;
        while (ros::ok()) {
            std::cout << "\n모터 속도 입력: ";
            std::getline(std::cin, input);
            
            if (input == "back") {
                break;
            } else if (input == "stop") {
                stopAllMotors();
                continue;
            }
            
            std::istringstream iss(input);
            double speed0, speed1;
            
            if (iss >> speed0 >> speed1) {
                setMotorSpeeds(speed0, speed1);
                ros::Duration(0.1).sleep();
                printStatus();
            } else {
                ROS_WARN("올바른 형식으로 입력하세요. 예: 5.0 -3.0");
            }
            
            ros::spinOnce();
        }
    }
    
    void waitAndMonitor(double duration) {
        ros::Time start_time = ros::Time::now();
        ros::Rate rate(10); // 10Hz
        
        while (ros::ok() && (ros::Time::now() - start_time).toSec() < duration) {
            ros::spinOnce();
            rate.sleep();
        }
        
        // 최종 상태 출력
        ros::spinOnce();
        ROS_INFO("📊 실제속도[%.1f, %.1f] 신뢰도:%.0f%%", 
                 actual_speed0_, actual_speed1_, sensor_confidence_);
    }
    
    void run() {
        int choice;
        
        while (ros::ok()) {
            printMenu();
            std::cin >> choice;
            std::cin.ignore(); // 개행 문자 제거
            
            switch (choice) {
                case 1:
                    basicTest();
                    break;
                case 2:
                    simultaneousTest();
                    break;
                case 3:
                    directionTest();
                    break;
                case 4:
                    smoothControlTest();
                    break;
                case 5:
                    stepSpeedTest();
                    break;
                case 6:
                    manualControlMode();
                    break;
                case 7:
                    printStatus();
                    break;
                case 0:
                    stopAllMotors();
                    ROS_INFO("🏁 테스트 종료");
                    return;
                default:
                    ROS_WARN("올바른 번호를 선택하세요 (0-7)");
                    break;
            }
            
            ros::spinOnce();
        }
    }
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "dual_motor_tester");
    
    DualMotorTester tester;
    tester.run();
    
    return 0;
} 