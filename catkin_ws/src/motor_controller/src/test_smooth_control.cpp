#include <ros/ros.h>
#include <std_msgs/Float32.h>
#include <iostream>
#include <string>
#include <iomanip>
#include <cmath>
#include <sys/select.h>
#include <unistd.h>

class SmoothControlTest {
public:
    SmoothControlTest() : nh_("~") {
        // 발행자 설정
        speed0_pub_ = nh_.advertise<std_msgs::Float32>("/target_speed0", 10);
        speed1_pub_ = nh_.advertise<std_msgs::Float32>("/target_speed1", 10);
        
        // 구독자 설정 - 실제 속도와 센서 데이터
        actual_speed0_sub_ = nh_.subscribe("/actual_speed0", 10, &SmoothControlTest::actualSpeed0Callback, this);
        actual_speed1_sub_ = nh_.subscribe("/actual_speed1", 10, &SmoothControlTest::actualSpeed1Callback, this);
        sensor_confidence_sub_ = nh_.subscribe("/sensor_confidence", 10, &SmoothControlTest::sensorConfidenceCallback, this);
        
        // 초기값 설정
        actual_speed0_ = 0.0;
        actual_speed1_ = 0.0;
        sensor_confidence_ = 0.0;
        
        ROS_INFO("=== 부드러운 모터 제어 테스트 ===");
        ROS_INFO("명령어:");
        ROS_INFO("  1: 부드러운 가속 테스트 (0%% -> 30%% -> 0%%)");
        ROS_INFO("  2: 계단 응답 테스트 (급격한 변화)");
        ROS_INFO("  3: 사인파 테스트 (부드러운 진동)");
        ROS_INFO("  4: 방향 전환 테스트 (+20%% -> -20%% -> 0%%)");
        ROS_INFO("  5: 미세 조정 테스트 (작은 변화들)");
        ROS_INFO("  q: 종료");
        ROS_INFO("명령을 입력하세요:");
    }
    
    void run() {
        ros::Rate rate(10);  // 10Hz
        
        while (ros::ok()) {
            // 키보드 입력 확인
            if (kbhit()) {
                char key = getchar();
                handleKeyInput(key);
            }
            
            // 상태 출력
            printStatus();
            
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
    ros::Subscriber sensor_confidence_sub_;
    
    float actual_speed0_;
    float actual_speed1_;
    float sensor_confidence_;
    
    void actualSpeed0Callback(const std_msgs::Float32::ConstPtr& msg) {
        actual_speed0_ = msg->data;
    }
    
    void actualSpeed1Callback(const std_msgs::Float32::ConstPtr& msg) {
        actual_speed1_ = msg->data;
    }
    
    void sensorConfidenceCallback(const std_msgs::Float32::ConstPtr& msg) {
        sensor_confidence_ = msg->data;
    }
    
    void handleKeyInput(char key) {
        switch (key) {
            case '1':
                runSmoothAccelTest();
                break;
            case '2':
                runStepResponseTest();
                break;
            case '3':
                runSineWaveTest();
                break;
            case '4':
                runDirectionChangeTest();
                break;
            case '5':
                runFineAdjustmentTest();
                break;
            case 'q':
            case 'Q':
                ROS_INFO("테스트 종료");
                ros::shutdown();
                break;
            default:
                ROS_INFO("알 수 없는 명령: %c", key);
                break;
        }
    }
    
    void runSmoothAccelTest() {
        ROS_INFO("=== 부드러운 가속 테스트 시작 ===");
        
        // 0% -> 30% (5초에 걸쳐)
        ROS_INFO("0%% -> 30%% 가속 중...");
        publishSpeed(30.0, 30.0);
        ros::Duration(5.0).sleep();
        
        // 30% 유지 (3초)
        ROS_INFO("30%% 유지 중...");
        ros::Duration(3.0).sleep();
        
        // 30% -> 0% (3초에 걸쳐)
        ROS_INFO("30%% -> 0%% 감속 중...");
        publishSpeed(0.0, 0.0);
        ros::Duration(3.0).sleep();
        
        ROS_INFO("부드러운 가속 테스트 완료");
    }
    
    void runStepResponseTest() {
        ROS_INFO("=== 계단 응답 테스트 시작 ===");
        
        // 급격한 변화들
        publishSpeed(0.0, 0.0);
        ros::Duration(2.0).sleep();
        
        publishSpeed(25.0, 25.0);
        ros::Duration(2.0).sleep();
        
        publishSpeed(0.0, 0.0);
        ros::Duration(2.0).sleep();
        
        publishSpeed(-15.0, -15.0);
        ros::Duration(2.0).sleep();
        
        publishSpeed(0.0, 0.0);
        ros::Duration(2.0).sleep();
        
        ROS_INFO("계단 응답 테스트 완료");
    }
    
    void runSineWaveTest() {
        ROS_INFO("=== 사인파 테스트 시작 (20초) ===");
        
        ros::Time start_time = ros::Time::now();
        ros::Rate rate(20);  // 20Hz
        
        while ((ros::Time::now() - start_time).toSec() < 20.0 && ros::ok()) {
            double elapsed = (ros::Time::now() - start_time).toSec();
            double amplitude = 20.0;  // ±20%
            double frequency = 0.2;   // 0.2Hz (5초 주기)
            
            double target = amplitude * sin(2 * M_PI * frequency * elapsed);
            publishSpeed(target, target);
            
            rate.sleep();
        }
        
        publishSpeed(0.0, 0.0);
        ROS_INFO("사인파 테스트 완료");
    }
    
    void runDirectionChangeTest() {
        ROS_INFO("=== 방향 전환 테스트 시작 ===");
        
        // +20%
        ROS_INFO("정방향 20%% 이동...");
        publishSpeed(20.0, 20.0);
        ros::Duration(4.0).sleep();
        
        // -20%
        ROS_INFO("역방향 20%% 이동...");
        publishSpeed(-20.0, -20.0);
        ros::Duration(4.0).sleep();
        
        // 0%
        ROS_INFO("정지...");
        publishSpeed(0.0, 0.0);
        ros::Duration(2.0).sleep();
        
        ROS_INFO("방향 전환 테스트 완료");
    }
    
    void runFineAdjustmentTest() {
        ROS_INFO("=== 미세 조정 테스트 시작 ===");
        
        float targets[] = {0.0, 2.0, 5.0, 3.0, 8.0, 1.0, 0.0};
        int num_targets = sizeof(targets) / sizeof(targets[0]);
        
        for (int i = 0; i < num_targets; i++) {
            ROS_INFO("목표: %.1f%%", targets[i]);
            publishSpeed(targets[i], targets[i]);
            ros::Duration(3.0).sleep();
        }
        
        ROS_INFO("미세 조정 테스트 완료");
    }
    
    void publishSpeed(float speed0, float speed1) {
        std_msgs::Float32 msg0, msg1;
        msg0.data = speed0;
        msg1.data = speed1;
        speed0_pub_.publish(msg0);
        speed1_pub_.publish(msg1);
    }
    
    void printStatus() {
        std::cout << "\\r목표: [입력대기] | 실제: [" 
                  << std::fixed << std::setprecision(1) 
                  << actual_speed0_ << "," << actual_speed1_ 
                  << "] RPM | 센서신뢰도: " << sensor_confidence_ << "%" 
                  << std::flush;
    }
    
    // 키보드 입력 확인 (논블로킹)
    bool kbhit() {
        struct timeval tv = {0L, 0L};
        fd_set fds;
        FD_ZERO(&fds);
        FD_SET(0, &fds);
        return select(1, &fds, NULL, NULL, &tv) > 0;
    }
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "test_smooth_control");
    
    SmoothControlTest test;
    test.run();
    
    return 0;
} 