#include <ros/ros.h>
#include <std_msgs/Float32.h>
#include <iostream>
#include <fstream>
#include <vector>
#include <chrono>
#include <iomanip>
#include <cmath>
#include <sys/select.h>
#include <unistd.h>

class MotorResponseTest {
private:
    struct TestData {
        double timestamp;
        float target_speed0, target_speed1;
        float actual_speed0, actual_speed1;
        float roll_angle, pitch_angle;
        float sensor_confidence;
        float wheel_position0, wheel_position1;  // 1/20 바퀴 단위
        
        TestData() : timestamp(0), target_speed0(0), target_speed1(0),
                    actual_speed0(0), actual_speed1(0), roll_angle(0), pitch_angle(0),
                    sensor_confidence(0), wheel_position0(0), wheel_position1(0) {}
    };

public:
    MotorResponseTest() : nh_("~") {
        // 발행자 설정
        speed0_pub_ = nh_.advertise<std_msgs::Float32>("/target_speed0", 10);
        speed1_pub_ = nh_.advertise<std_msgs::Float32>("/target_speed1", 10);
        
        // 구독자 설정 - 실제 속도와 센서 데이터
        actual_speed0_sub_ = nh_.subscribe("/actual_speed0", 10, &MotorResponseTest::actualSpeed0Callback, this);
        actual_speed1_sub_ = nh_.subscribe("/actual_speed1", 10, &MotorResponseTest::actualSpeed1Callback, this);
        
        // 센서 데이터 구독 (motor_controller에서 발행하는 토픽들)
        sensor_roll_sub_ = nh_.subscribe("/sensor_roll", 10, &MotorResponseTest::sensorRollCallback, this);
        sensor_pitch_sub_ = nh_.subscribe("/sensor_pitch", 10, &MotorResponseTest::sensorPitchCallback, this);
        sensor_confidence_sub_ = nh_.subscribe("/sensor_confidence", 10, &MotorResponseTest::sensorConfidenceCallback, this);
        
        // 초기화
        actual_speed0_ = 0.0;
        actual_speed1_ = 0.0;
        current_roll_ = 0.0;
        current_pitch_ = 0.0;
        sensor_confidence_ = 0.0;
        
        test_running_ = false;
        test_start_time_ = ros::Time::now();
        
        ROS_INFO("=== Motor Response Test Node ===");
        ROS_INFO("Commands:");
        ROS_INFO("  1: Step Response Test (5%% -> 15%% -> 0%%)");
        ROS_INFO("  2: Ramp Response Test (0%% -> 20%% gradually)");
        ROS_INFO("  3: Oscillation Test (±10%% square wave)");
        ROS_INFO("  4: Micro Step Test (1/20 wheel increments)");
        ROS_INFO("  5: Sensor Impact Test (motor on/off comparison)");
        ROS_INFO("  s: Stop all tests");
        ROS_INFO("  q: Quit");
        
        showMenu();
    }
    
    void run() {
        ros::Rate rate(50);  // 50Hz for precise measurement
        
        while (ros::ok()) {
            processInput();
            
            if (test_running_) {
                updateTest();
            }
            
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
    ros::Subscriber sensor_roll_sub_;
    ros::Subscriber sensor_pitch_sub_;
    ros::Subscriber sensor_confidence_sub_;

    
    // 측정 데이터
    float actual_speed0_, actual_speed1_;
    float current_roll_, current_pitch_;
    float sensor_confidence_;
    
    // 테스트 상태
    bool test_running_;
    int current_test_;
    ros::Time test_start_time_;
    std::vector<TestData> test_data_;
    
    void actualSpeed0Callback(const std_msgs::Float32::ConstPtr& msg) {
        actual_speed0_ = msg->data;
    }
    
    void actualSpeed1Callback(const std_msgs::Float32::ConstPtr& msg) {
        actual_speed1_ = msg->data;
    }
    
    void sensorRollCallback(const std_msgs::Float32::ConstPtr& msg) {
        current_roll_ = msg->data;
    }
    
    void sensorPitchCallback(const std_msgs::Float32::ConstPtr& msg) {
        current_pitch_ = msg->data;
    }
    
    void sensorConfidenceCallback(const std_msgs::Float32::ConstPtr& msg) {
        sensor_confidence_ = msg->data;
    }
    
    // 센서 데이터는 이제 토픽으로 받아옴
    
    void publishSpeeds(float speed0, float speed1) {
        std_msgs::Float32 msg0, msg1;
        msg0.data = speed0;
        msg1.data = speed1;
        speed0_pub_.publish(msg0);
        speed1_pub_.publish(msg1);
    }
    
    void recordData(float target0, float target1) {
        TestData data;
        data.timestamp = (ros::Time::now() - test_start_time_).toSec();
        data.target_speed0 = target0;
        data.target_speed1 = target1;
        data.actual_speed0 = actual_speed0_;
        data.actual_speed1 = actual_speed1_;
        data.roll_angle = current_roll_;
        data.pitch_angle = current_pitch_;
        data.sensor_confidence = sensor_confidence_;
        
        // 바퀴 위치를 1/20 바퀴 단위로 계산 (20 steps per revolution)
        data.wheel_position0 = (actual_speed0_ * data.timestamp * 2 * M_PI / 60.0) / (2 * M_PI / 20.0);
        data.wheel_position1 = (actual_speed1_ * data.timestamp * 2 * M_PI / 60.0) / (2 * M_PI / 20.0);
        
        test_data_.push_back(data);
        
        // 실시간 출력
        std::cout << std::fixed << std::setprecision(2);
        std::cout << "T:" << data.timestamp << "s | ";
        std::cout << "Tgt:[" << target0 << "," << target1 << "] | ";
        std::cout << "Act:[" << actual_speed0_ << "," << actual_speed1_ << "] RPM | ";
        std::cout << "Pos:[" << data.wheel_position0 << "," << data.wheel_position1 << "]/20 | ";
        std::cout << "Tilt:[" << current_roll_ << "," << current_pitch_ << "]° | ";
        std::cout << "Conf:" << sensor_confidence_ << "%%\r" << std::flush;
    }
    
    void startTest(int test_number) {
        current_test_ = test_number;
        test_running_ = true;
        test_start_time_ = ros::Time::now();
        test_data_.clear();
        
        std::cout << "\n=== Starting Test " << test_number << " ===" << std::endl;
        
        switch (test_number) {
            case 1:
                std::cout << "Step Response Test: 0%% -> 5%% -> 15%% -> 0%%" << std::endl;
                break;
            case 2:
                std::cout << "Ramp Response Test: 0%% -> 20%% over 10 seconds" << std::endl;
                break;
            case 3:
                std::cout << "Oscillation Test: ±10%% square wave, 2Hz" << std::endl;
                break;
            case 4:
                std::cout << "Micro Step Test: Precise 1/20 wheel increments" << std::endl;
                break;
            case 5:
                std::cout << "Sensor Impact Test: Motor on/off sensor comparison" << std::endl;
                break;
        }
    }
    
    void updateTest() {
        double elapsed = (ros::Time::now() - test_start_time_).toSec();
        float target0 = 0.0, target1 = 0.0;
        
        switch (current_test_) {
            case 1: // Step Response Test
                if (elapsed < 2.0) {
                    target0 = 0.0;  // 2초간 정지
                } else if (elapsed < 5.0) {
                    target0 = 5.0;  // 3초간 5%% 출력
                } else if (elapsed < 8.0) {
                    target0 = 15.0; // 3초간 15%% 출력
                } else if (elapsed < 10.0) {
                    target0 = 0.0;  // 2초간 정지
                } else {
                    stopTest();
                    return;
                }
                break;
                
            case 2: // Ramp Response Test
                if (elapsed < 10.0) {
                    target0 = (elapsed / 10.0) * 20.0;  // 10초에 걸쳐 0->20%%
                } else if (elapsed < 15.0) {
                    target0 = 20.0 - ((elapsed - 10.0) / 5.0) * 20.0;  // 5초에 걸쳐 20->0%%
                } else {
                    stopTest();
                    return;
                }
                break;
                
            case 3: // Oscillation Test
                if (elapsed < 20.0) {
                    target0 = (std::sin(elapsed * 2 * M_PI * 2.0) > 0) ? 10.0 : -10.0;  // 2Hz 사각파
                } else {
                    stopTest();
                    return;
                }
                break;
                
            case 4: // Micro Step Test
                if (elapsed < 30.0) {
                    // 매 2초마다 1/20 바퀴씩 이동하도록 매우 낮은 속도
                    int step = (int)(elapsed / 2.0);
                    target0 = (step % 2 == 0) ? 1.0 : 0.0;  // 매우 낮은 속도로 스텝 이동
                } else {
                    stopTest();
                    return;
                }
                break;
                
            case 5: // Sensor Impact Test
                if (elapsed < 5.0) {
                    target0 = 0.0;  // 5초간 모터 정지 상태에서 센서 측정
                } else if (elapsed < 10.0) {
                    target0 = 10.0; // 5초간 모터 동작 상태에서 센서 측정
                } else if (elapsed < 15.0) {
                    target0 = 0.0;  // 다시 5초간 정지 상태
                } else {
                    stopTest();
                    return;
                }
                break;
        }
        
        publishSpeeds(target0, target1);
        recordData(target0, target1);
    }
    
    void stopTest() {
        test_running_ = false;
        publishSpeeds(0.0, 0.0);
        
        std::cout << "\n=== Test Completed ===" << std::endl;
        saveTestData();
        analyzeResults();
        showMenu();
    }
    
    void saveTestData() {
        std::string filename = "motor_test_" + std::to_string(current_test_) + "_" + 
                              std::to_string((int)ros::Time::now().toSec()) + ".csv";
        
        std::ofstream file(filename);
        if (!file.is_open()) {
            ROS_ERROR("Failed to open file: %s", filename.c_str());
            return;
        }
        
        // CSV 헤더
        file << "Time,Target0,Target1,Actual0,Actual1,Roll,Pitch,Confidence,Pos0,Pos1\n";
        
        // 데이터 저장
        for (const auto& data : test_data_) {
            file << std::fixed << std::setprecision(4);
            file << data.timestamp << ","
                 << data.target_speed0 << "," << data.target_speed1 << ","
                 << data.actual_speed0 << "," << data.actual_speed1 << ","
                 << data.roll_angle << "," << data.pitch_angle << ","
                 << data.sensor_confidence << ","
                 << data.wheel_position0 << "," << data.wheel_position1 << "\n";
        }
        
        file.close();
        ROS_INFO("Test data saved to: %s", filename.c_str());
    }
    
    void analyzeResults() {
        if (test_data_.empty()) return;
        
        std::cout << "\n=== Test Analysis ===" << std::endl;
        
        // 응답 시간 분석
        float max_target = 0.0;
        float response_time = 0.0;
        bool found_90_percent = false;
        
        for (const auto& data : test_data_) {
            if (std::abs(data.target_speed0) > max_target) {
                max_target = std::abs(data.target_speed0);
            }
            
            if (!found_90_percent && max_target > 0) {
                float response_threshold = max_target * 0.9f;
                if (std::abs(data.actual_speed0) >= response_threshold) {
                    response_time = data.timestamp;
                    found_90_percent = true;
                }
            }
        }
        
        // 센서 안정성 분석
        float min_confidence = 100.0f, max_confidence = 0.0f;
        float avg_confidence = 0.0f;
        int confidence_samples = 0;
        
        for (const auto& data : test_data_) {
            if (data.sensor_confidence > 0) {
                min_confidence = std::min(min_confidence, data.sensor_confidence);
                max_confidence = std::max(max_confidence, data.sensor_confidence);
                avg_confidence += data.sensor_confidence;
                confidence_samples++;
            }
        }
        
        if (confidence_samples > 0) {
            avg_confidence /= confidence_samples;
        }
        
        // 결과 출력
        std::cout << "Response Time (90%%): " << response_time << " seconds" << std::endl;
        std::cout << "Max Target Speed: " << max_target << " RPM" << std::endl;
        std::cout << "Sensor Confidence - Min: " << min_confidence 
                  << "%%, Max: " << max_confidence 
                  << "%%, Avg: " << avg_confidence << "%%" << std::endl;
        
        // 1/20 바퀴 정밀도 분석 (테스트 4의 경우)
        if (current_test_ == 4) {
            std::cout << "Micro Step Analysis:" << std::endl;
            for (size_t i = 1; i < test_data_.size(); i++) {
                float pos_change = test_data_[i].wheel_position0 - test_data_[i-1].wheel_position0;
                if (std::abs(pos_change) > 0.01) {  // 의미있는 변화만
                    std::cout << "  Step at " << test_data_[i].timestamp 
                              << "s: " << pos_change << "/20 wheel" << std::endl;
                }
            }
        }
    }
    
    void showMenu() {
        std::cout << "\n=== Motor Response Test Menu ===" << std::endl;
        std::cout << "Select test (1-5) or command (s/q): ";
    }
    
    void processInput() {
        fd_set readfds;
        struct timeval timeout;
        
        FD_ZERO(&readfds);
        FD_SET(STDIN_FILENO, &readfds);
        timeout.tv_sec = 0;
        timeout.tv_usec = 1000;  // 1ms timeout
        
        if (select(STDIN_FILENO + 1, &readfds, NULL, NULL, &timeout) > 0) {
            char input;
            if (read(STDIN_FILENO, &input, 1) > 0) {
                switch (input) {
                    case '1':
                    case '2':
                    case '3':
                    case '4':
                    case '5':
                        if (!test_running_) {
                            startTest(input - '0');
                        }
                        break;
                    case 's':
                        if (test_running_) {
                            stopTest();
                        }
                        break;
                    case 'q':
                        publishSpeeds(0.0, 0.0);
                        ros::shutdown();
                        break;
                    default:
                        if (!test_running_) {
                            showMenu();
                        }
                        break;
                }
            }
        }
    }
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "motor_response_test");
    
    MotorResponseTest test;
    test.run();
    
    return 0;
} 