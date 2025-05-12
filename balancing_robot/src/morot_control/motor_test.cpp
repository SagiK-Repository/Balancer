#include <ros/ros.h>
#include <iostream>

// Jetson GPIO를 시스템 콜로 제어
void set_gpio(int pin, bool value) {
    std::ofstream export_file("/sys/class/gpio/export");
    export_file << pin;
    export_file.close();
    
    std::string direction_path = "/sys/class/gpio/gpio" + std::to_string(pin) + "/direction";
    std::ofstream direction_file(direction_path);
    direction_file << "out";
    direction_file.close();
    
    std::string value_path = "/sys/class/gpio/gpio" + std::to_string(pin) + "/value";
    std::ofstream value_file(value_path);
    value_file << (value ? "1" : "0");
    value_file.close();
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "motor_test");
    ros::NodeHandle nh;
    
    ROS_INFO("Motor Test Started");
    
    // 테스트 코드
    int test_pin = 161; // J41 핀 7번
    for(int i = 0; i < 5; i++) {
        set_gpio(test_pin, true);
        ros::Duration(0.5).sleep();
        set_gpio(test_pin, false);
        ros::Duration(0.5).sleep();
    }
    
    return 0;
}