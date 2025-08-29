// led_blink_node.cpp 파일을 수정합니다

#include <ros/ros.h> 
#include <gpiod.h> 
#include <unistd.h> 
#include <string> 
#include <vector>

class LedBlinker { 
private:
    ros::NodeHandle nh_;
    ros::NodeHandle private_nh_;
    
    struct gpiod_chip *chip_;
    struct gpiod_line *line_;
    
    int led_pin_;
    double blink_rate_;
    
public:
    LedBlinker() : private_nh_("~") {
        // 파라미터 로드
        private_nh_.param("led_pin", led_pin_, 18);
        private_nh_.param("blink_rate", blink_rate_, 1.0);
        
        // GPIO 초기화 - 여러 칩 이름 시도
        std::vector<std::string> chip_names = {"gpiochip0", "gpiochip1", "tegra-gpio"};
        bool chip_opened = false;
        
        for (const auto& name : chip_names) {
            ROS_INFO("Trying to open GPIO chip: %s", name.c_str());
            chip_ = gpiod_chip_open_by_name(name.c_str());
            if (chip_) {
                chip_opened = true;
                ROS_INFO("Successfully opened GPIO chip: %s", name.c_str());
                break;
            }
        }
        
        if (!chip_opened) {
            ROS_ERROR("Could not open any GPIO chip");
            ros::shutdown();
            return;
        }
        
        line_ = gpiod_chip_get_line(chip_, led_pin_);
        if (!line_) {
            ROS_ERROR("Could not get GPIO line %d", led_pin_);
            gpiod_chip_close(chip_);
            ros::shutdown();
            return;
        }
        
        if (gpiod_line_request_output(line_, "led_blink", 0) < 0) {
            ROS_ERROR("Could not request GPIO line as output");
            gpiod_chip_close(chip_);
            ros::shutdown();
            return;
        }
        
        ROS_INFO("LED Blink node started: pin=%d, rate=%.1fHz", led_pin_, blink_rate_);
    }
    
    ~LedBlinker() {
        if (line_) {
            gpiod_line_release(line_);
        }
        if (chip_) {
            gpiod_chip_close(chip_);
        }
        ROS_INFO("GPIO resources cleaned up");
    }
    
    void run() {
        ros::Rate rate(blink_rate_ * 2); // Double for ON/OFF states
        int value = 0;
	ROS_INFO("Loop On");
        while (ros::ok()) {
            value = !value;
            gpiod_line_set_value(line_, value);
            rate.sleep();
	    
	    //ROS_INFO("LED IS %s",(value?"ON":"OFF"));
        }
    }
};
int main(int argc, char **argv) {
    ros::init(argc, argv, "led_blink_node");
    
    LedBlinker blinker;
    blinker.run();
    
    return 0;
}
