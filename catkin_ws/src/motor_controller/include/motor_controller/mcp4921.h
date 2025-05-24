// mcp4921.h
#ifndef MCP4921_H
#define MCP4921_H
#include "spi_controller.h"

class MCP4921 {
public:
    MCP4921(BitBangSPI *p_spi, int cs);
    ~MCP4921();
    
    // DAC 출력 설정 (0-4095)
    bool setOutput(uint16_t value, bool buffered = true, bool gain_1x = true, bool active = true);
    
    // 0-100% 범위로 출력 설정
    bool setOutputPercent(float percent, bool buffered = true, bool gain_1x = true);
    
    // 0-vref 볼트 범위로 출력 설정
    bool setOutputVoltage(float voltage, float vref = 5.0f, bool buffered = true, bool gain_1x = true);
    
    // 초기화 상태 확인
    bool isInitialized() const; private:
    BitBangSPI *spi;
    bool initialized_;

    int cs_pin;
    struct gpiod_chip *chip;
    struct gpiod_line* cs_line;
};
#endif // MCP4921_H
/*

        cs_pin = cs_pin_;
        
        // gpiochip0 열기
        chip = gpiod_chip_open("/dev/gpiochip0");
        if (!chip) {
            ROS_ERROR("Failed to open GPIO chip");
            return false;
        }


        cs_line = gpiod_chip_get_line(chip, cs_pin);
        if (!cs_line) {
            gpiod_chip_close(chip);
            ROS_ERROR("Failed to get GPIO lines");
        }

    ret = gpiod_line_request_output(cs_line, "spi_cs", 1);  // CS는 HIGH로 시작
    if (ret < 0) {
        gpiod_line_release(mosi_line);
        gpiod_line_release(miso_line);
        gpiod_line_release(sclk_line);
        gpiod_chip_close(chip);
        ROS_ERROR("Failed to request CS GPIO line as output");
    }

    int cs_pin_;



    void BitBangSPI::setCS(bool level) {
        if (gpio_initialized_) {
            auto& gpio = GPIOController::getInstance();
            gpio.writePin(cs_pin_, level);
        }
    }
    gpio.writePin(cs_pin_, true);     // CS는 기본적으로 비활성화(HIGH)
    gpiod_line_set_value(cs_line, value ? 1 : 0);
    


    
    // Chip Select 활성화 (LOW)
    gpiod_line_set_value(cs_line, false);

    // Chip Select 비활성화 (HIGH)
    gpiod_line_set_value(cs_line, true);
*/