#include "integrated_balancer/mcp4921.h"

MCP4921::MCP4921(HardwareSPI* spi, int cs_pin)
    : spi_(spi), cs_pin_(cs_pin), initialized_(false), chip_(nullptr), cs_line_(nullptr) {
    
    if (initializeGPIO()) {
        initialized_ = true;
        setOutput(0);  // 초기 출력을 0으로 설정
    }
}

MCP4921::~MCP4921() {
    // 종료 시 출력을 0으로 설정
    if (initialized_) {
        setOutput(0);
    }
    cleanupGPIO();
}

bool MCP4921::initializeGPIO() {
    chip_ = gpiod_chip_open("/dev/gpiochip0");
    if (!chip_) {
        return false;
    }
    
    cs_line_ = gpiod_chip_get_line(chip_, cs_pin_);
    if (!cs_line_) {
        gpiod_chip_close(chip_);
        chip_ = nullptr;
        return false;
    }
    
    int ret = gpiod_line_request_output(cs_line_, "mcp4921_cs", 1);  // CS는 HIGH로 시작
    if (ret < 0) {
        gpiod_line_release(cs_line_);
        gpiod_chip_close(chip_);
        cs_line_ = nullptr;
        chip_ = nullptr;
        return false;
    }
    
    return true;
}

void MCP4921::cleanupGPIO() {
    if (cs_line_) {
        gpiod_line_release(cs_line_);
        cs_line_ = nullptr;
    }
    if (chip_) {
        gpiod_chip_close(chip_);
        chip_ = nullptr;
    }
}

bool MCP4921::setOutput(uint16_t value, bool buffered, bool gain_1x, bool active) {
    if (!initialized_ || !spi_ || !spi_->isInitialized()) {
        return false;
    }
    
    // 값 범위 제한 (0-4095, 12비트)
    value = value & 0x0FFF;
    
    // 명령 바이트 구성
    // Bit 15: Always 0 (DAC A selected)
    // Bit 14: 1 = Buffered, 0 = Unbuffered
    // Bit 13: 1 = ~Gain = 1, 0 = ~Gain = 2
    // Bit 12: 1 = Active, 0 = Shutdown the device
    // Bit 11-0: 12-bit DAC value
    uint16_t command = value;
    
    if (buffered) command |= 0x4000; // Bit 14 = 1
    if (gain_1x) command |= 0x2000; // Bit 13 = 1
    if (active) command |= 0x1000; // Bit 12 = 1
    
    // Chip Select 활성화 (LOW)
    if (cs_line_) {
        gpiod_line_set_value(cs_line_, 0);
    }
    
    // 2바이트 명령 전송
    spi_->transfer16(command);
    
    // Chip Select 비활성화 (HIGH)
    if (cs_line_) {
        gpiod_line_set_value(cs_line_, 1);
    }
    
    return true;
}

bool MCP4921::setOutputPercent(float percent, bool buffered, bool gain_1x) {
    // 0-100% 범위를 0-4095 범위로 변환
    if (percent < 0.0f) percent = 0.0f;
    if (percent > 100.0f) percent = 100.0f;
    
    uint16_t value = static_cast<uint16_t>(percent * 40.95f);
    return setOutput(value, buffered, gain_1x);
}

bool MCP4921::setOutputVoltage(float voltage, float vref, bool buffered, bool gain_1x) {
    // 최대 출력 전압 계산
    float max_voltage = gain_1x ? vref : vref / 2.0f;
    
    // 전압 제한
    if (voltage < 0.0f) voltage = 0.0f;
    if (voltage > max_voltage) voltage = max_voltage;
    
    // 전압을 0-4095 범위로 변환
    uint16_t value = static_cast<uint16_t>((voltage / max_voltage) * 4095.0f);
    return setOutput(value, buffered, gain_1x);
}




