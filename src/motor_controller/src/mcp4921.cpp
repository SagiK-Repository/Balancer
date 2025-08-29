// mcp4921.cpp
#include <ros/ros.h>
#include "motor_controller/mcp4921.h"
#include <iostream>
#include <cmath>
#include <chrono>
#include <iostream>
#include <unistd.h>
#include "gpiod.h"

MCP4921::MCP4921(BitBangSPI* _spi, int cs)
    : initialized_(false) {

    cs_pin = cs;
    // CS 사용 과정
    chip = gpiod_chip_open("/dev/gpiochip0");
    if (!chip) {ROS_ERROR("Failed to open GPIO chip");return;}
    cs_line = gpiod_chip_get_line(chip, cs_pin);
    if (!cs_line) {gpiod_chip_close(chip);ROS_ERROR("Failed to get GPIO lines");return;}
    int ret = gpiod_line_request_output(cs_line, "spi_cs", 1);  // CS는 HIGH로 시작
    if (ret < 0) {gpiod_chip_close(chip);ROS_ERROR("Failed to request CS GPIO line as output");return;}

    spi = _spi;

    initialized_ = true;//spi_.isOpen();
    
    if (initialized_) {
        ROS_INFO("MCP4921 initialized successfully");
    } else {
        ROS_ERROR("Failed to initialize MCP4921");
    }
    setOutput(4095);
}
MCP4921::~MCP4921() {
    // 종료 시 출력을 0으로 설정
    if (initialized_) {
        setOutput(0);
    }
}
bool MCP4921::isInitialized() const {
    return initialized_;
}
bool MCP4921::setOutput(uint16_t value, bool buffered, bool gain_1x, bool active) {
    if (!initialized_) {
        ROS_ERROR("MCP4921 not initialized");
        return false;
    }
    
    // 값 범위 제한 (0-4095, 12비트)
    value = value & 0x0FFF;
    //ROS_INFO("%d",value);
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
    
    // 2바이트 명령 전송
    uint8_t tx_data[2];
    tx_data[0] = (command >> 8) & 0xFF; // 상위 바이트
    tx_data[1] = command & 0xFF; // 하위 바이트
    
    // Chip Select 활성화 (LOW)
    gpiod_line_set_value(cs_line, false);

    int ret = spi->transfer16(command);

    // Chip Select 비활성화 (HIGH)
    gpiod_line_set_value(cs_line, true);
    
    return (ret > 0);
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
