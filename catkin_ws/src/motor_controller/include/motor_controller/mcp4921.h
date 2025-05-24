// mcp4921.h
#ifndef MCP4921_H
#define MCP4921_H
#include "spi_controller.h"

class MCP4921 {
public:
    MCP4921(int mosi, int miso, int sclk, int cs, uint32_t spi_speed = 500000);
    ~MCP4921();
    
    // DAC 출력 설정 (0-4095)
    bool setOutput(uint16_t value, bool buffered = true, bool gain_1x = true, bool active = true);
    
    // 0-100% 범위로 출력 설정
    bool setOutputPercent(float percent, bool buffered = true, bool gain_1x = true);
    
    // 0-vref 볼트 범위로 출력 설정
    bool setOutputVoltage(float voltage, float vref = 5.0f, bool buffered = true, bool gain_1x = true);
    
    // 초기화 상태 확인
    bool isInitialized() const; private:
    BitBangSPI spi_;
    bool initialized_;
};
#endif // MCP4921_H
