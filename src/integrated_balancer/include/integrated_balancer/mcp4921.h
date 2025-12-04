#ifndef MCP4921_H
#define MCP4921_H

#include "integrated_balancer/hardware_spi.h"
#include <gpiod.h>
#include <cstdint>

/**
 * @brief MCP4921 DAC 클래스
 * 
 * HardwareSPI를 사용하는 MCP4921 DAC 제어 클래스
 */
class MCP4921 {
public:
    /**
     * @brief 생성자
     * @param spi HardwareSPI 포인터
     * @param cs_pin CS 핀 번호
     */
    MCP4921(HardwareSPI* spi, int cs_pin);
    
    /**
     * @brief 소멸자
     */
    ~MCP4921();
    
    /**
     * @brief 초기화 상태 확인
     */
    bool isInitialized() const { return initialized_; }
    
    /**
     * @brief 출력 설정
     * @param value 12비트 값 (0-4095)
     * @param buffered 버퍼링 여부
     * @param gain_1x 게인 1x 여부
     * @param active 활성화 여부
     * @return 성공 여부
     */
    bool setOutput(uint16_t value, bool buffered = true, bool gain_1x = true, bool active = true);
    
    /**
     * @brief 퍼센트 출력 설정
     * @param percent 0-100%
     * @param buffered 버퍼링 여부
     * @param gain_1x 게인 1x 여부
     * @return 성공 여부
     */
    bool setOutputPercent(float percent, bool buffered = true, bool gain_1x = true);
    
    /**
     * @brief 전압 출력 설정
     * @param voltage 출력 전압 (V)
     * @param vref 기준 전압 (V)
     * @param buffered 버퍼링 여부
     * @param gain_1x 게인 1x 여부
     * @return 성공 여부
     */
    bool setOutputVoltage(float voltage, float vref = 3.3f, bool buffered = true, bool gain_1x = true);

private:
    HardwareSPI* spi_;
    int cs_pin_;
    bool initialized_;
    
    struct gpiod_chip* chip_;
    struct gpiod_line* cs_line_;
    
    bool initializeGPIO();
    void cleanupGPIO();
};

#endif // MCP4921_H




