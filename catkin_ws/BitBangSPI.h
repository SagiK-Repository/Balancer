#ifndef BITBANG_SPI_H
#define BITBANG_SPI_H

#include <cstdint>
#include <string>
#include <vector>

class BitBangSPI {
public:
    // SPI 모드 상수
    enum SPIMode {
        MODE0 = 0, // CPOL=0, CPHA=0
        MODE1 = 1, // CPOL=0, CPHA=1
        MODE2 = 2, // CPOL=1, CPHA=0
        MODE3 = 3  // CPOL=1, CPHA=1
    };

    /**
     * 생성자
     * @param mosi_pin MOSI GPIO 핀 번호 (BCM 번호 체계)
     * @param miso_pin MISO GPIO 핀 번호
     * @param sclk_pin SCLK GPIO 핀 번호
     * @param cs_pin CS GPIO 핀 번호
     * @param mode SPI 모드 (0-3)
     * @param max_speed_hz 최대 클럭 속도 (Hz)
     * @param bit_order 비트 순서 (true=LSB 먼저, false=MSB 먼저)
     */
    BitBangSPI(int mosi_pin, int miso_pin, int sclk_pin, int cs_pin,
               SPIMode mode = MODE0, uint32_t max_speed_hz = 1000000,
               bool bit_order_lsb = false);

    // 소멸자
    ~BitBangSPI();

    /**
     * 단일 바이트 전송 및 수신
     * @param data_out 전송할 바이트
     * @return 수신된 바이트
     */
    uint8_t transfer(uint8_t data_out);

    /**
     * 16비트 데이터 전송 및 수신
     * @param data_out 전송할 16비트 데이터
     * @return 수신된 16비트 데이터
     */
    uint16_t transfer16(uint16_t data_out);

    /**
     * 여러 바이트 전송 및 수신
     * @param data_out 전송할 데이터 배열
     * @return 수신된 데이터 배열
     */
    std::vector<uint8_t> transferBuffer(const std::vector<uint8_t>& data_out);

    /**
     * SPI 모드 설정
     * @param mode SPI 모드 (0-3)
     */
    void setMode(SPIMode mode);

    /**
     * SPI 속도 설정
     * @param speed_hz 속도 (Hz)
     */
    void setMaxSpeed(uint32_t speed_hz);

    /**
     * CS 핀 수동 제어
     * @param level 핀 레벨 (true=HIGH, false=LOW)
     */
    void setCS(bool level);

private:
    // GPIO 핀 번호
    int mosi_pin_;
    int miso_pin_;
    int sclk_pin_;
    int cs_pin_;
    
    // SPI 설정
    SPIMode mode_;
    uint32_t max_speed_hz_;
    bool bit_order_lsb_;
    
    // 내부 상태
    bool cpol_;  // 클럭 극성
    bool cpha_;  // 클럭 위상
    
    // GPIO 초기화 여부
    bool gpio_initialized_;
    
    // GPIO 초기화 함수
    void initGPIO();
    
    // GPIO 정리 함수
    void cleanupGPIO();
    
    // 마이크로초 단위 대기 함수
    void delayMicroseconds(unsigned int usec);
};

#endif // BITBANG_SPI_H