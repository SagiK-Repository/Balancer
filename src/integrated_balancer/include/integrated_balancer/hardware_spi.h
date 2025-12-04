#ifndef HARDWARE_SPI_H
#define HARDWARE_SPI_H

#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <linux/spi/spidev.h>
#include <cstdint>
#include <string>

/**
 * @brief Hardware SPI 클래스
 * 
 * Linux 커널 SPI 드라이버를 사용하는 하드웨어 SPI 통신 클래스
 * Bit-Banging 방식 대신 ioctl을 사용하여 고속 통신 제공
 */
class HardwareSPI {
public:
    /**
     * @brief 생성자
     * @param device SPI 디바이스 경로 (예: "/dev/spidev0.0")
     * @param speed_hz SPI 통신 속도 (Hz)
     * @param mode SPI 모드 (0-3)
     * @param bits_per_word 비트 수 (기본 8)
     */
    HardwareSPI(const std::string& device = "/dev/spidev0.0", 
                uint32_t speed_hz = 1000000,
                uint8_t mode = SPI_MODE_0,
                uint8_t bits_per_word = 8);
    
    /**
     * @brief 소멸자
     */
    ~HardwareSPI();
    
    /**
     * @brief SPI 초기화
     * @return 성공 여부
     */
    bool initialize();
    
    /**
     * @brief SPI 종료
     */
    void close();
    
    /**
     * @brief SPI 데이터 전송
     * @param tx 송신 데이터 버퍼
     * @param rx 수신 데이터 버퍼 (nullptr 가능)
     * @param len 데이터 길이 (바이트)
     * @return 전송된 바이트 수, 실패 시 -1
     */
    int transfer(uint8_t* tx, uint8_t* rx, int len);
    
    /**
     * @brief 단일 바이트 전송
     * @param data_out 송신 데이터
     * @return 수신 데이터
     */
    uint8_t transferByte(uint8_t data_out);
    
    /**
     * @brief 16비트 데이터 전송
     * @param data_out 송신 데이터
     * @return 수신 데이터
     */
    uint16_t transfer16(uint16_t data_out);
    
    /**
     * @brief SPI 모드 설정
     * @param mode SPI 모드 (0-3)
     */
    void setMode(uint8_t mode);
    
    /**
     * @brief SPI 속도 설정
     * @param speed_hz 속도 (Hz)
     */
    void setSpeed(uint32_t speed_hz);
    
    /**
     * @brief 초기화 상태 확인
     * @return 초기화 여부
     */
    bool isInitialized() const { return initialized_; }

private:
    std::string device_;
    uint32_t speed_hz_;
    uint8_t mode_;
    uint8_t bits_per_word_;
    int fd_;
    bool initialized_;
};

#endif // HARDWARE_SPI_H




