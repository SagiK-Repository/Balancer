#include "motor_controller/spi_controller.h"
#include <chrono>
#include <thread>
#include <stdexcept>
#include <iostream>
#include <fcntl.h>
#include <unistd.h>
#include <sys/mman.h>
#include <ros/ros.h>
#include <gpiod.h>

// Jetson Nano GPIO 직접 제어를 위한 메모리 맵핑 주소
#define GPIO_BASE 0x6000D000
#define GPIO_SIZE 0x1000

// GPIO 레지스터 오프셋
#define GPIO_CNF     0x00
#define GPIO_OE      0x10
#define GPIO_OUT     0x20
#define GPIO_IN      0x30
#define GPIO_INT_STA 0x40

// GPIO 핀을 레지스터 인덱스와 비트로 변환
#define GPIO_PORT(pin)   ((pin) / 32)
#define GPIO_BIT(pin)    ((pin) % 32)

class GPIOController {
public:
    static GPIOController& getInstance() {
        static GPIOController instance;
        return instance;
    }
    int mosi_pin;
    int miso_pin;
    int sclk_pin;
    int cpol;

    ~GPIOController() {
        if (initialized_) {
            gpiod_line_release(mosi_line);
            gpiod_line_release(miso_line);
            gpiod_line_release(sclk_line);
            gpiod_chip_close(chip);
            initialized_ = false;
        }
    }

    bool init(int mosi_pin_,int miso_pin_,int sclk_pin_, int cpol_) {
        if (initialized_) return true;

        mosi_pin = mosi_pin_;
        miso_pin = miso_pin_;
        sclk_pin = sclk_pin_;
        cpol = cpol_;
        // gpiochip0 열기
        chip = gpiod_chip_open("/dev/gpiochip0");
        if (!chip) {
            ROS_ERROR("Failed to open GPIO chip");
            return false;
        }

        // GPIO 라인 얻기
        mosi_line = gpiod_chip_get_line(chip, mosi_pin);
        miso_line = gpiod_chip_get_line(chip, miso_pin);
        sclk_line = gpiod_chip_get_line(chip, sclk_pin);

        if (!mosi_line || !miso_line || !sclk_line) {
            gpiod_chip_close(chip);
            ROS_ERROR("Failed to get GPIO lines");
        }

        // 라인 설정
        int ret = gpiod_line_request_output(mosi_line, "spi_mosi", 0);
        if (ret < 0) {
            gpiod_chip_close(chip);
            ROS_ERROR("Failed to request MOSI GPIO line as output");
        }

        ret = gpiod_line_request_input(miso_line, "spi_miso");
        if (ret < 0) {
            gpiod_line_release(mosi_line);
            gpiod_chip_close(chip);
            ROS_ERROR("Failed to request MISO GPIO line as input");
        }

        ret = gpiod_line_request_output(sclk_line, "spi_sclk", cpol);
        if (ret < 0) {
            gpiod_line_release(mosi_line);
            gpiod_line_release(miso_line);
            gpiod_chip_close(chip);
            ROS_ERROR("Failed to request SCLK GPIO line as output");
        }

        initialized_ = true;
        return true;
    }

    void setDirection(int pin, bool output) {
        if (!initialized_) return;
    }

    void writePin(int line, bool value) {
        if (!initialized_) return;

        if (line == mosi_pin) {
            gpiod_line_set_value(mosi_line, value ? 1 : 0);
        } else if (line == sclk_pin) {
            gpiod_line_set_value(sclk_line, value ? 1 : 0);
        }
    }

    bool readPin(int line) {
        if (!initialized_) return false;

        if (line == miso_pin) {
            return gpiod_line_get_value(miso_line) == 1;
        }
        return false;
    }

private:
    GPIOController() : fd_(-1), gpio_map_(MAP_FAILED), initialized_(false) {}
    GPIOController(const GPIOController&) = delete;
    GPIOController& operator=(const GPIOController&) = delete;

    int fd_;
    void* gpio_map_;
    bool initialized_;
    
    
    struct gpiod_chip* chip;
    struct gpiod_line* mosi_line;
    struct gpiod_line* miso_line;
    struct gpiod_line* sclk_line;
    
};

BitBangSPI::BitBangSPI(){
    return ;
}

BitBangSPI::BitBangSPI(int mosi_pin, int miso_pin, int sclk_pin,
                       SPIMode mode, uint32_t max_speed_hz, bool bit_order_lsb)
    : mosi_pin_(mosi_pin), miso_pin_(miso_pin), sclk_pin_(sclk_pin),
      mode_(mode), max_speed_hz_(max_speed_hz), bit_order_lsb_(bit_order_lsb),
      gpio_initialized_(false) {
    
    // 모드에 따른 CPOL, CPHA 설정
    cpol_ = (mode_ == MODE2 || mode_ == MODE3);
    cpha_ = (mode_ == MODE1 || mode_ == MODE3);
    
    initGPIO();
}

BitBangSPI::~BitBangSPI() {
    if (gpio_initialized_) {
        cleanupGPIO();
    }
}

void BitBangSPI::initGPIO() {
    auto& gpio = GPIOController::getInstance();
    if (!gpio.init(mosi_pin_,miso_pin_,sclk_pin_,cpol_)) {
        ROS_ERROR("Failed to initialize GPIO");
    }
    
    // 핀 방향 설정
    gpio.setDirection(mosi_pin_, true);  // 출력
    gpio.setDirection(miso_pin_, false); // 입력
    gpio.setDirection(sclk_pin_, true);  // 출력
    
    // 초기 상태 설정
    gpio.writePin(sclk_pin_, cpol_);  // CPOL에 따른 초기 클럭 레벨
    
    gpio_initialized_ = true;
}

void BitBangSPI::cleanupGPIO() {
    // GPIO 제어에서는 특별한 정리 작업이 필요 없음
    gpio_initialized_ = false;
}

void BitBangSPI::delayMicroseconds(unsigned int usec) {
    std::this_thread::sleep_for(std::chrono::microseconds(usec));
}

uint8_t BitBangSPI::transfer(uint8_t data_out) {
    auto& gpio = GPIOController::getInstance();
    uint8_t data_in = 0;
    
    // 반 클럭 주기 계산 (마이크로초)
    unsigned int half_period_us = 500000 / max_speed_hz_;
    if (half_period_us < 1) half_period_us = 0;
    
    // 비트별 처리
    for (int i = 0; i < 8; i++) {
        int bit_idx = bit_order_lsb_ ? i : (7 - i);
        bool bit_out = (data_out & (1 << bit_idx)) != 0;
        
        // MOSI에 비트 설정
        gpio.writePin(mosi_pin_, bit_out);
        
        // CPHA에 따른 첫 번째 클럭 에지
        if (cpha_) {
            delayMicroseconds(half_period_us);
            gpio.writePin(sclk_pin_, !cpol_);
            delayMicroseconds(half_period_us);
            
            // MISO에서 비트 읽기
            bool bit_in = gpio.readPin(miso_pin_);
            if (bit_in) {
                data_in |= (1 << bit_idx);
            }
            
            gpio.writePin(sclk_pin_, cpol_);
        } else {
            delayMicroseconds(half_period_us);
            gpio.writePin(sclk_pin_, !cpol_);
            
            // MISO에서 비트 읽기
            bool bit_in = gpio.readPin(miso_pin_);
            if (bit_in) {
                data_in |= (1 << bit_idx);
            }
            
            delayMicroseconds(half_period_us);
            gpio.writePin(sclk_pin_, cpol_);
        }
    }
    
    return data_in;
}

uint16_t BitBangSPI::transfer16(uint16_t data_out) {
    uint16_t data_in = 0;
    
    // MSB 우선 전송의 경우 (일반적인 SPI)
    if (!bit_order_lsb_) {
        // 상위 바이트 먼저 전송
        uint8_t msb = static_cast<uint8_t>((data_out >> 8) & 0xFF);
        uint8_t lsb = static_cast<uint8_t>(data_out & 0xFF);
        
        uint8_t msb_in = transfer(msb);
        uint8_t lsb_in = transfer(lsb);
        
        data_in = (static_cast<uint16_t>(msb_in) << 8) | lsb_in;
    } else {
        // LSB 우선 전송의 경우
        uint8_t lsb = static_cast<uint8_t>(data_out & 0xFF);
        uint8_t msb = static_cast<uint8_t>((data_out >> 8) & 0xFF);
        
        uint8_t lsb_in = transfer(lsb);
        uint8_t msb_in = transfer(msb);
        
        data_in = (static_cast<uint16_t>(msb_in) << 8) | lsb_in;
    }
    
    
    return data_in;
}

std::vector<uint8_t> BitBangSPI::transferBuffer(const std::vector<uint8_t>& data_out) {
    std::vector<uint8_t> data_in(data_out.size());
    
    // 각 바이트 전송
    for (size_t i = 0; i < data_out.size(); i++) {
        data_in[i] = transfer(data_out[i]);
    }
    
    return data_in;
}

void BitBangSPI::setMode(SPIMode mode) {
    mode_ = mode;
    cpol_ = (mode_ == MODE2 || mode_ == MODE3);
    cpha_ = (mode_ == MODE1 || mode_ == MODE3);
    
    // CPOL에 맞게 클럭 라인 초기화
    if (gpio_initialized_) {
        auto& gpio = GPIOController::getInstance();
        gpio.writePin(sclk_pin_, cpol_);
    }
}

void BitBangSPI::setMaxSpeed(uint32_t speed_hz) {
    max_speed_hz_ = speed_hz;
}
