#include "integrated_balancer/hardware_spi.h"
#include <cstring>
#include <stdexcept>

HardwareSPI::HardwareSPI(const std::string& device, uint32_t speed_hz, 
                         uint8_t mode, uint8_t bits_per_word)
    : device_(device), speed_hz_(speed_hz), mode_(mode), 
      bits_per_word_(bits_per_word), fd_(-1), initialized_(false) {
}

HardwareSPI::~HardwareSPI() {
    close();
}

bool HardwareSPI::initialize() {
    // SPI 디바이스 열기
    fd_ = open(device_.c_str(), O_RDWR);
    if (fd_ < 0) {
        return false;
    }
    
    // SPI 모드 설정
    if (ioctl(fd_, SPI_IOC_WR_MODE, &mode_) < 0) {
        close();
        return false;
    }
    
    // 비트 수 설정
    if (ioctl(fd_, SPI_IOC_WR_BITS_PER_WORD, &bits_per_word_) < 0) {
        close();
        return false;
    }
    
    // 속도 설정
    if (ioctl(fd_, SPI_IOC_WR_MAX_SPEED_HZ, &speed_hz_) < 0) {
        close();
        return false;
    }
    
    initialized_ = true;
    return true;
}

void HardwareSPI::close() {
    if (fd_ >= 0) {
        ::close(fd_);
        fd_ = -1;
    }
    initialized_ = false;
}

int HardwareSPI::transfer(uint8_t* tx, uint8_t* rx, int len) {
    if (!initialized_ || fd_ < 0) {
        return -1;
    }
    
    struct spi_ioc_transfer tr;
    memset(&tr, 0, sizeof(tr));
    
    tr.tx_buf = (unsigned long)tx;
    tr.rx_buf = (unsigned long)rx;
    tr.len = len;
    tr.speed_hz = speed_hz_;
    tr.bits_per_word = bits_per_word_;
    tr.delay_usecs = 0;
    tr.cs_change = 0;
    
    int ret = ioctl(fd_, SPI_IOC_MESSAGE(1), &tr);
    if (ret < 0) {
        return -1;
    }
    
    return len;
}

uint8_t HardwareSPI::transferByte(uint8_t data_out) {
    uint8_t data_in = 0;
    transfer(&data_out, &data_in, 1);
    return data_in;
}

uint16_t HardwareSPI::transfer16(uint16_t data_out) {
    uint8_t tx[2] = {
        static_cast<uint8_t>((data_out >> 8) & 0xFF),
        static_cast<uint8_t>(data_out & 0xFF)
    };
    uint8_t rx[2] = {0, 0};
    
    transfer(tx, rx, 2);
    
    return (static_cast<uint16_t>(rx[0]) << 8) | rx[1];
}

void HardwareSPI::setMode(uint8_t mode) {
    mode_ = mode;
    if (initialized_ && fd_ >= 0) {
        ioctl(fd_, SPI_IOC_WR_MODE, &mode_);
    }
}

void HardwareSPI::setSpeed(uint32_t speed_hz) {
    speed_hz_ = speed_hz;
    if (initialized_ && fd_ >= 0) {
        ioctl(fd_, SPI_IOC_WR_MAX_SPEED_HZ, &speed_hz_);
    }
}




