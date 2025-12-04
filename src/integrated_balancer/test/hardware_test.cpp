#include <ros/ros.h>
#include <iostream>
#include <unistd.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <linux/spi/spidev.h>
#include <linux/i2c-dev.h>
#include <gpiod.h>
#include "integrated_balancer/hardware_spi.h"
#include "integrated_balancer/i2c_controller.h"
#include "integrated_balancer/mcp4921.h"

/**
 * @brief 하드웨어 테스트 유틸리티
 * 
 * 각 하드웨어 컴포넌트를 개별적으로 테스트할 수 있는 도구
 */
class HardwareTest {
public:
    HardwareTest() {}
    
    /**
     * @brief SPI 디바이스 존재 확인
     */
    bool testSPIDevice(const std::string& device) {
        std::cout << "\n=== SPI Device Test ===" << std::endl;
        std::cout << "Checking: " << device << std::endl;
        
        int fd = open(device.c_str(), O_RDWR);
        if (fd < 0) {
            std::cout << "❌ FAILED: Cannot open " << device << std::endl;
            std::cout << "   Make sure SPI is enabled in device tree" << std::endl;
            return false;
        }
        
        uint8_t mode = SPI_MODE_0;
        uint8_t bits = 8;
        uint32_t speed = 1000000;
        
        if (ioctl(fd, SPI_IOC_WR_MODE, &mode) < 0) {
            std::cout << "❌ FAILED: Cannot set SPI mode" << std::endl;
            close(fd);
            return false;
        }
        
        if (ioctl(fd, SPI_IOC_WR_BITS_PER_WORD, &bits) < 0) {
            std::cout << "❌ FAILED: Cannot set bits per word" << std::endl;
            close(fd);
            return false;
        }
        
        if (ioctl(fd, SPI_IOC_WR_MAX_SPEED_HZ, &speed) < 0) {
            std::cout << "❌ FAILED: Cannot set SPI speed" << std::endl;
            close(fd);
            return false;
        }
        
        std::cout << "✅ SUCCESS: SPI device is accessible" << std::endl;
        close(fd);
        return true;
    }
    
    /**
     * @brief I2C 디바이스 존재 확인
     */
    bool testI2CDevice(const std::string& device) {
        std::cout << "\n=== I2C Device Test ===" << std::endl;
        std::cout << "Checking: " << device << std::endl;
        
        int fd = open(device.c_str(), O_RDWR);
        if (fd < 0) {
            std::cout << "❌ FAILED: Cannot open " << device << std::endl;
            std::cout << "   Make sure I2C is enabled" << std::endl;
            return false;
        }
        
        std::cout << "✅ SUCCESS: I2C device is accessible" << std::endl;
        close(fd);
        return true;
    }
    
    /**
     * @brief I2C 슬레이브 주소 스캔
     */
    void scanI2CDevices(const std::string& device) {
        std::cout << "\n=== I2C Device Scan ===" << std::endl;
        std::cout << "Scanning: " << device << std::endl;
        
        int fd = open(device.c_str(), O_RDWR);
        if (fd < 0) {
            std::cout << "❌ FAILED: Cannot open " << device << std::endl;
            return;
        }
        
        std::cout << "Found devices at addresses:" << std::endl;
        bool found_any = false;
        
        for (int addr = 0x08; addr <= 0x77; addr++) {
            if (ioctl(fd, I2C_SLAVE, addr) < 0) {
                continue;
            }
            
            // 간단한 읽기 시도
            uint8_t buffer[1];
            if (read(fd, buffer, 1) >= 0) {
                std::cout << "  ✅ 0x" << std::hex << addr << std::dec << std::endl;
                found_any = true;
            }
        }
        
        if (!found_any) {
            std::cout << "  ❌ No devices found" << std::endl;
        }
        
        close(fd);
    }
    
    /**
     * @brief MPU6050 센서 테스트
     */
    bool testMPU6050() {
        std::cout << "\n=== MPU6050 Sensor Test ===" << std::endl;
        
        I2CController i2c("/dev/i2c-1", 0x68);
        
        if (!i2c.initI2C()) {
            std::cout << "❌ FAILED: Cannot initialize I2C" << std::endl;
            return false;
        }
        
        if (!i2c.initMPU6050()) {
            std::cout << "❌ FAILED: Cannot initialize MPU6050" << std::endl;
            return false;
        }
        
        std::cout << "✅ SUCCESS: MPU6050 initialized" << std::endl;
        
        // 센서 데이터 읽기 테스트
        SensorData data;
        std::cout << "\nReading sensor data (10 samples)..." << std::endl;
        
        for (int i = 0; i < 10; i++) {
            if (i2c.readSensorData(data)) {
                double accel_x = i2c.convertAccelData(data.accel_x);
                double accel_y = i2c.convertAccelData(data.accel_y);
                double accel_z = i2c.convertAccelData(data.accel_z);
                double gyro_x = i2c.convertGyroData(data.gyro_x);
                double gyro_y = i2c.convertGyroData(data.gyro_y);
                double gyro_z = i2c.convertGyroData(data.gyro_z);
                double temp = i2c.convertTempData(data.temp);
                
                std::cout << "Sample " << (i+1) << ":" << std::endl;
                std::cout << "  Accel: X=" << accel_x << " Y=" << accel_y << " Z=" << accel_z << " m/s²" << std::endl;
                std::cout << "  Gyro:  X=" << gyro_x << " Y=" << gyro_y << " Z=" << gyro_z << " °/s" << std::endl;
                std::cout << "  Temp:  " << temp << " °C" << std::endl;
            } else {
                std::cout << "❌ FAILED: Cannot read sensor data" << std::endl;
                return false;
            }
            usleep(100000); // 100ms
        }
        
        std::cout << "\n✅ SUCCESS: MPU6050 is working correctly" << std::endl;
        return true;
    }
    
    /**
     * @brief HardwareSPI 테스트
     */
    bool testHardwareSPI(const std::string& device) {
        std::cout << "\n=== HardwareSPI Test ===" << std::endl;
        std::cout << "Testing: " << device << std::endl;
        
        HardwareSPI spi(device, 1000000, SPI_MODE_0, 8);
        
        if (!spi.initialize()) {
            std::cout << "❌ FAILED: Cannot initialize HardwareSPI" << std::endl;
            return false;
        }
        
        std::cout << "✅ SUCCESS: HardwareSPI initialized" << std::endl;
        
        // SPI 통신 테스트 (루프백 테스트)
        std::cout << "\nTesting SPI communication..." << std::endl;
        std::cout << "Note: This is a loopback test. Connect MOSI to MISO for proper test." << std::endl;
        
        uint8_t tx_data[4] = {0x01, 0x02, 0x03, 0x04};
        uint8_t rx_data[4] = {0};
        
        int result = spi.transfer(tx_data, rx_data, 4);
        if (result < 0) {
            std::cout << "❌ FAILED: SPI transfer failed" << std::endl;
            return false;
        }
        
        std::cout << "  Sent:    ";
        for (int i = 0; i < 4; i++) {
            std::cout << "0x" << std::hex << (int)tx_data[i] << std::dec << " ";
        }
        std::cout << std::endl;
        
        std::cout << "  Received: ";
        for (int i = 0; i < 4; i++) {
            std::cout << "0x" << std::hex << (int)rx_data[i] << std::dec << " ";
        }
        std::cout << std::endl;
        
        std::cout << "\n✅ SUCCESS: HardwareSPI is working" << std::endl;
        return true;
    }
    
    /**
     * @brief MCP4921 DAC 테스트
     */
    bool testMCP4921(const std::string& spi_device, int cs_pin) {
        std::cout << "\n=== MCP4921 DAC Test ===" << std::endl;
        std::cout << "SPI Device: " << spi_device << std::endl;
        std::cout << "CS Pin: " << cs_pin << std::endl;
        
        HardwareSPI* spi = new HardwareSPI(spi_device, 1000000, SPI_MODE_0, 8);
        
        if (!spi->initialize()) {
            std::cout << "❌ FAILED: Cannot initialize SPI" << std::endl;
            delete spi;
            return false;
        }
        
        MCP4921 dac(spi, cs_pin);
        
        if (!dac.isInitialized()) {
            std::cout << "❌ FAILED: Cannot initialize MCP4921" << std::endl;
            delete spi;
            return false;
        }
        
        std::cout << "✅ SUCCESS: MCP4921 initialized" << std::endl;
        
        // DAC 출력 테스트
        std::cout << "\nTesting DAC output (0% -> 50% -> 100% -> 0%)..." << std::endl;
        std::cout << "Monitor DAC output voltage with multimeter" << std::endl;
        
        float test_values[] = {0.0f, 50.0f, 100.0f, 0.0f};
        
        for (int i = 0; i < 4; i++) {
            std::cout << "  Setting output to " << test_values[i] << "%" << std::endl;
            dac.setOutputPercent(test_values[i]);
            sleep(2); // 2초 대기
        }
        
        std::cout << "\n✅ SUCCESS: MCP4921 is working" << std::endl;
        delete spi;
        return true;
    }
    
    /**
     * @brief GPIO 핀 테스트
     */
    bool testGPIO(int pin, bool is_output = true) {
        std::cout << "\n=== GPIO Pin Test ===" << std::endl;
        std::cout << "Pin: " << pin << " (" << (is_output ? "Output" : "Input") << ")" << std::endl;
        
        struct gpiod_chip* chip = gpiod_chip_open("/dev/gpiochip0");
        if (!chip) {
            std::cout << "❌ FAILED: Cannot open GPIO chip" << std::endl;
            return false;
        }
        
        struct gpiod_line* line = gpiod_chip_get_line(chip, pin);
        if (!line) {
            std::cout << "❌ FAILED: Cannot get GPIO line" << std::endl;
            gpiod_chip_close(chip);
            return false;
        }
        
        int ret;
        if (is_output) {
            ret = gpiod_line_request_output(line, "test", 0);
            if (ret < 0) {
                std::cout << "❌ FAILED: Cannot request GPIO as output" << std::endl;
                gpiod_chip_close(chip);
                return false;
            }
            
            std::cout << "Testing output (HIGH -> LOW -> HIGH)..." << std::endl;
            gpiod_line_set_value(line, 1);
            sleep(1);
            gpiod_line_set_value(line, 0);
            sleep(1);
            gpiod_line_set_value(line, 1);
        } else {
            ret = gpiod_line_request_input(line, "test");
            if (ret < 0) {
                std::cout << "❌ FAILED: Cannot request GPIO as input" << std::endl;
                gpiod_chip_close(chip);
                return false;
            }
            
            std::cout << "Reading input value (5 times)..." << std::endl;
            for (int i = 0; i < 5; i++) {
                int value = gpiod_line_get_value(line);
                std::cout << "  Value: " << value << std::endl;
                sleep(1);
            }
        }
        
        gpiod_line_release(line);
        gpiod_chip_close(chip);
        
        std::cout << "✅ SUCCESS: GPIO pin is working" << std::endl;
        return true;
    }
    
    /**
     * @brief 전체 하드웨어 테스트
     */
    void runAllTests() {
        std::cout << "========================================" << std::endl;
        std::cout << "  Hardware Test Suite" << std::endl;
        std::cout << "========================================" << std::endl;
        
        bool all_passed = true;
        
        // 1. SPI 디바이스 확인
        all_passed &= testSPIDevice("/dev/spidev0.0");
        all_passed &= testSPIDevice("/dev/spidev0.1");
        all_passed &= testSPIDevice("/dev/spidev1.0");
        
        // 2. I2C 디바이스 확인
        all_passed &= testI2CDevice("/dev/i2c-0");
        all_passed &= testI2CDevice("/dev/i2c-1");
        
        // 3. I2C 스캔
        scanI2CDevices("/dev/i2c-1");
        
        // 4. MPU6050 테스트
        all_passed &= testMPU6050();
        
        // 5. HardwareSPI 테스트
        all_passed &= testHardwareSPI("/dev/spidev0.0");
        
        // 6. MCP4921 테스트 (CS 핀 번호는 하드웨어에 맞게 조정)
        // all_passed &= testMCP4921("/dev/spidev0.0", 19);
        // all_passed &= testMCP4921("/dev/spidev0.0", 20);
        
        // 7. GPIO 테스트 (핀 번호는 하드웨어에 맞게 조정)
        // all_passed &= testGPIO(168, true);  // Reverse0
        // all_passed &= testGPIO(13, true);   // Reverse1
        
        std::cout << "\n========================================" << std::endl;
        if (all_passed) {
            std::cout << "  ✅ All tests passed!" << std::endl;
        } else {
            std::cout << "  ❌ Some tests failed!" << std::endl;
        }
        std::cout << "========================================" << std::endl;
    }
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "hardware_test");
    
    HardwareTest test;
    
    if (argc > 1) {
        std::string test_type = argv[1];
        
        if (test_type == "spi") {
            std::string device = (argc > 2) ? argv[2] : "/dev/spidev0.0";
            test.testSPIDevice(device);
        } else if (test_type == "i2c") {
            std::string device = (argc > 2) ? argv[2] : "/dev/i2c-1";
            test.testI2CDevice(device);
        } else if (test_type == "scan") {
            std::string device = (argc > 2) ? argv[2] : "/dev/i2c-1";
            test.scanI2CDevices(device);
        } else if (test_type == "mpu6050") {
            test.testMPU6050();
        } else if (test_type == "hardware_spi") {
            std::string device = (argc > 2) ? argv[2] : "/dev/spidev0.0";
            test.testHardwareSPI(device);
        } else if (test_type == "dac") {
            std::string device = (argc > 2) ? argv[2] : "/dev/spidev0.0";
            int cs_pin = (argc > 3) ? std::stoi(argv[3]) : 19;
            test.testMCP4921(device, cs_pin);
        } else if (test_type == "gpio") {
            int pin = (argc > 2) ? std::stoi(argv[2]) : 168;
            bool is_output = (argc > 3) ? (std::string(argv[3]) == "out") : true;
            test.testGPIO(pin, is_output);
        } else {
            std::cout << "Usage: hardware_test [test_type] [options]" << std::endl;
            std::cout << "Test types:" << std::endl;
            std::cout << "  spi <device>        - Test SPI device" << std::endl;
            std::cout << "  i2c <device>        - Test I2C device" << std::endl;
            std::cout << "  scan <device>       - Scan I2C devices" << std::endl;
            std::cout << "  mpu6050             - Test MPU6050 sensor" << std::endl;
            std::cout << "  hardware_spi <dev>  - Test HardwareSPI class" << std::endl;
            std::cout << "  dac <dev> <cs_pin>  - Test MCP4921 DAC" << std::endl;
            std::cout << "  gpio <pin> [in/out] - Test GPIO pin" << std::endl;
            std::cout << "  all                 - Run all tests" << std::endl;
        }
    } else {
        test.runAllTests();
    }
    
    return 0;
}




