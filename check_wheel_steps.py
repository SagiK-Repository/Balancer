#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
원격 서버에서 바퀴 스텝 값(엔코더 카운트) 확인
"""

import paramiko
import sys
import time

HOSTNAME = "192.168.0.28"
USERNAME = "tbot3"
PASSWORD = "1234"

def create_ssh_client():
    """SSH 클라이언트 생성"""
    client = paramiko.SSHClient()
    client.set_missing_host_key_policy(paramiko.AutoAddPolicy())
    
    try:
        print(f"Connecting to {USERNAME}@{HOSTNAME}...")
        client.connect(HOSTNAME, username=USERNAME, password=PASSWORD, timeout=10)
        print("✅ Connected successfully!")
        return client
    except Exception as e:
        print(f"❌ Connection failed: {e}")
        return None

def run_command(client, command, description=""):
    """명령 실행 및 출력"""
    if description:
        print(f"\n=== {description} ===")
    
    try:
        stdin, stdout, stderr = client.exec_command(command)
        
        # 출력 실시간 표시
        output = ""
        for line in stdout:
            line_text = line.rstrip()
            print(line_text)
            output += line_text + "\n"
        
        exit_status = stdout.channel.recv_exit_status()
        
        if exit_status != 0:
            print("Error output:")
            for line in stderr:
                print(line.rstrip())
        
        return exit_status == 0, output
    except Exception as e:
        print(f"❌ Command execution failed: {e}")
        return False, ""

def check_hall_sensors(client):
    """홀 센서 상태 직접 확인"""
    print("\n" + "="*50)
    print("Checking Hall Sensor States...")
    print("="*50)
    
    setup_cmd = "cd ~/catkin_ws && source devel/setup.bash"
    
    # SPI를 통해 홀 센서 읽기 테스트
    # 기존 balance_robot_nodes의 모터 상태 확인
    print("\n1. Checking motor status from balance_robot_nodes...")
    check_cmd = f"{setup_cmd} && timeout 3 rostopic echo /balance_robot/motor_status -n 1 2>/dev/null || echo 'No motor_status topic found'"
    success, output = run_command(client, check_cmd, "Motor Status")
    
    if "motor0_hall_state" in output or "motor1_hall_state" in output:
        print("\n✅ Found motor status data!")
        return True
    
    # 통합 밸런서 실행하여 확인
    print("\n2. Starting integrated_balancer to check encoder values...")
    print("   (This will run for 5 seconds)")
    
    # 백그라운드로 실행하고 토픽 확인
    start_cmd = f"{setup_cmd} && rosrun integrated_balancer integrated_balancer_node &"
    run_command(client, start_cmd, "Starting integrated_balancer")
    
    time.sleep(2)  # 시작 대기
    
    # 모터 상태 확인 (기존 노드가 실행 중인지 확인)
    print("\n3. Checking if balance_robot_nodes is running...")
    check_cmd = "ps aux | grep -E '(spi_hardware_node|balance_controller)' | grep -v grep"
    success, output = run_command(client, check_cmd, "Running Nodes")
    
    if "spi_hardware_node" in output:
        print("\n✅ Found spi_hardware_node running!")
        print("Reading motor status...")
        
        # 모터 상태 토픽 읽기
        for i in range(3):
            check_cmd = f"{setup_cmd} && timeout 1 rostopic echo /balance_robot/motor_status -n 1 2>/dev/null"
            success, output = run_command(client, check_cmd, f"Motor Status (attempt {i+1})")
            
            if "motor0_hall_state" in output:
                # 값 추출
                lines = output.split('\n')
                for line in lines:
                    if 'motor0_hall_state' in line or 'motor0_hall_count' in line:
                        print(f"  {line.strip()}")
                    if 'motor1_hall_state' in line or 'motor1_hall_count' in line:
                        print(f"  {line.strip()}")
                return True
            
            time.sleep(1)
    
    return False

def create_encoder_test(client):
    """엔코더 값을 직접 읽는 테스트 프로그램 생성"""
    print("\n" + "="*50)
    print("Creating encoder test program...")
    print("="*50)
    
    test_code = '''#include <iostream>
#include <unistd.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <linux/spi/spidev.h>
#include <gpiod.h>
#include <cstdint>

int main() {
    // SPI 초기화
    int fd = open("/dev/spidev0.0", O_RDWR);
    if (fd < 0) {
        std::cerr << "Failed to open SPI device" << std::endl;
        return 1;
    }
    
    uint8_t mode = SPI_MODE_0;
    uint8_t bits = 8;
    uint32_t speed = 1000000;
    ioctl(fd, SPI_IOC_WR_MODE, &mode);
    ioctl(fd, SPI_IOC_WR_BITS_PER_WORD, &bits);
    ioctl(fd, SPI_IOC_WR_MAX_SPEED_HZ, &speed);
    
    // GPIO 초기화 (Latch 핀)
    struct gpiod_chip* chip = gpiod_chip_open("/dev/gpiochip0");
    if (!chip) {
        std::cerr << "Failed to open GPIO chip" << std::endl;
        close(fd);
        return 1;
    }
    
    struct gpiod_line* latch_line = gpiod_chip_get_line(chip, 38);
    if (!latch_line) {
        std::cerr << "Failed to get latch line" << std::endl;
        gpiod_chip_close(chip);
        close(fd);
        return 1;
    }
    
    gpiod_line_request_output(latch_line, "encoder_test", 1);
    
    // 홀 센서 읽기
    std::cout << "Reading hall sensor states..." << std::endl;
    
    for (int i = 0; i < 10; i++) {
        gpiod_line_set_value(latch_line, 1);
        usleep(1000);
        
        uint8_t tx = 0x00;
        uint8_t rx = 0x00;
        
        struct spi_ioc_transfer tr = {
            .tx_buf = (unsigned long)&tx,
            .rx_buf = (unsigned long)&rx,
            .len = 1,
            .speed_hz = speed,
            .bits_per_word = bits,
        };
        
        gpiod_line_set_value(latch_line, 0);
        usleep(1000);
        
        ioctl(fd, SPI_IOC_MESSAGE(1), &tr);
        
        uint8_t hall_state_0 = (rx >> 0) & 0x7;
        uint8_t hall_state_1 = (rx >> 3) & 0x7;
        
        std::cout << "Sample " << (i+1) << ": Wheel0=" << (int)hall_state_0 
                  << " Wheel1=" << (int)hall_state_1 << std::endl;
        
        usleep(100000); // 100ms
    }
    
    gpiod_line_release(latch_line);
    gpiod_chip_close(chip);
    close(fd);
    
    return 0;
}
'''
    
    # 원격 서버에 테스트 프로그램 작성
    setup_cmd = "cd ~/catkin_ws && source devel/setup.bash"
    
    # 임시 파일로 저장
    write_cmd = f"cat > /tmp/encoder_test.cpp << 'EOF'\n{test_code}\nEOF"
    run_command(client, write_cmd, "Writing test program")
    
    # 컴파일
    compile_cmd = "g++ -o /tmp/encoder_test /tmp/encoder_test.cpp -lgpiod -std=c++11"
    success, _ = run_command(client, compile_cmd, "Compiling test program")
    
    if success:
        # 실행
        run_cmd = "sudo /tmp/encoder_test"
        success, output = run_command(client, run_cmd, "Running encoder test")
        return success
    
    return False

def main():
    """메인 함수"""
    print("=== Wheel Step Value Check ===")
    
    client = create_ssh_client()
    if not client:
        return 1
    
    try:
        # 방법 1: 기존 노드의 모터 상태 확인
        if check_hall_sensors(client):
            print("\n✅ Successfully read encoder values from running nodes!")
        else:
            print("\n⚠️  Could not read from running nodes. Trying direct SPI read...")
            
            # 방법 2: 직접 SPI로 홀 센서 읽기
            if create_encoder_test(client):
                print("\n✅ Successfully read hall sensor states directly!")
            else:
                print("\n❌ Failed to read encoder values")
                return 1
        
        print("\n" + "="*50)
        print("Summary")
        print("="*50)
        print("If you see hall_state values:")
        print("  - 0 = 000 (binary)")
        print("  - 1 = 001")
        print("  - 2 = 010")
        print("  - 3 = 011")
        print("  - 4 = 100")
        print("  - 5 = 101")
        print("  - 6 = 110")
        print("  - 7 = 111")
        print("\nIf all values are 0, check:")
        print("  1. Hall sensor connections")
        print("  2. SPI communication")
        print("  3. Latch pin (GPIO 38) connection")
        
        return 0
        
    except Exception as e:
        print(f"❌ Error: {e}")
        import traceback
        traceback.print_exc()
        return 1
    finally:
        client.close()

if __name__ == "__main__":
    # Windows에서 UTF-8 출력 설정
    if sys.platform == 'win32':
        import io
        sys.stdout = io.TextIOWrapper(sys.stdout.buffer, encoding='utf-8')
        sys.stderr = io.TextIOWrapper(sys.stderr.buffer, encoding='utf-8')
    
    sys.exit(main())



