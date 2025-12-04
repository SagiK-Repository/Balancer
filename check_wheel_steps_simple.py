#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
원격 서버에서 바퀴 스텝 값(엔코더 카운트) 확인 - 간단한 버전
"""

import paramiko
import sys
import time
import signal
import threading

HOSTNAME = "192.168.0.28"
USERNAME = "tbot3"
PASSWORD = "1234"

# 전역 변수로 클라이언트 저장 (종료 시 닫기 위해)
ssh_client = None

def create_ssh_client():
    """SSH 클라이언트 생성"""
    client = paramiko.SSHClient()
    client.set_missing_host_key_policy(paramiko.AutoAddPolicy())
    
    try:
        print(f"Connecting to {USERNAME}@{HOSTNAME}...")
        client.connect(HOSTNAME, username=USERNAME, password=PASSWORD, timeout=10)
        print("Connected successfully!")
        return client
    except Exception as e:
        print(f"Connection failed: {e}")
        return None

def run_command(client, command, timeout=10):
    """명령 실행 및 출력 반환 (타임아웃 적용, Ctrl+C 지원)"""
    try:
        # exec_command 사용 (더 안정적)
        stdin, stdout, stderr = client.exec_command(command, timeout=timeout)
        
        # 비동기로 출력 읽기
        output = ""
        error = ""
        start_time = time.time()
        
        while True:
            # 타임아웃 체크
            if time.time() - start_time > timeout:
                stdout.channel.close()
                stderr.channel.close()
                return False, output, f"Command timeout after {timeout}s"
            
            # 출력 읽기 (논블로킹)
            if stdout.channel.recv_ready():
                output += stdout.channel.recv(4096).decode('utf-8', errors='ignore')
            if stderr.channel.recv_stderr_ready():
                error += stderr.channel.recv_stderr(4096).decode('utf-8', errors='ignore')
            
            # 종료 확인
            if stdout.channel.exit_status_ready():
                exit_status = stdout.channel.recv_exit_status()
                # 남은 출력 읽기
                while stdout.channel.recv_ready():
                    output += stdout.channel.recv(4096).decode('utf-8', errors='ignore')
                while stderr.channel.recv_stderr_ready():
                    error += stderr.channel.recv_stderr(4096).decode('utf-8', errors='ignore')
                break
            
            time.sleep(0.05)  # 짧은 대기
        
        return exit_status == 0, output, error
    except KeyboardInterrupt:
        raise
    except paramiko.SSHException as e:
        return False, "", f"SSH error: {e}"
    except Exception as e:
        return False, "", str(e)

def signal_handler(sig, frame):
    """Ctrl+C 핸들러"""
    print("\n\nInterrupted by user. Closing connections...")
    global ssh_client
    if ssh_client:
        try:
            ssh_client.close()
        except:
            pass
    sys.exit(0)

def main():
    """메인 함수"""
    global ssh_client
    
    # 시그널 핸들러 등록
    signal.signal(signal.SIGINT, signal_handler)
    if sys.platform != 'win32':
        signal.signal(signal.SIGTERM, signal_handler)
    
    print("=== Wheel Step Value Check (Simple) ===\n")
    print("Press Ctrl+C to exit at any time\n")
    
    client = create_ssh_client()
    if not client:
        return 1
    
    ssh_client = client  # 전역 변수에 저장
    
    try:
        setup_cmd = "cd ~/catkin_ws && source devel/setup.bash"
        
        # 방법 1: 기존 balance_robot_nodes가 실행 중인지 확인
        print("1. Checking if balance_robot_nodes is running...")
        success, output, error = run_command(client, "ps aux | grep -E 'spi_hardware_node|balance_controller' | grep -v grep", timeout=5)
        
        if "spi_hardware_node" in output or "balance_controller" in output:
            print("   Found running nodes!")
            print("\n2. Reading motor status from ROS topics...")
            
            # 모터 상태 토픽 읽기
            cmd = f"{setup_cmd} && timeout 2 rostopic echo /balance_robot/motor_status -n 3"
            success, output, error = run_command(client, cmd, timeout=5)
            
            if success and output:
                print("\n--- Motor Status ---")
                lines = output.split('\n')
                found_data = False
                
                for line in lines:
                    line = line.strip()
                    if 'motor0_hall_state' in line or 'motor0_hall_count' in line:
                        print(f"  {line}")
                        found_data = True
                    if 'motor1_hall_state' in line or 'motor1_hall_count' in line:
                        print(f"  {line}")
                        found_data = True
                
                if found_data:
                    print("\n✅ Successfully read encoder values!")
                    return 0
                else:
                    print("   No encoder data found in topic")
            else:
                print("   Could not read from topic")
        else:
            print("   No running nodes found")
        
        # 방법 2: 직접 SPI로 홀 센서 읽기 (간단한 C 프로그램)
        print("\n3. Reading hall sensor directly via SPI...")
        
        # 간단한 C 프로그램 작성
        c_program = '''        #define _POSIX_C_SOURCE 200809L
#include <stdio.h>
#include <unistd.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <time.h>
#include <linux/spi/spidev.h>
#include <gpiod.h>
#include <stdint.h>
#include <string.h>

int main() {
    int fd = open("/dev/spidev0.0", O_RDWR);
    if (fd < 0) {
        perror("open SPI");
        return 1;
    }
    
    uint8_t mode = SPI_MODE_0;
    uint8_t bits = 8;
    uint32_t speed = 1000000;
    ioctl(fd, SPI_IOC_WR_MODE, &mode);
    ioctl(fd, SPI_IOC_WR_BITS_PER_WORD, &bits);
    ioctl(fd, SPI_IOC_WR_MAX_SPEED_HZ, &speed);
    
    struct gpiod_chip* chip = gpiod_chip_open("/dev/gpiochip0");
    if (!chip) {
        perror("open GPIO");
        close(fd);
        return 1;
    }
    
    struct gpiod_line* latch = gpiod_chip_get_line(chip, 38);
    if (!latch) {
        perror("get latch line");
        gpiod_chip_close(chip);
        close(fd);
        return 1;
    }
    
    gpiod_line_request_output(latch, "test", 1);
    
    printf("Reading hall sensor states (10 samples):\\n");
    
    for (int i = 0; i < 10; i++) {
        gpiod_line_set_value(latch, 1);
        usleep(1000);
        
        uint8_t tx = 0x00;
        uint8_t rx = 0x00;
        
        struct spi_ioc_transfer tr;
        memset(&tr, 0, sizeof(tr));
        tr.tx_buf = (unsigned long)&tx;
        tr.rx_buf = (unsigned long)&rx;
        tr.len = 1;
        tr.speed_hz = speed;
        tr.bits_per_word = bits;
        
        gpiod_line_set_value(latch, 0);
        usleep(1000);
        
        if (ioctl(fd, SPI_IOC_MESSAGE(1), &tr) < 0) {
            perror("SPI transfer");
            break;
        }
        
        uint8_t hall0 = (rx >> 0) & 0x7;
        uint8_t hall1 = (rx >> 3) & 0x7;
        
        printf("Sample %2d: Wheel0=%d (0x%X) Wheel1=%d (0x%X)\\n", 
               i+1, hall0, hall0, hall1, hall1);
        
        usleep(100000);
    }
    
    gpiod_line_release(latch);
    gpiod_chip_close(chip);
    close(fd);
    return 0;
}
'''
        
        # 프로그램 작성
        write_cmd = f"cat > /tmp/read_hall.c << 'ENDOFFILE'\n{c_program}\nENDOFFILE"
        success, _, _ = run_command(client, write_cmd, timeout=5)
        
        if not success:
            print("   Failed to write test program")
            return 1
        
        # 컴파일
        print("   Compiling test program...")
        compile_cmd = "gcc -o /tmp/read_hall /tmp/read_hall.c -lgpiod -std=c99 2>&1"
        success, output, error = run_command(client, compile_cmd, timeout=10)
        
        if not success:
            print(f"   Compilation failed: {error}")
            if output:
                print(f"   Output: {output}")
            return 1
        
        # 실행 (sudo 필요할 수 있음)
        print("   Running test program (this will take ~2 seconds)...")
        run_cmd = "timeout 3 sudo /tmp/read_hall 2>&1 || timeout 3 /tmp/read_hall 2>&1"
        success, output, error = run_command(client, run_cmd, timeout=5)
        
        if success and output:
            print("\n--- Hall Sensor States ---")
            print(output)
            print("\n✅ Successfully read hall sensor states!")
            print("\nNote:")
            print("  - Values should change when wheels rotate")
            print("  - If all values are 0, check hall sensor connections")
            print("  - Hall state range: 0-7 (3-bit value)")
            return 0
        else:
            print(f"   Execution failed: {error}")
            if output:
                print(f"   Output: {output}")
            return 1
        
    except KeyboardInterrupt:
        print("\n\nInterrupted by user.")
        return 1
    except Exception as e:
        print(f"Error: {e}")
        import traceback
        traceback.print_exc()
        return 1
    finally:
        try:
            if client:
                client.close()
        except:
            pass

if __name__ == "__main__":
    # Windows에서 UTF-8 출력 설정
    if sys.platform == 'win32':
        import io
        try:
            sys.stdout = io.TextIOWrapper(sys.stdout.buffer, encoding='utf-8')
            sys.stderr = io.TextIOWrapper(sys.stderr.buffer, encoding='utf-8')
        except:
            pass
    
    sys.exit(main())

