#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
원격 서버에서 모터를 10초간 실행 후 정지
"""

import paramiko
import sys
import time
import signal

HOSTNAME = "192.168.0.28"
USERNAME = "tbot3"
PASSWORD = "1234"

ssh_client = None

def signal_handler(sig, frame):
    """Ctrl+C 핸들러"""
    print("\n\nInterrupted by user. Stopping motors...")
    stop_motors()
    sys.exit(0)

def stop_motors():
    """모터 정지"""
    global ssh_client
    if ssh_client:
        try:
            setup_cmd = "cd ~/catkin_ws && source devel/setup.bash"
            # DAC 출력을 0으로 설정
            cmd = f"{setup_cmd} && rosrun integrated_balancer hardware_test dac /dev/spidev0.0 19 0 2>/dev/null &"
            ssh_client.exec_command(cmd, timeout=2)
            cmd = f"{setup_cmd} && rosrun integrated_balancer hardware_test dac /dev/spidev0.0 20 0 2>/dev/null &"
            ssh_client.exec_command(cmd, timeout=2)
        except:
            pass

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

def run_motor_test(client, motor_id, speed_percent, duration):
    """모터 테스트 실행"""
    setup_cmd = "cd ~/catkin_ws && source devel/setup.bash"
    
    # CS 핀 번호 결정
    cs_pin = 19 if motor_id == 0 else 20
    
    print(f"\n=== Motor {motor_id} Test ===")
    print(f"Speed: {speed_percent}%")
    print(f"Duration: {duration} seconds")
    print(f"CS Pin: {cs_pin}")
    
    # 간단한 C 프로그램으로 모터 제어
    c_program = f'''#define _POSIX_C_SOURCE 200809L
#include <stdio.h>
#include <unistd.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <time.h>
#include <linux/spi/spidev.h>
#include <gpiod.h>
#include <stdint.h>
#include <string.h>

int main() {{
    int fd = open("/dev/spidev0.0", O_RDWR);
    if (fd < 0) {{
        perror("open SPI");
        return 1;
    }}
    
    uint8_t mode = SPI_MODE_0;
    uint8_t bits = 8;
    uint32_t speed = 1000000;
    ioctl(fd, SPI_IOC_WR_MODE, &mode);
    ioctl(fd, SPI_IOC_WR_BITS_PER_WORD, &bits);
    ioctl(fd, SPI_IOC_WR_MAX_SPEED_HZ, &speed);
    
    struct gpiod_chip* chip = gpiod_chip_open("/dev/gpiochip0");
    if (!chip) {{
        perror("open GPIO");
        close(fd);
        return 1;
    }}
    
    // CS 핀 설정
    struct gpiod_line* cs_line = gpiod_chip_get_line(chip, {cs_pin});
    if (!cs_line) {{
        perror("get CS line");
        gpiod_chip_close(chip);
        close(fd);
        return 1;
    }}
    
    gpiod_line_request_output(cs_line, "motor_test", 1);
    
    // DAC 값 계산 (0-4095, 12비트)
    uint16_t dac_value = (uint16_t)({speed_percent} * 40.95f);
    if (dac_value > 4095) dac_value = 4095;
    
    // MCP4921 명령 구성
    uint16_t command = dac_value;
    command |= 0x4000; // Buffered
    command |= 0x2000; // Gain 1x
    command |= 0x1000; // Active
    
    printf("Starting motor {motor_id} at {speed_percent}%% for {duration} seconds...\\n");
    
    // 모터 시작
    gpiod_line_set_value(cs_line, 0);
    
    uint8_t tx[2] = {{(command >> 8) & 0xFF, command & 0xFF}};
    uint8_t rx[2] = {{0, 0}};
    
    struct spi_ioc_transfer tr;
    memset(&tr, 0, sizeof(tr));
    tr.tx_buf = (unsigned long)tx;
    tr.rx_buf = (unsigned long)rx;
    tr.len = 2;
    tr.speed_hz = speed;
    tr.bits_per_word = bits;
    
    ioctl(fd, SPI_IOC_MESSAGE(1), &tr);
    gpiod_line_set_value(cs_line, 1);
    
    printf("Motor running...\\n");
    
    // {duration}초 대기
    sleep({duration});
    
    // 모터 정지
    printf("Stopping motor...\\n");
    command = 0; // 0% 출력
    command |= 0x4000;
    command |= 0x2000;
    command |= 0x1000;
    
    tx[0] = (command >> 8) & 0xFF;
    tx[1] = command & 0xFF;
    
    gpiod_line_set_value(cs_line, 0);
    ioctl(fd, SPI_IOC_MESSAGE(1), &tr);
    gpiod_line_set_value(cs_line, 1);
    
    printf("Motor stopped.\\n");
    
    gpiod_line_release(cs_line);
    gpiod_chip_close(chip);
    close(fd);
    return 0;
}}
'''
    
    # 프로그램 작성
    write_cmd = f"cat > /tmp/motor_test_{motor_id}.c << 'ENDOFFILE'\n{c_program}\nENDOFFILE"
    success, _, _ = run_command(client, write_cmd, timeout=5)
    
    if not success:
        print(f"   Failed to write test program for motor {motor_id}")
        return False
    
    # 컴파일
    print("   Compiling...")
    compile_cmd = f"gcc -o /tmp/motor_test_{motor_id} /tmp/motor_test_{motor_id}.c -lgpiod -std=c99 2>&1"
    success, output, error = run_command(client, compile_cmd, timeout=10)
    
    if not success:
        print(f"   Compilation failed: {error}")
        if output:
            print(f"   Output: {output}")
        return False
    
    # 실행
    print("   Running motor...")
    run_cmd = f"sudo /tmp/motor_test_{motor_id} 2>&1 || /tmp/motor_test_{motor_id} 2>&1"
    success, output, error = run_command(client, run_cmd, timeout=duration+5)
    
    if success:
        print(f"\n✅ Motor {motor_id} test completed!")
        if output:
            print(output)
        return True
    else:
        print(f"   Execution failed: {error}")
        if output:
            print(f"   Output: {output}")
        return False

def run_command(client, command, timeout=10):
    """명령 실행 및 출력 반환"""
    try:
        stdin, stdout, stderr = client.exec_command(command, timeout=timeout)
        
        output = ""
        error = ""
        start_time = time.time()
        
        while True:
            if time.time() - start_time > timeout:
                stdout.channel.close()
                stderr.channel.close()
                return False, output, f"Command timeout after {timeout}s"
            
            if stdout.channel.recv_ready():
                output += stdout.channel.recv(4096).decode('utf-8', errors='ignore')
            if stderr.channel.recv_stderr_ready():
                error += stderr.channel.recv_stderr(4096).decode('utf-8', errors='ignore')
            
            if stdout.channel.exit_status_ready():
                exit_status = stdout.channel.recv_exit_status()
                while stdout.channel.recv_ready():
                    output += stdout.channel.recv(4096).decode('utf-8', errors='ignore')
                while stderr.channel.recv_stderr_ready():
                    error += stderr.channel.recv_stderr(4096).decode('utf-8', errors='ignore')
                break
            
            time.sleep(0.05)
        
        return exit_status == 0, output, error
    except KeyboardInterrupt:
        raise
    except Exception as e:
        return False, "", str(e)

def main():
    """메인 함수"""
    global ssh_client
    
    # 시그널 핸들러 등록
    signal.signal(signal.SIGINT, signal_handler)
    if sys.platform != 'win32':
        signal.signal(signal.SIGTERM, signal_handler)
    
    print("=== Motor Test (10 seconds) ===")
    print("Press Ctrl+C to stop early\n")
    
    client = create_ssh_client()
    if not client:
        return 1
    
    ssh_client = client
    
    try:
        # 모터 속도 설정 (30%로 시작 - 안전)
        speed = 30
        duration = 10
        
        print(f"Starting both motors at {speed}% for {duration} seconds...")
        print("Watch the wheels with your camera!\n")
        
        # 두 모터를 동시에 실행 (백그라운드)
        print("Starting Motor 0...")
        success0 = run_motor_test(client, 0, speed, duration)
        
        # 약간의 지연 후 모터 1 시작 (동시 실행)
        time.sleep(0.5)
        print("\nStarting Motor 1...")
        success1 = run_motor_test(client, 1, speed, duration)
        
        if success0 and success1:
            print("\n" + "="*50)
            print("✅ Both motors completed successfully!")
            print("="*50)
            return 0
        else:
            print("\n⚠️  Some motors may have failed")
            return 1
        
    except KeyboardInterrupt:
        print("\n\nStopping motors...")
        stop_motors()
        return 1
    except Exception as e:
        print(f"❌ Error: {e}")
        stop_motors()
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



