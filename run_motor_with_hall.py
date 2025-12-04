#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
모터를 돌리면서 동시에 홀 센서 값 읽기
"""

import paramiko
import sys
import time
import signal
import threading

HOSTNAME = "192.168.0.28"
USERNAME = "tbot3"
PASSWORD = "1234"

ssh_client = None
hall_reading = True

def signal_handler(sig, frame):
    """Ctrl+C 핸들러"""
    global hall_reading
    print("\n\nInterrupted by user. Stopping...")
    hall_reading = False
    stop_motors()
    sys.exit(0)

def stop_motors():
    """모터 정지"""
    global ssh_client
    if ssh_client:
        try:
            cmd = "pkill -f motor_test || true"
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

def read_hall_sensors(client):
    """홀 센서 값 읽기 (백그라운드 스레드)"""
    global hall_reading
    
    c_program = '''#define _POSIX_C_SOURCE 200809L
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
    
    gpiod_line_request_output(latch, "hall_read", 1);
    
    printf("Time,Wheel0,Wheel1\\n");
    fflush(stdout);
    
    int count = 0;
    while (count < 200) {  // 약 20초 (100ms 간격)
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
        
        struct timespec ts;
        clock_gettime(CLOCK_MONOTONIC, &ts);
        double time_sec = ts.tv_sec + ts.tv_nsec / 1e9;
        
        printf("%.3f,%d,%d\\n", time_sec, hall0, hall1);
        fflush(stdout);
        
        usleep(100000);  // 100ms
        count++;
    }
    
    gpiod_line_release(latch);
    gpiod_chip_close(chip);
    close(fd);
    return 0;
}
'''
    
    # 프로그램 작성
    write_cmd = f"cat > /tmp/read_hall_continuous.c << 'ENDOFFILE'\n{c_program}\nENDOFFILE"
    success, _, _ = run_command(client, write_cmd, timeout=5)
    
    if not success:
        return False
    
    # 컴파일
    compile_cmd = "gcc -o /tmp/read_hall_continuous /tmp/read_hall_continuous.c -lgpiod -std=c99 -lrt 2>&1"
    success, output, error = run_command(client, compile_cmd, timeout=10)
    
    if not success:
        print(f"   Compilation failed: {error}")
        return False
    
    # 실행 (백그라운드)
    run_cmd = "sudo /tmp/read_hall_continuous 2>/dev/null || /tmp/read_hall_continuous 2>/dev/null"
    stdin, stdout, stderr = client.exec_command(run_cmd, timeout=25)
    
    # 출력 읽기
    print("\n--- Hall Sensor Values (Real-time) ---")
    print("Time(s)  Wheel0  Wheel1")
    print("-" * 30)
    
    start_time = time.time()
    prev_wheel0 = -1
    prev_wheel1 = -1
    
    try:
        while hall_reading:
            if stdout.channel.recv_ready():
                line = stdout.channel.recv(1024).decode('utf-8', errors='ignore')
                if line:
                    for l in line.split('\n'):
                        l = l.strip()
                        if l and ',' in l and not l.startswith('Time'):
                            parts = l.split(',')
                            if len(parts) == 3:
                                try:
                                    t = float(parts[0])
                                    w0 = int(parts[1])
                                    w1 = int(parts[2])
                                    
                                    # 변화 감지
                                    change0 = " ⚠️" if w0 != prev_wheel0 and prev_wheel0 != -1 else ""
                                    change1 = " ⚠️" if w1 != prev_wheel1 and prev_wheel1 != -1 else ""
                                    
                                    print(f"{t:7.2f}    {w0:3d}    {w1:3d}{change0}{change1}")
                                    
                                    prev_wheel0 = w0
                                    prev_wheel1 = w1
                                except:
                                    pass
            
            if stdout.channel.exit_status_ready():
                break
            
            time.sleep(0.05)
    except:
        pass
    
    return True

def run_motor(client, motor_id, speed_percent, duration):
    """모터 실행"""
    cs_pin = 19 if motor_id == 0 else 20
    
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
    
    struct gpiod_line* cs_line = gpiod_chip_get_line(chip, {cs_pin});
    if (!cs_line) {{
        perror("get CS line");
        gpiod_chip_close(chip);
        close(fd);
        return 1;
    }}
    
    gpiod_line_request_output(cs_line, "motor", 1);
    
    uint16_t dac_value = (uint16_t)({speed_percent} * 40.95f);
    if (dac_value > 4095) dac_value = 4095;
    
    uint16_t command = dac_value;
    command |= 0x4000;
    command |= 0x2000;
    command |= 0x1000;
    
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
    
    // {duration}초 대기
    sleep({duration});
    
    // 정지
    command = 0;
    command |= 0x4000;
    command |= 0x2000;
    command |= 0x1000;
    
    tx[0] = (command >> 8) & 0xFF;
    tx[1] = command & 0xFF;
    
    gpiod_line_set_value(cs_line, 0);
    ioctl(fd, SPI_IOC_MESSAGE(1), &tr);
    gpiod_line_set_value(cs_line, 1);
    
    gpiod_line_release(cs_line);
    gpiod_chip_close(chip);
    close(fd);
    return 0;
}}
'''
    
    write_cmd = f"cat > /tmp/motor_{motor_id}.c << 'ENDOFFILE'\n{c_program}\nENDOFFILE"
    run_command(client, write_cmd, timeout=5)
    
    compile_cmd = f"gcc -o /tmp/motor_{motor_id} /tmp/motor_{motor_id}.c -lgpiod -std=c99 2>&1"
    success, _, _ = run_command(client, compile_cmd, timeout=10)
    
    if not success:
        return False
    
    # 백그라운드 실행
    run_cmd = f"sudo /tmp/motor_{motor_id} > /dev/null 2>&1 &"
    client.exec_command(run_cmd, timeout=2)
    
    return True

def run_command(client, command, timeout=10):
    """명령 실행"""
    try:
        stdin, stdout, stderr = client.exec_command(command, timeout=timeout)
        start_time = time.time()
        
        output = ""
        error = ""
        
        while True:
            if time.time() - start_time > timeout:
                stdout.channel.close()
                stderr.channel.close()
                return False, output, f"Timeout after {timeout}s"
            
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
    global ssh_client, hall_reading
    
    signal.signal(signal.SIGINT, signal_handler)
    if sys.platform != 'win32':
        signal.signal(signal.SIGTERM, signal_handler)
    
    print("=== Motor Test with Hall Sensor Reading ===")
    print("Press Ctrl+C to stop early\n")
    
    client = create_ssh_client()
    if not client:
        return 1
    
    ssh_client = client
    
    try:
        speed = 30
        duration = 10
        
        print(f"Starting motors at {speed}% for {duration} seconds...")
        print("Reading hall sensor values in real-time...\n")
        
        # 홀 센서 읽기 시작 (백그라운드)
        hall_thread = threading.Thread(target=lambda: read_hall_sensors(client))
        hall_thread.daemon = True
        hall_thread.start()
        
        time.sleep(1)  # 홀 센서 읽기 시작 대기
        
        # 모터 시작
        print("\n--- Starting Motors ---")
        run_motor(client, 0, speed, duration)
        time.sleep(0.2)
        run_motor(client, 1, speed, duration)
        
        print(f"\nMotors running for {duration} seconds...")
        print("(Watch for hall sensor value changes marked with ⚠️)\n")
        
        # 모터 실행 대기
        time.sleep(duration + 1)
        
        hall_reading = False
        hall_thread.join(timeout=2)
        
        print("\n" + "="*50)
        print("✅ Test completed!")
        print("="*50)
        print("\nSummary:")
        print("  - If you saw ⚠️ marks, hall sensor values changed (working!)")
        print("  - If all values were 0, check hall sensor connections")
        print("  - If values didn't change, wheels may not be rotating")
        
        return 0
        
    except KeyboardInterrupt:
        print("\n\nStopping...")
        hall_reading = False
        stop_motors()
        return 1
    except Exception as e:
        print(f"❌ Error: {e}")
        hall_reading = False
        stop_motors()
        return 1
    finally:
        try:
            if client:
                client.close()
        except:
            pass

if __name__ == "__main__":
    if sys.platform == 'win32':
        import io
        try:
            sys.stdout = io.TextIOWrapper(sys.stdout.buffer, encoding='utf-8')
            sys.stderr = io.TextIOWrapper(sys.stderr.buffer, encoding='utf-8')
        except:
            pass
    
    sys.exit(main())



