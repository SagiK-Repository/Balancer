#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
모터가 안 돌아가는 문제 진단
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
    print("\n\nInterrupted by user.")
    sys.exit(0)

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

def test_dac_output(client, dac_id, test_values):
    """DAC 출력 테스트 (여러 값으로)"""
    cs_pin = 19 if dac_id == 0 else 20
    
    print(f"\n=== DAC{dac_id} Output Test (CS Pin: {cs_pin}) ===")
    print("Testing different output levels...")
    print("(Use multimeter to measure DAC output voltage)")
    print()
    
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
    
    gpiod_line_request_output(cs_line, "dac_test", 1);
    
    struct spi_ioc_transfer tr;
    memset(&tr, 0, sizeof(tr));
    tr.speed_hz = speed;
    tr.bits_per_word = bits;
    tr.len = 2;
    
    uint8_t tx[2];
    uint8_t rx[2];
    tr.tx_buf = (unsigned long)tx;
    tr.rx_buf = (unsigned long)rx;
    
    float test_percents[] = {{0.0f, 25.0f, 50.0f, 75.0f, 100.0f, 0.0f}};
    int num_tests = 6;
    
    for (int i = 0; i < num_tests; i++) {{
        float percent = test_percents[i];
        uint16_t dac_value = (uint16_t)(percent * 40.95f);
        if (dac_value > 4095) dac_value = 4095;
        
        uint16_t command = dac_value;
        command |= 0x4000; // Buffered
        command |= 0x2000; // Gain 1x
        command |= 0x1000; // Active
        
        tx[0] = (command >> 8) & 0xFF;
        tx[1] = command & 0xFF;
        
        gpiod_line_set_value(cs_line, 0);
        ioctl(fd, SPI_IOC_MESSAGE(1), &tr);
        gpiod_line_set_value(cs_line, 1);
        
        printf("DAC{dac_id}: %.0f%% (value: %d)\\n", percent, dac_value);
        fflush(stdout);
        
        sleep(2);  // 2초 대기 (멀티미터로 측정)
    }}
    
    gpiod_line_release(cs_line);
    gpiod_chip_close(chip);
    close(fd);
    return 0;
}}
'''
    
    write_cmd = f"cat > /tmp/dac_test_{dac_id}.c << 'ENDOFFILE'\n{c_program}\nENDOFFILE"
    success, _, _ = run_command(client, write_cmd, timeout=5)
    
    if not success:
        print(f"   Failed to write test program")
        return False
    
    compile_cmd = f"gcc -o /tmp/dac_test_{dac_id} /tmp/dac_test_{dac_id}.c -lgpiod -std=c99 2>&1"
    success, output, error = run_command(client, compile_cmd, timeout=10)
    
    if not success:
        print(f"   Compilation failed: {error}")
        return False
    
    print("Running test (each level for 2 seconds)...")
    print("Measure voltage at DAC output pin with multimeter")
    print()
    
    run_cmd = f"sudo /tmp/dac_test_{dac_id} 2>&1 || /tmp/dac_test_{dac_id} 2>&1"
    success, output, error = run_command(client, run_cmd, timeout=15)
    
    if success and output:
        print(output)
        print("\n✅ DAC test completed!")
        print("\nExpected voltages (with 3.3V VREF, Gain 1x):")
        print("  0%   -> 0.00V")
        print("  25%  -> 0.83V")
        print("  50%  -> 1.65V")
        print("  75%  -> 2.48V")
        print("  100% -> 3.30V")
        return True
    else:
        print(f"   Execution failed: {error}")
        return False

def check_gpio_pins(client):
    """GPIO 핀 상태 확인"""
    print("\n=== GPIO Pin Status Check ===")
    
    # Reverse 핀 확인
    pins = [
        (168, "Reverse0 (Motor 0 direction)"),
        (13, "Reverse1 (Motor 1 direction)")
    ]
    
    for pin, description in pins:
        print(f"\nChecking {description} (GPIO {pin})...")
        
        c_program = f'''#define _POSIX_C_SOURCE 200809L
#include <stdio.h>
#include <gpiod.h>
#include <time.h>

int main() {{
    struct gpiod_chip* chip = gpiod_chip_open("/dev/gpiochip0");
    if (!chip) {{
        perror("open GPIO");
        return 1;
    }}
    
    struct gpiod_line* line = gpiod_chip_get_line(chip, {pin});
    if (!line) {{
        perror("get line");
        gpiod_chip_close(chip);
        return 1;
    }}
    
    int ret = gpiod_line_request_output(line, "test", 1);
    if (ret < 0) {{
        perror("request output");
        gpiod_chip_close(chip);
        return 1;
    }}
    
    printf("Setting HIGH...\\n");
    gpiod_line_set_value(line, 1);
    sleep(1);
    
    printf("Setting LOW...\\n");
    gpiod_line_set_value(line, 0);
    sleep(1);
    
    printf("Setting HIGH again...\\n");
    gpiod_line_set_value(line, 1);
    
    gpiod_line_release(line);
    gpiod_chip_close(chip);
    return 0;
}}
'''
        
        write_cmd = f"cat > /tmp/gpio_test_{pin}.c << 'ENDOFFILE'\n{c_program}\nENDOFFILE"
        run_command(client, write_cmd, timeout=5)
        
        compile_cmd = f"gcc -o /tmp/gpio_test_{pin} /tmp/gpio_test_{pin}.c -lgpiod -std=c99 2>&1"
        success, output, error = run_command(client, compile_cmd, timeout=10)
        
        if success:
            run_cmd = f"sudo /tmp/gpio_test_{pin} 2>&1 || /tmp/gpio_test_{pin} 2>&1"
            success, output, error = run_command(client, run_cmd, timeout=5)
            if success:
                print(f"  ✅ GPIO {pin} is working")
            else:
                print(f"  ❌ GPIO {pin} failed: {error}")
        else:
            print(f"  ❌ Compilation failed: {error}")

def main():
    """메인 함수"""
    global ssh_client
    
    signal.signal(signal.SIGINT, signal_handler)
    if sys.platform != 'win32':
        signal.signal(signal.SIGTERM, signal_handler)
    
    print("=== Motor Diagnosis ===")
    print("Diagnosing why motors are not rotating...\n")
    
    client = create_ssh_client()
    if not client:
        return 1
    
    ssh_client = client
    
    try:
        # 1. SPI 디바이스 확인
        print("1. Checking SPI device...")
        success, output, error = run_command(client, "ls -l /dev/spidev0.0", timeout=5)
        if success:
            print("   ✅ SPI device exists")
        else:
            print("   ❌ SPI device not found")
        
        # 2. GPIO 핀 확인
        check_gpio_pins(client)
        
        # 3. DAC 출력 테스트
        print("\n" + "="*50)
        print("2. Testing DAC Output")
        print("="*50)
        print("This will test DAC output voltage.")
        print("Connect multimeter to DAC output pin to measure voltage.")
        print()
        
        test_dac_output(client, 0, [0, 25, 50, 75, 100, 0])
        
        print("\n" + "="*50)
        print("Diagnosis Summary")
        print("="*50)
        print("\nCheck the following:")
        print("1. DAC output voltage:")
        print("   - If voltage changes: DAC is working")
        print("   - If voltage is always 0: DAC or SPI problem")
        print("2. Motor driver connection:")
        print("   - DAC output -> Motor driver input")
        print("   - Motor driver power supply")
        print("   - Motor driver enable signal")
        print("3. Motor connection:")
        print("   - Motor wires to driver")
        print("   - Motor power supply")
        print("4. GPIO direction pins:")
        print("   - Reverse0 (GPIO 168) should control motor 0 direction")
        print("   - Reverse1 (GPIO 13) should control motor 1 direction")
        
        return 0
        
    except KeyboardInterrupt:
        print("\n\nInterrupted by user.")
        return 1
    except Exception as e:
        print(f"❌ Error: {e}")
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
    if sys.platform == 'win32':
        import io
        try:
            sys.stdout = io.TextIOWrapper(sys.stdout.buffer, encoding='utf-8')
            sys.stderr = io.TextIOWrapper(sys.stderr.buffer, encoding='utf-8')
        except:
            pass
    
    sys.exit(main())



