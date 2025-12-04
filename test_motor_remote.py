#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
원격 서버에서 SPI 장치 및 모터 테스트
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

def main():
    """메인 함수"""
    print("=== Remote Motor Test ===")
    
    client = create_ssh_client()
    if not client:
        return 1
    
    try:
        # 1. 환경 설정
        setup_cmd = "cd ~/catkin_ws && source devel/setup.bash"
        
        # 2. SPI 디바이스 확인
        print("\n" + "="*50)
        print("1. Checking SPI devices...")
        print("="*50)
        run_command(client, "ls -l /dev/spidev* 2>/dev/null || echo 'No SPI devices found'", 
                   "SPI Devices")
        
        # 3. I2C 디바이스 확인
        print("\n" + "="*50)
        print("2. Checking I2C devices...")
        print("="*50)
        run_command(client, "ls -l /dev/i2c-* 2>/dev/null || echo 'No I2C devices found'", 
                   "I2C Devices")
        
        # 4. 하드웨어 테스트 실행
        print("\n" + "="*50)
        print("3. Running hardware tests...")
        print("="*50)
        
        # SPI 테스트
        test_cmd = f"{setup_cmd} && rosrun integrated_balancer hardware_test spi /dev/spidev0.0"
        success, _ = run_command(client, test_cmd, "SPI Device Test")
        
        # I2C 테스트
        test_cmd = f"{setup_cmd} && rosrun integrated_balancer hardware_test i2c /dev/i2c-1"
        success, _ = run_command(client, test_cmd, "I2C Device Test")
        
        # MPU6050 테스트
        test_cmd = f"{setup_cmd} && timeout 5 rosrun integrated_balancer hardware_test mpu6050"
        success, _ = run_command(client, test_cmd, "MPU6050 Sensor Test")
        
        # HardwareSPI 클래스 테스트
        test_cmd = f"{setup_cmd} && rosrun integrated_balancer hardware_test hardware_spi /dev/spidev0.0"
        success, _ = run_command(client, test_cmd, "HardwareSPI Class Test")
        
        # 5. DAC 테스트 (모터 출력 테스트)
        print("\n" + "="*50)
        print("4. Testing DAC (Motor Output)...")
        print("="*50)
        print("This will test motor output. Make sure motors are connected!")
        print("Testing DAC0 (CS pin 19)...")
        
        test_cmd = f"{setup_cmd} && timeout 10 rosrun integrated_balancer hardware_test dac /dev/spidev0.0 19"
        success, _ = run_command(client, test_cmd, "DAC0 Test")
        
        if success:
            print("\n✅ DAC0 test completed. Check motor 0 movement.")
        
        print("\nTesting DAC1 (CS pin 20)...")
        test_cmd = f"{setup_cmd} && timeout 10 rosrun integrated_balancer hardware_test dac /dev/spidev0.0 20"
        success, _ = run_command(client, test_cmd, "DAC1 Test")
        
        if success:
            print("\n✅ DAC1 test completed. Check motor 1 movement.")
        
        # 6. GPIO 테스트 (방향 제어)
        print("\n" + "="*50)
        print("5. Testing GPIO (Direction Control)...")
        print("="*50)
        
        test_cmd = f"{setup_cmd} && rosrun integrated_balancer hardware_test gpio 168 out"
        success, _ = run_command(client, test_cmd, "GPIO Reverse0 Test")
        
        test_cmd = f"{setup_cmd} && rosrun integrated_balancer hardware_test gpio 13 out"
        success, _ = run_command(client, test_cmd, "GPIO Reverse1 Test")
        
        # 7. 통합 테스트 요약
        print("\n" + "="*50)
        print("Test Summary")
        print("="*50)
        print("✅ SPI device check completed")
        print("✅ I2C device check completed")
        print("✅ Hardware tests completed")
        print("✅ DAC (Motor) tests completed")
        print("✅ GPIO tests completed")
        print("\nIf motors moved during DAC test, hardware is working correctly!")
        
        return 0
        
    except Exception as e:
        print(f"❌ Error: {e}")
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



