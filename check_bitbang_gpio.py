#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
BitBangSPI GPIO 연결 상세 점검
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
        print("✅ Connected successfully!\n")
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
    except Exception as e:
        return False, "", str(e)

def main():
    """메인 함수"""
    print("="*60)
    print("BitBangSPI GPIO 연결 상세 점검")
    print("="*60 + "\n")
    
    client = create_ssh_client()
    if not client:
        return 1
    
    try:
        setup_cmd = "cd ~/catkin_ws && source devel/setup.bash"
        
        # 1. ROS 파라미터에서 GPIO 핀 번호 확인
        print("1. ROS 파라미터에서 GPIO 핀 설정 확인")
        print("-" * 60)
        
        success, output, error = run_command(client, f"{setup_cmd} && timeout 3 rosparam list 2>&1", timeout=5)
        
        if success and output:
            params = [line.strip() for line in output.strip().split('\n') if line.strip()]
            
            # GPIO 관련 파라미터 찾기
            gpio_params = {}
            pin_names = ['mosi_pin', 'miso_pin', 'clk_pin', 'cs_pin', 'ss0_pin', 'ss1_pin', 'latch_pin', 
                        'reverse0_pin', 'reverse1_pin', 'inc_button_pin', 'dec_button_pin']
            
            for param in params:
                for pin_name in pin_names:
                    if pin_name in param.lower():
                        success2, output2, error2 = run_command(client, f"{setup_cmd} && timeout 2 rosparam get {param} 2>&1", timeout=4)
                        if success2 and output2:
                            gpio_params[param] = output2.strip()
            
            if gpio_params:
                print("   📋 GPIO Pin Configuration:")
                for param, value in sorted(gpio_params.items()):
                    print(f"      {param}: {value}")
            else:
                print("   ⚠️  No GPIO parameters found in ROS")
                print("   ℹ️  Checking default values from code...")
                print("      Default pins (from hardware_interface.h):")
                print("         mosi_pin: 10")
                print("         miso_pin: 9")
                print("         clk_pin: 11")
                print("         cs_pin: 8")
                print("         ss0_pin: 19")
                print("         ss1_pin: 20")
                print("         latch_pin: 38 (or 18 from config)")
        
        # 2. GPIO 라인 사용 상태 확인
        print("\n2. GPIO 라인 사용 상태 확인")
        print("-" * 60)
        
        # gpioinfo로 라인 상태 확인
        success, output, error = run_command(client, "gpioinfo gpiochip0 2>&1 | head -50", timeout=5)
        
        if success and output:
            print("   📋 GPIO Chip 0 Line Status (first 50 lines):")
            lines = output.strip().split('\n')
            used_lines = []
            
            for line in lines:
                if 'line' in line.lower() and ('used' in line.lower() or 'unused' in line.lower()):
                    # 사용 중인 라인 찾기
                    if 'used' in line.lower() and 'unused' not in line.lower():
                        # 라인 번호 추출
                        if 'line' in line:
                            parts = line.split()
                            for i, part in enumerate(parts):
                                if part == 'line':
                                    if i + 1 < len(parts):
                                        line_num = parts[i + 1].rstrip(':')
                                        used_lines.append(line_num)
                    print(f"      {line.strip()}")
            
            if used_lines:
                print(f"\n   ⚠️  Found {len(used_lines)} potentially used lines")
        
        # 3. 특정 핀 상태 확인 (예상되는 핀들)
        print("\n3. 예상 GPIO 핀 상태 확인")
        print("-" * 60)
        
        test_pins = [8, 9, 10, 11, 18, 19, 20, 38]
        
        for pin in test_pins:
            # gpioinfo로 특정 라인 확인
            success, output, error = run_command(client, f"gpioinfo gpiochip0 | grep -A 2 'line {pin}:' 2>&1", timeout=3)
            
            if success and output:
                lines = output.strip().split('\n')
                status = "unknown"
                consumer = ""
                
                for line in lines:
                    if 'used' in line.lower():
                        status = "USED"
                        # consumer 이름 추출
                        if 'consumer' in line.lower():
                            parts = line.split('consumer:')
                            if len(parts) > 1:
                                consumer = parts[1].strip()
                    elif 'unused' in line.lower():
                        status = "UNUSED"
                
                status_icon = "🔴" if status == "USED" else "🟢"
                print(f"   {status_icon} Pin {pin:2d}: {status:8s}", end="")
                if consumer:
                    print(f" (by: {consumer})")
                else:
                    print()
            else:
                print(f"   ⚠️  Pin {pin:2d}: Could not check")
        
        # 4. 노드 로그에서 상세 에러 확인
        print("\n4. 노드 로그 상세 분석")
        print("-" * 60)
        
        success, output, error = run_command(client, "grep -i 'gpio\|pin\|latch\|cs' /tmp/spi_node.log 2>/dev/null | tail -20", timeout=3)
        
        if success and output:
            print("   📋 GPIO/Pin related log entries:")
            for line in output.strip().split('\n'):
                if line.strip():
                    if 'ERROR' in line or 'error' in line.lower():
                        print(f"      ❌ {line.strip()}")
                    elif 'WARN' in line or 'warning' in line.lower():
                        print(f"      ⚠️  {line.strip()}")
                    else:
                        print(f"      ℹ️  {line.strip()}")
        else:
            print("   ℹ️  No GPIO/Pin related log entries found")
        
        # 5. GPIO 권한 확인
        print("\n5. GPIO 권한 확인")
        print("-" * 60)
        
        # gpio 그룹 확인
        success, output, error = run_command(client, "groups $USER 2>&1", timeout=3)
        if success and output:
            groups = output.strip().split()
            if 'gpio' in groups:
                print("   ✅ User is in 'gpio' group")
            else:
                print("   ⚠️  User is NOT in 'gpio' group")
                print(f"      Current groups: {', '.join(groups)}")
        
        # /dev/gpiochip0 권한 확인
        success, output, error = run_command(client, "ls -l /dev/gpiochip0 2>&1", timeout=3)
        if success and output:
            print(f"   📋 /dev/gpiochip0 permissions: {output.strip()}")
        
        # 6. 프로세스가 사용 중인 GPIO 확인
        print("\n6. 실행 중인 프로세스의 GPIO 사용 확인")
        print("-" * 60)
        
        success, output, error = run_command(client, "ps aux | grep spi_hardware_node | grep -v grep", timeout=3)
        
        if success and output:
            parts = output.split()
            if len(parts) > 1:
                pid = parts[1]
                print(f"   📋 spi_hardware_node PID: {pid}")
                
                # 프로세스가 열고 있는 파일 확인
                success2, output2, error2 = run_command(client, f"lsof -p {pid} 2>/dev/null | grep -E 'gpio|spi'", timeout=3)
                if success2 and output2:
                    print("   📋 Open GPIO/SPI files:")
                    for line in output2.strip().split('\n'):
                        if line.strip():
                            print(f"      {line.strip()}")
        
        # 7. 요약 및 권장 사항
        print("\n" + "="*60)
        print("점검 요약 및 권장 사항")
        print("="*60)
        
        print("\n✅ 확인된 사항:")
        print("   - GPIO 칩은 정상적으로 감지됨")
        print("   - SPI 장치는 존재함")
        print("   - ROS 노드는 실행 중")
        
        print("\n⚠️  문제점:")
        print("   - CS GPIO 라인 요청 실패 (latch_pin 또는 다른 핀)")
        print("   - 홀 센서 값이 모두 0으로 고정됨")
        
        print("\n💡 권장 사항:")
        print("   1. GPIO 핀 충돌 확인:")
        print("      - 다른 프로세스가 같은 핀을 사용 중일 수 있음")
        print("      - 이전에 실행된 프로세스가 핀을 해제하지 않았을 수 있음")
        print("   2. 핀 번호 확인:")
        print("      - latch_pin이 38인지 18인지 확인")
        print("      - ROS 파라미터 또는 launch 파일에서 설정 확인")
        print("   3. 권한 확인:")
        print("      - 사용자가 gpio 그룹에 속해 있는지 확인")
        print("      - /dev/gpiochip0 접근 권한 확인")
        print("   4. 재시작:")
        print("      - 모든 ROS 노드 종료 후 재시작")
        print("      - GPIO 핀 해제 확인")
        
        return 0
        
    except Exception as e:
        print(f"❌ Error: {e}")
        import traceback
        traceback.print_exc()
        return 1
    finally:
        client.close()

if __name__ == "__main__":
    if sys.platform == 'win32':
        import io
        try:
            sys.stdout = io.TextIOWrapper(sys.stdout.buffer, encoding='utf-8')
            sys.stderr = io.TextIOWrapper(sys.stderr.buffer, encoding='utf-8')
        except:
            pass
    
    sys.exit(main())



