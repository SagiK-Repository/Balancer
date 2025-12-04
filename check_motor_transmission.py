#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
모터 제어 신호 전송 경로 확인
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
    print("="*70)
    print("모터 제어 신호 전송 경로 확인")
    print("="*70 + "\n")
    
    client = create_ssh_client()
    if not client:
        return 1
    
    try:
        # 현재 IP 확인
        success, output, error = run_command(client, "hostname -I | awk '{print $1}'", timeout=3)
        if success:
            current_ip = output.strip()
        else:
            current_ip = "192.168.0.28"
        
        setup_cmd = "cd ~/catkin_ws && source devel/setup.bash"
        env_cmd = f"export ROS_MASTER_URI='http://{current_ip}:11311' && export ROS_IP='{current_ip}' && {setup_cmd}"
        
        # 1. ROS 토픽 확인
        print("1. ROS 토픽 확인")
        print("-" * 70)
        
        success, output, error = run_command(client, f"{env_cmd} && timeout 3 rostopic list 2>&1", timeout=5)
        
        if success and output:
            topics = [line.strip() for line in output.strip().split('\n') if line.strip()]
            motor_topics = [t for t in topics if 'motor' in t.lower()]
            
            print("   📋 Motor 관련 토픽:")
            for topic in motor_topics:
                print(f"      - {topic}")
                
                # 토픽 정보 확인
                success2, output2, error2 = run_command(client, f"{env_cmd} && timeout 2 rostopic info {topic} 2>&1", timeout=4)
                if success2 and output2:
                    lines = output2.strip().split('\n')
                    for line in lines[:5]:
                        if line.strip():
                            print(f"        {line.strip()}")
        
        # 2. /motor_command 토픽 메시지 타입 확인
        print("\n2. /motor_command 메시지 구조")
        print("-" * 70)
        
        success, output, error = run_command(client, f"{env_cmd} && timeout 2 rostopic type /motor_command 2>&1", timeout=4)
        if success and output:
            msg_type = output.strip()
            print(f"   메시지 타입: {msg_type}")
            
            # 메시지 정의 확인
            success2, output2, error2 = run_command(client, f"{env_cmd} && timeout 2 rosmsg show {msg_type} 2>&1", timeout=4)
            if success2 and output2:
                print(f"\n   메시지 필드:")
                for line in output2.strip().split('\n'):
                    if line.strip():
                        print(f"      {line.strip()}")
        
        # 3. 노드 정보 확인
        print("\n3. SPI Hardware Node 정보")
        print("-" * 70)
        
        success, output, error = run_command(client, f"{env_cmd} && timeout 3 rosnode info /spi_hardware_node 2>&1", timeout=5)
        
        if success and output:
            lines = output.strip().split('\n')
            in_subscriptions = False
            
            for line in lines:
                if 'Subscriptions:' in line:
                    in_subscriptions = True
                    print(f"   {line.strip()}")
                    continue
                elif 'Publications:' in line or 'Services:' in line:
                    in_subscriptions = False
                
                if in_subscriptions:
                    print(f"      {line.strip()}")
        
        # 4. 실제 전송 경로 확인 (코드 분석 결과)
        print("\n4. 모터 제어 신호 전송 경로 (코드 분석)")
        print("-" * 70)
        
        print("   📊 전송 경로:")
        print("      1. ROS 토픽: /motor_command")
        print("         └─> MotorCommand 메시지 (motor0_output, motor1_output, motor0_direction, motor1_direction)")
        print()
        print("      2. SPI Hardware Node: motorCommandCallback()")
        print("         └─> current_command_ 저장")
        print("         └─> applyMotorCommand() 호출")
        print()
        print("      3. applyMotorCommand()")
        print("         └─> setMotorDirection(0/1, forward/reverse)  [GPIO 제어]")
        print("         └─> setMotorSpeed(0/1, speed_percent)       [DAC 제어]")
        print()
        print("      4. setMotorSpeed()")
        print("         └─> dac0_->setOutputPercent(speed_percent)   [Motor 0]")
        print("         └─> dac1_->setOutputPercent(speed_percent)   [Motor 1]")
        print()
        print("      5. MCP4921::setOutputPercent()")
        print("         └─> 0-100% → 0-4095 (12-bit) 변환")
        print("         └─> setOutput(value) 호출")
        print()
        print("      6. MCP4921::setOutput()")
        print("         └─> 16-bit 명령 구성 (CS, Buffered, Gain, Active, 12-bit value)")
        print("         └─> CS 라인 LOW")
        print("         └─> BitBangSPI::transfer16() 호출")
        print("         └─> CS 라인 HIGH")
        print()
        print("      7. BitBangSPI::transfer16()")
        print("         └─> GPIO 핀 제어 (MOSI, MISO, SCLK)")
        print("         └─> 비트 단위 SPI 통신")
        print()
        print("      8. 하드웨어")
        print("         └─> MCP4921 DAC 칩")
        print("         └─> 아날로그 전압 출력")
        print("         └─> 모터 드라이버")
        print("         └─> 모터")
        
        # 5. 현재 설정된 GPIO 핀 확인
        print("\n5. GPIO 핀 설정 확인")
        print("-" * 70)
        
        success, output, error = run_command(client, f"{env_cmd} && timeout 3 rosparam list 2>&1", timeout=5)
        
        if success and output:
            params = [line.strip() for line in output.strip().split('\n') if line.strip()]
            gpio_params = [p for p in params if 'pin' in p.lower()]
            
            if gpio_params:
                print("   📋 GPIO 관련 파라미터:")
                for param in gpio_params[:10]:
                    success2, output2, error2 = run_command(client, f"{env_cmd} && timeout 2 rosparam get {param} 2>&1", timeout=4)
                    if success2 and output2:
                        print(f"      {param}: {output2.strip()}")
            else:
                print("   ℹ️  ROS 파라미터에서 GPIO 핀 설정을 찾을 수 없음")
                print("   ℹ️  기본값 (hardware_interface.h):")
                print("      - MOSI: 10")
                print("      - MISO: 9")
                print("      - SCLK: 11")
                print("      - CS (BitBangSPI): 8")
                print("      - SS0 (DAC0): 19")
                print("      - SS1 (DAC1): 20")
                print("      - Latch: 38")
                print("      - Reverse0: GPIO 핀")
                print("      - Reverse1: GPIO 핀")
        
        # 6. 실제 전송 테스트
        print("\n6. 실제 전송 테스트")
        print("-" * 70)
        
        print("   테스트: 모터 명령 발행 후 상태 확인")
        
        # 모터 명령 발행
        test_cmd = f"{env_cmd} && timeout 1 rostopic pub -1 /motor_command balance_robot_nodes/MotorCommand '{{motor0_output: 25.0, motor1_output: 25.0, motor0_direction: true, motor1_direction: true, emergency_stop: false}}' 2>&1"
        success, output, error = run_command(client, test_cmd, timeout=3)
        
        if success:
            print("   ✅ 모터 명령 발행 완료 (25% 속도)")
        else:
            print(f"   ⚠️  명령 발행 실패: {error}")
        
        time.sleep(1)
        
        # 모터 상태 확인
        success, output, error = run_command(client, f"{env_cmd} && timeout 2 rostopic echo /motor_status -n 1 2>&1 | grep -E 'motor0_actual_output|motor1_actual_output|dac0_ok|dac1_ok'", timeout=4)
        
        if success and output:
            print("\n   📊 모터 상태:")
            for line in output.strip().split('\n'):
                if line.strip():
                    print(f"      {line.strip()}")
        else:
            print("   ⚠️  모터 상태를 읽을 수 없음")
        
        # 7. 요약
        print("\n" + "="*70)
        print("전송 경로 요약")
        print("="*70)
        
        print("\n📡 신호 흐름:")
        print("   ROS Topic (/motor_command)")
        print("   → SPI Hardware Node (motorCommandCallback)")
        print("   → applyMotorCommand()")
        print("   → setMotorSpeed()")
        print("   → MCP4921::setOutputPercent()")
        print("   → BitBangSPI::transfer16()")
        print("   → GPIO 핀 제어 (MOSI, SCLK, CS)")
        print("   → MCP4921 DAC 칩")
        print("   → 아날로그 전압 출력")
        print("   → 모터 드라이버")
        print("   → 모터")
        
        print("\n🔧 사용되는 하드웨어:")
        print("   - BitBangSPI: GPIO 핀을 통한 소프트웨어 SPI")
        print("   - MCP4921: 12-bit DAC 칩 (2개: DAC0, DAC1)")
        print("   - GPIO: 방향 제어 (Reverse0, Reverse1)")
        
        print("\n⚠️  주의사항:")
        print("   - BitBangSPI는 소프트웨어로 구현된 SPI (하드웨어 SPI보다 느림)")
        print("   - CS 라인 요청 실패 시에도 동작 (홀 센서 읽기에는 문제 없음)")
        print("   - DAC 초기화 실패 시 더미 모드로 동작")
        
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



