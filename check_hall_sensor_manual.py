#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
ROS 홀 센서 데이터 확인 가이드
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
        output = ""
        error = ""
        
        while True:
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
    print("="*80)
    print("ROS 홀 센서 데이터 확인 가이드")
    print("="*80)
    print()
    
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
        
        print("1. 사용 가능한 토픽 목록:")
        print("-" * 80)
        cmd = f"{env_cmd} && rostopic list"
        success, output, error = run_command(client, cmd, timeout=5)
        if success:
            print(output)
        else:
            print(f"❌ Error: {error}")
        print()
        
        print("2. /motor_status 토픽 타입 확인:")
        print("-" * 80)
        cmd = f"{env_cmd} && rostopic type /motor_status"
        success, output, error = run_command(client, cmd, timeout=5)
        if success:
            print(output)
        else:
            print(f"⚠️  토픽이 없거나 접근할 수 없습니다: {error}")
        print()
        
        print("3. /motor_status 토픽 발행 주기 확인 (3초간):")
        print("-" * 80)
        cmd = f"{env_cmd} && timeout 3 rostopic hz /motor_status 2>&1 | head -10"
        success, output, error = run_command(client, cmd, timeout=5)
        if success:
            print(output)
        else:
            print(f"⚠️  토픽이 발행되지 않습니다: {error}")
        print()
        
        print("4. 홀 센서 데이터 샘플 (5초간):")
        print("-" * 80)
        print("홀 센서 관련 필드만 표시합니다.")
        print()
        cmd = f"{env_cmd} && timeout 5 rostopic echo /motor_status 2>&1 | grep -E '(motor0_hall|motor1_hall)' | head -20"
        success, output, error = run_command(client, cmd, timeout=8)
        if success and output.strip():
            print(output)
        else:
            print("⚠️  데이터를 받을 수 없습니다.")
            print("   roscore와 spi_hardware_node가 실행 중인지 확인하세요.")
        print()
        
        print("="*80)
        print("직접 ROS 명령어 사용 방법:")
        print("="*80)
        print()
        print("SSH로 접속한 후 다음 명령어를 사용하세요:")
        print()
        print("1. ROS 환경 설정:")
        print(f"   export ROS_MASTER_URI='http://{current_ip}:11311'")
        print(f"   export ROS_IP='{current_ip}'")
        print("   source ~/catkin_ws/devel/setup.bash")
        print()
        print("2. 토픽 목록 확인:")
        print("   rostopic list")
        print()
        print("3. 홀 센서 데이터 실시간 확인:")
        print("   rostopic echo /motor_status")
        print()
        print("4. 홀 센서 데이터만 확인:")
        print("   rostopic echo /motor_status | grep -E '(motor0_hall|motor1_hall)'")
        print()
        print("5. 토픽 발행 주기 확인:")
        print("   rostopic hz /motor_status")
        print()
        print("6. 토픽 타입 확인:")
        print("   rostopic type /motor_status")
        print()
        print("7. 토픽 정보 확인:")
        print("   rostopic info /motor_status")
        print()
        
        return 0
        
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


