#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
원격에서 홀 센서 원본 데이터 확인
"""

import paramiko
import sys
import time
import re

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
    print("="*80)
    print("홀 센서 원본 데이터 확인")
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
        
        # ROS 마스터 확인
        print("1. ROS 마스터 확인 중...")
        cmd = f"{env_cmd} && timeout 2 rostopic list > /dev/null 2>&1 && echo 'OK' || echo 'FAIL'"
        success, output, error = run_command(client, cmd, timeout=5)
        if 'OK' in output:
            print("   ✅ ROS 마스터 연결 성공\n")
        else:
            print("   ⚠️  ROS 마스터 연결 실패 - roscore를 시작합니다...\n")
            # roscore 시작
            roscore_cmd = f"{env_cmd} && nohup roscore > /tmp/roscore.log 2>&1 &"
            client.exec_command(roscore_cmd, timeout=2)
            time.sleep(3)
        
        # 메시지 정의 확인
        print("2. MotorStatus 메시지 정의 확인 중...")
        cmd = f"{env_cmd} && rosmsg show balance_robot_nodes/MotorStatus"
        success, output, error = run_command(client, cmd, timeout=5)
        if success:
            print(output)
            if 'raw_hall_data' in output:
                print("   ✅ raw_hall_data 필드 존재\n")
            else:
                print("   ⚠️  raw_hall_data 필드가 없습니다!\n")
        else:
            print(f"   ❌ 메시지 확인 실패: {error}\n")
        
        # 토픽 데이터 확인
        print("3. /motor_status 토픽 데이터 확인 중 (5초간)...")
        print("-" * 80)
        cmd = f"{env_cmd} && timeout 5 rostopic echo /motor_status 2>&1 | head -50"
        success, output, error = run_command(client, cmd, timeout=8)
        
        if success and output:
            print(output)
            
            # raw_hall_data 필드 확인
            if 'raw_hall_data' in output:
                print("\n   ✅ raw_hall_data 필드 발견!")
                # raw_hall_data 값 추출
                matches = re.findall(r'raw_hall_data:\s*(\d+)', output)
                if matches:
                    print(f"   값: {matches[0]}")
                matches_bin = re.findall(r'raw_hall_data_binary:\s*"([^"]+)"', output)
                if matches_bin:
                    print(f"   2진수: {matches_bin[0]}")
            else:
                print("\n   ⚠️  raw_hall_data 필드가 토픽에 없습니다!")
                print("   메시지가 업데이트되지 않았을 수 있습니다.")
        else:
            print(f"   ❌ 토픽 데이터 수신 실패: {error}")
        
        print("\n" + "="*80)
        print("SSH에서 직접 확인하는 방법:")
        print("="*80)
        print()
        print(f"export ROS_MASTER_URI='http://{current_ip}:11311'")
        print(f"export ROS_IP='{current_ip}'")
        print("source ~/catkin_ws/devel/setup.bash")
        print()
        print("# 전체 메시지 확인:")
        print("rostopic echo /motor_status")
        print()
        print("# 홀 센서 관련 필드만 확인:")
        print("rostopic echo /motor_status | grep -E '(raw_hall|motor.*hall)'")
        print()
        print("# 메시지 정의 확인:")
        print("rosmsg show balance_robot_nodes/MotorStatus")
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


