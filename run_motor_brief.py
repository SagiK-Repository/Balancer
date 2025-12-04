#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
모터 잠시 실행
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
    print("모터 잠시 실행")
    print("="*60 + "\n")
    
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
        
        # 모터 속도 설정 (30%)
        speed = 30.0
        duration = 10  # 10초
        
        print(f"모터 설정:")
        print(f"  속도: {speed}%")
        print(f"  시간: {duration}초")
        print()
        
        # 모터 명령 발행
        print("모터 시작...")
        motor_cmd = f"{env_cmd} && rostopic pub -1 /motor_command balance_robot_nodes/MotorCommand '{{motor0_output: {speed}, motor1_output: {speed}, motor0_direction: true, motor1_direction: true, emergency_stop: false}}' 2>&1"
        success, output, error = run_command(client, motor_cmd, timeout=3)
        
        if success:
            print(f"✅ 모터 명령 발행 완료 ({speed}% 속도)")
        else:
            print(f"⚠️  명령 발행: {error if error else 'timeout'}")
        
        # 모터 상태 확인
        print(f"\n모터 실행 중... ({duration}초)")
        for i in range(duration):
            time.sleep(1)
            if (i + 1) % 2 == 0:
                print(f"  {i + 1}초 경과...")
        
        # 모터 정지
        print("\n모터 정지...")
        stop_cmd = f"{env_cmd} && rostopic pub -1 /motor_command balance_robot_nodes/MotorCommand '{{motor0_output: 0.0, motor1_output: 0.0, motor0_direction: true, motor1_direction: true, emergency_stop: false}}' 2>&1"
        success, output, error = run_command(client, stop_cmd, timeout=3)
        
        if success:
            print("✅ 모터 정지 완료")
        else:
            print(f"⚠️  정지 명령: {error if error else 'timeout'}")
        
        # 최종 상태 확인
        print("\n최종 모터 상태 확인...")
        status_cmd = f"{env_cmd} && timeout 2 rostopic echo /motor_status -n 1 2>&1 | grep -E 'motor0_actual_output|motor1_actual_output'"
        success, output, error = run_command(client, status_cmd, timeout=4)
        
        if success and output:
            print("  모터 상태:")
            for line in output.strip().split('\n'):
                if line.strip():
                    print(f"    {line.strip()}")
        
        print("\n✅ 완료!")
        
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



