#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
모터 간단 실행
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

def run_command_background(client, command):
    """백그라운드 명령 실행"""
    try:
        stdin, stdout, stderr = client.exec_command(command, timeout=1)
        return True
    except:
        return False

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
        current_ip = "192.168.0.28"
        
        setup_cmd = "cd ~/catkin_ws && source devel/setup.bash"
        env_cmd = f"export ROS_MASTER_URI='http://{current_ip}:11311' && export ROS_IP='{current_ip}' && {setup_cmd}"
        
        # 모터 속도 설정
        speed = 30.0
        duration = 10  # 10초
        
        print(f"모터 설정:")
        print(f"  속도: {speed}%")
        print(f"  시간: {duration}초")
        print()
        
        # 모터 명령 발행 (백그라운드로 지속 발행)
        print("모터 시작...")
        motor_cmd = f"{env_cmd} && (for i in $(seq 1 {duration}); do rostopic pub -1 /motor_command balance_robot_nodes/MotorCommand '{{motor0_output: {speed}, motor1_output: {speed}, motor0_direction: true, motor1_direction: true, emergency_stop: false}}' 2>/dev/null; sleep 1; done; rostopic pub -1 /motor_command balance_robot_nodes/MotorCommand '{{motor0_output: 0.0, motor1_output: 0.0, motor0_direction: true, motor1_direction: true, emergency_stop: false}}' 2>/dev/null) > /tmp/motor_run.log 2>&1 &"
        run_command_background(client, motor_cmd)
        time.sleep(1)
        
        print(f"✅ 모터 명령 시작 ({speed}% 속도, {duration}초)")
        print(f"\n모터 실행 중... ({duration}초)")
        
        # 진행 상황 표시
        for i in range(duration):
            time.sleep(1)
            remaining = duration - (i + 1)
            if (i + 1) % 2 == 0 or remaining <= 3:
                print(f"  {i + 1}초 경과... (남은 시간: {remaining}초)")
        
        # 모터 정지 대기
        time.sleep(1)
        
        # 최종 상태 확인
        print("\n최종 모터 상태 확인...")
        time.sleep(1)
        
        # 간단한 상태 확인
        check_cmd = f"{env_cmd} && timeout 2 rostopic echo /motor_status -n 1 2>&1 | head -20"
        stdin, stdout, stderr = client.exec_command(check_cmd, timeout=4)
        time.sleep(1)
        
        if stdout.channel.recv_ready():
            output = stdout.channel.recv(4096).decode('utf-8', errors='ignore')
            lines = output.split('\n')
            for line in lines:
                if 'motor0_actual_output' in line or 'motor1_actual_output' in line or 'dac0_ok' in line or 'dac1_ok' in line:
                    print(f"  {line.strip()}")
        
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

