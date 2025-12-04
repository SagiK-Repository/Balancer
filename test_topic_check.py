#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
토픽 발행 확인 스크립트
"""

import paramiko
import time
import sys

HOSTNAME = "192.168.0.28"
USERNAME = "tbot3"
PASSWORD = "1234"

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
    print("="*80)
    print("토픽 발행 확인")
    print("="*80 + "\n")
    
    client = paramiko.SSHClient()
    client.set_missing_host_key_policy(paramiko.AutoAddPolicy())
    
    try:
        print(f"Connecting to {USERNAME}@{HOSTNAME}...")
        client.connect(HOSTNAME, username=USERNAME, password=PASSWORD, timeout=10)
        print("✅ Connected!\n")
        
        # IP 확인
        success, output, error = run_command(client, "hostname -I | awk '{print $1}'", timeout=3)
        if success:
            current_ip = output.strip()
        else:
            current_ip = "192.168.0.28"
        
        setup_cmd = "cd ~/catkin_ws && source devel/setup.bash"
        env_cmd = f"export ROS_MASTER_URI='http://{current_ip}:11311' && export ROS_IP='{current_ip}' && {setup_cmd}"
        
        # roscore 확인
        print("1. roscore 확인...")
        success, output, error = run_command(client, "ps aux | grep roscore | grep -v grep", timeout=3)
        if success and output.strip():
            print("   ✅ roscore 실행 중")
        else:
            print("   ❌ roscore 실행 안 됨")
        
        # 토픽 목록 확인
        print("\n2. 토픽 목록 확인...")
        cmd = f"{env_cmd} && timeout 3 rostopic list"
        success, output, error = run_command(client, cmd, timeout=5)
        if success:
            print("   ✅ 토픽 목록:")
            for line in output.strip().split('\n'):
                if line.strip():
                    print(f"      - {line.strip()}")
        else:
            print(f"   ❌ 실패: {error}")
        
        # motor_status 토픽 확인
        print("\n3. /motor_status 토픽 확인...")
        cmd = f"{env_cmd} && timeout 3 rostopic list | grep motor_status"
        success, output, error = run_command(client, cmd, timeout=5)
        if success and 'motor_status' in output:
            print("   ✅ /motor_status 토픽 존재")
            
            # 토픽 정보
            print("\n4. 토픽 정보 확인...")
            cmd = f"{env_cmd} && timeout 3 rostopic info /motor_status"
            success, output, error = run_command(client, cmd, timeout=5)
            if success:
                print(f"   {output}")
            else:
                print(f"   ❌ 실패: {error}")
            
            # 한 번만 읽기 시도
            print("\n5. 토픽 데이터 읽기 시도 (5초)...")
            cmd = f"{env_cmd} && timeout 5 rostopic echo /motor_status -n 1"
            success, output, error = run_command(client, cmd, timeout=7)
            if success:
                if output.strip():
                    print("   ✅ 데이터 수신 성공:")
                    print("   " + "-"*70)
                    for line in output.strip().split('\n')[:20]:  # 처음 20줄만
                        print(f"   {line}")
                    if len(output.strip().split('\n')) > 20:
                        print("   ... (더 많은 데이터)")
                else:
                    print("   ⚠️  명령은 성공했지만 데이터가 없음")
            else:
                print(f"   ❌ 실패: {error}")
        else:
            print("   ❌ /motor_status 토픽이 없음")
        
        # spi_hardware_node 확인
        print("\n6. spi_hardware_node 프로세스 확인...")
        success, output, error = run_command(client, "ps aux | grep spi_hardware_node | grep -v grep", timeout=3)
        if success and output.strip():
            print("   ✅ spi_hardware_node 실행 중")
            print(f"   {output.strip()}")
        else:
            print("   ❌ spi_hardware_node 실행 안 됨")
        
        # 로그 확인
        print("\n7. 로그 확인...")
        cmd = "tail -20 /tmp/spi_node.log 2>/dev/null || echo '로그 파일 없음'"
        success, output, error = run_command(client, cmd, timeout=3)
        if success:
            print("   /tmp/spi_node.log:")
            for line in output.strip().split('\n'):
                print(f"   {line}")
        
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
    sys.exit(main())


