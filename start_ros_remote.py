#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
원격 ROS 시작 스크립트
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

def run_command(client, command, timeout=10, show_output=False):
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
                chunk = stdout.channel.recv(4096).decode('utf-8', errors='ignore')
                output += chunk
                if show_output:
                    print(chunk, end='', flush=True)
            
            if stderr.channel.recv_stderr_ready():
                chunk = stderr.channel.recv_stderr(4096).decode('utf-8', errors='ignore')
                error += chunk
                if show_output:
                    print(chunk, end='', flush=True, file=sys.stderr)
            
            if stdout.channel.exit_status_ready():
                exit_status = stdout.channel.recv_exit_status()
                while stdout.channel.recv_ready():
                    chunk = stdout.channel.recv(4096).decode('utf-8', errors='ignore')
                    output += chunk
                    if show_output:
                        print(chunk, end='', flush=True)
                while stderr.channel.recv_stderr_ready():
                    chunk = stderr.channel.recv_stderr(4096).decode('utf-8', errors='ignore')
                    error += chunk
                    if show_output:
                        print(chunk, end='', flush=True, file=sys.stderr)
                break
            
            time.sleep(0.05)
        
        return exit_status == 0, output, error
    except Exception as e:
        return False, "", str(e)

def main():
    """메인 함수"""
    print("="*80)
    print("원격 ROS 시작")
    print("="*80)
    print()
    
    client = create_ssh_client()
    if not client:
        return 1
    
    try:
        # 현재 IP 확인
        print("1. 현재 IP 확인 중...")
        success, output, error = run_command(client, "hostname -I | awk '{print $1}'", timeout=3, show_output=False)
        if success:
            current_ip = output.strip()
            print(f"   ✅ IP: {current_ip}\n")
        else:
            current_ip = "192.168.0.28"
            print(f"   ⚠️  IP 확인 실패, 기본값 사용: {current_ip}\n")
        
        setup_cmd = "cd ~/catkin_ws && source devel/setup.bash"
        env_cmd = f"export ROS_MASTER_URI='http://{current_ip}:11311' && export ROS_IP='{current_ip}' && {setup_cmd}"
        
        # 기존 프로세스 종료
        print("2. 기존 ROS 프로세스 종료 중...")
        run_command(client, "pkill -f spi_hardware_node", timeout=3, show_output=False)
        run_command(client, "pkill -f roscore", timeout=3, show_output=False)
        time.sleep(2)
        print("   ✅ 프로세스 종료 완료\n")
        
        # roscore 시작
        print("3. roscore 시작 중...")
        roscore_cmd = f"{env_cmd} && nohup roscore > /tmp/roscore.log 2>&1 &"
        client.exec_command(roscore_cmd, timeout=2)
        time.sleep(3)
        
        # roscore 실행 확인
        print("   roscore 실행 확인 중...")
        for i in range(5):
            success, output, error = run_command(client, "ps aux | grep roscore | grep -v grep", timeout=3, show_output=False)
            if success and output.strip():
                print("   ✅ roscore 실행 중\n")
                break
            time.sleep(1)
        else:
            print("   ⚠️  roscore 실행 확인 실패\n")
        
        # ROS 마스터 연결 확인
        print("4. ROS 마스터 연결 확인 중...")
        for i in range(5):
            cmd = f"{env_cmd} && timeout 2 rostopic list > /dev/null 2>&1"
            success, output, error = run_command(client, cmd, timeout=3, show_output=False)
            if success:
                print("   ✅ ROS 마스터 연결 성공\n")
                break
            time.sleep(1)
        else:
            print("   ⚠️  ROS 마스터 연결 실패\n")
        
        # SPI Hardware Node 시작
        print("5. SPI Hardware Node 시작 중...")
        spi_node_cmd = f"{env_cmd} && nohup rosrun balance_robot_nodes spi_hardware_node > /tmp/spi_node.log 2>&1 &"
        client.exec_command(spi_node_cmd, timeout=2)
        time.sleep(3)
        
        # spi_hardware_node 실행 확인
        print("   spi_hardware_node 실행 확인 중...")
        for i in range(5):
            success, output, error = run_command(client, "ps aux | grep spi_hardware_node | grep -v grep", timeout=3, show_output=False)
            if success and output.strip():
                print("   ✅ spi_hardware_node 실행 중\n")
                break
            time.sleep(1)
        else:
            print("   ⚠️  spi_hardware_node 실행 확인 실패\n")
        
        # 토픽 확인
        print("6. 토픽 확인 중...")
        time.sleep(2)
        cmd = f"{env_cmd} && timeout 3 rostopic list"
        success, output, error = run_command(client, cmd, timeout=5, show_output=True)
        print()
        
        print("="*80)
        print("✅ ROS 시작 완료!")
        print("="*80)
        print()
        print("다음 명령어로 토픽을 확인할 수 있습니다:")
        print()
        print(f"  export ROS_MASTER_URI='http://{current_ip}:11311'")
        print(f"  export ROS_IP='{current_ip}'")
        print("  source ~/catkin_ws/devel/setup.bash")
        print("  rostopic echo /motor_status")
        print()
        print("또는 홀 센서 데이터만 확인:")
        print("  rostopic echo /motor_status | grep -E '(raw_hall_data|motor.*hall)'")
        print()
        
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


