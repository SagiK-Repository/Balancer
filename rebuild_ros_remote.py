#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
원격 ROS 빌드 및 업데이트 스크립트
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

def run_command(client, command, timeout=60, show_output=True):
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
    print("원격 ROS 빌드 및 업데이트")
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
        
        # 코드 빌드
        print("3. ROS 패키지 빌드 중...")
        print("   (이 작업은 몇 분이 걸릴 수 있습니다)\n")
        build_cmd = f"{env_cmd} && cd ~/catkin_ws && catkin_make --pkg balance_robot_nodes"
        success, output, error = run_command(client, build_cmd, timeout=120, show_output=True)
        
        if not success:
            print(f"\n❌ 빌드 실패!")
            print(f"Error output:\n{error}")
            return 1
        
        print("\n   ✅ 빌드 완료!\n")
        
        # 빌드 결과 확인
        print("4. 빌드 결과 확인 중...")
        check_cmd = f"{env_cmd} && ls -lh ~/catkin_ws/devel/lib/balance_robot_nodes/spi_hardware_node 2>/dev/null || echo '파일 없음'"
        success, output, error = run_command(client, check_cmd, timeout=5, show_output=True)
        print()
        
        # 메시지 파일 확인
        print("5. 메시지 파일 확인 중...")
        msg_check_cmd = f"{env_cmd} && rosmsg show balance_robot_nodes/MotorStatus | grep -E '(raw_hall_data|motor.*hall)' || echo '메시지 확인 실패'"
        success, output, error = run_command(client, msg_check_cmd, timeout=5, show_output=True)
        print()
        
        print("="*80)
        print("✅ 빌드 및 업데이트 완료!")
        print("="*80)
        print()
        print("다음 명령어로 실행할 수 있습니다:")
        print()
        print(f"  export ROS_MASTER_URI='http://{current_ip}:11311'")
        print(f"  export ROS_IP='{current_ip}'")
        print("  source ~/catkin_ws/devel/setup.bash")
        print("  rosrun balance_robot_nodes spi_hardware_node")
        print()
        print("또는 토픽 확인:")
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


