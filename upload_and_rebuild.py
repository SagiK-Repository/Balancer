#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
메시지 파일 업로드 및 재빌드
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

def upload_file(sftp, local_path, remote_path):
    """파일 업로드"""
    try:
        sftp.put(local_path, remote_path)
        print(f"   ✅ 업로드 완료: {remote_path}")
        return True
    except Exception as e:
        print(f"   ❌ 업로드 실패: {e}")
        return False

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
    print("메시지 파일 업로드 및 재빌드")
    print("="*80)
    print()
    
    client = create_ssh_client()
    if not client:
        return 1
    
    try:
        # 현재 IP 확인
        success, output, error = run_command(client, "hostname -I | awk '{print $1}'", timeout=3, show_output=False)
        if success:
            current_ip = output.strip()
        else:
            current_ip = "192.168.0.28"
        
        setup_cmd = "cd ~/catkin_ws && source devel/setup.bash"
        env_cmd = f"export ROS_MASTER_URI='http://{current_ip}:11311' && export ROS_IP='{current_ip}' && {setup_cmd}"
        
        # 기존 프로세스 종료
        print("1. 기존 ROS 프로세스 종료 중...")
        run_command(client, "pkill -f spi_hardware_node", timeout=3, show_output=False)
        run_command(client, "pkill -f roscore", timeout=3, show_output=False)
        time.sleep(2)
        print("   ✅ 프로세스 종료 완료\n")
        
        # 파일 업로드
        print("2. 메시지 파일 업로드 중...")
        sftp = client.open_sftp()
        
        # 로컬 파일 읽기
        try:
            with open('src/balance_robot_nodes/msg/MotorStatus.msg', 'r', encoding='utf-8') as f:
                local_content = f.read()
        except Exception as e:
            print(f"   ❌ 로컬 파일 읽기 실패: {e}")
            return 1
        
        # 원격 파일에 쓰기
        remote_path = '/home/tbot3/catkin_ws/src/balance_robot_nodes/msg/MotorStatus.msg'
        try:
            with sftp.file(remote_path, 'w') as remote_file:
                remote_file.write(local_content)
            print(f"   ✅ 업로드 완료: {remote_path}\n")
        except Exception as e:
            print(f"   ❌ 업로드 실패: {e}\n")
            return 1
        finally:
            sftp.close()
        
        # 코드 빌드
        print("3. ROS 패키지 재빌드 중...")
        print("   (이 작업은 몇 분이 걸릴 수 있습니다)\n")
        build_cmd = f"{env_cmd} && cd ~/catkin_ws && catkin_make --pkg balance_robot_nodes"
        success, output, error = run_command(client, build_cmd, timeout=120, show_output=True)
        
        if not success:
            print(f"\n❌ 빌드 실패!")
            print(f"Error output:\n{error}")
            return 1
        
        print("\n   ✅ 빌드 완료!\n")
        
        # 메시지 확인
        print("4. 메시지 정의 확인 중...")
        msg_check_cmd = f"{env_cmd} && rosmsg show balance_robot_nodes/MotorStatus | grep -E '(raw_hall_data|motor.*hall)'"
        success, output, error = run_command(client, msg_check_cmd, timeout=5, show_output=True)
        print()
        
        print("="*80)
        print("✅ 업로드 및 빌드 완료!")
        print("="*80)
        print()
        print("이제 다음 명령어로 확인할 수 있습니다:")
        print()
        print(f"  export ROS_MASTER_URI='http://{current_ip}:11311'")
        print(f"  export ROS_IP='{current_ip}'")
        print("  source ~/catkin_ws/devel/setup.bash")
        print("  rostopic echo /motor_status | grep raw_hall_data")
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


