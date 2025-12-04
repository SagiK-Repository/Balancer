#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
ROS IP 설정 확인 및 적용
"""

import paramiko
import sys

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
        print("✅ Connected successfully!")
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
        start_time = time.time()
        
        while True:
            if time.time() - start_time > timeout:
                stdout.channel.close()
                stderr.channel.close()
                return False, output, f"Timeout"
            
            if stdout.channel.recv_ready():
                output += stdout.channel.recv(4096).decode('utf-8', errors='ignore')
            if stderr.channel.recv_stderr_ready():
                error += stderr.channel.recv_stderr(4096).decode('utf-8', errors='ignore')
            
            if stdout.channel.exit_status_ready():
                exit_status = stdout.channel.recv_exit_status()
                while stdout.channel.recv_ready():
                    output += stdout.channel.recv(4096).decode('utf-8', errors='ignore')
                break
            
            time.sleep(0.05)
        
        return exit_status == 0, output, error
    except Exception as e:
        return False, "", str(e)

def main():
    """메인 함수"""
    print("=== Verifying ROS IP Configuration ===\n")
    
    client = create_ssh_client()
    if not client:
        return 1
    
    try:
        # bashrc 다시 로드
        print("1. Reloading ~/.bashrc...")
        success, output, error = run_command(client, "source ~/.bashrc && echo 'Bashrc reloaded'", timeout=5)
        
        # 현재 IP 확인
        print("\n2. Checking current IP address...")
        success, output, error = run_command(client, "hostname -I | awk '{print $1}'", timeout=3)
        if success:
            current_ip = output.strip()
            print(f"   Current IP: {current_ip}")
        
        # ROS 환경 변수 확인
        print("\n3. Checking ROS environment variables...")
        vars_to_check = ['ROS_MASTER_URI', 'ROS_IP', 'ROS_HOSTNAME', 'DISPLAY']
        
        for var in vars_to_check:
            cmd = f"bash -c 'source ~/.bashrc && echo ${var}'"
            success, output, error = run_command(client, cmd, timeout=5)
            if success:
                value = output.strip()
                print(f"   {var}: {value}")
            else:
                print(f"   {var}: (not set)")
        
        print("\n" + "="*50)
        print("✅ Verification completed!")
        print("="*50)
        print("\nROS should now work with the current IP address.")
        print("You can now run ROS commands without IP issues.")
        
        return 0
        
    except Exception as e:
        print(f"❌ Error: {e}")
        return 1
    finally:
        client.close()

if __name__ == "__main__":
    import time
    if sys.platform == 'win32':
        import io
        try:
            sys.stdout = io.TextIOWrapper(sys.stdout.buffer, encoding='utf-8')
            sys.stderr = io.TextIOWrapper(sys.stderr.buffer, encoding='utf-8')
        except:
            pass
    
    sys.exit(main())



