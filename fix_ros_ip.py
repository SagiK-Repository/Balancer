#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
ROS IP 주소 자동 설정 수정
~/.bashrc 파일을 수정하여 IP 주소를 자동으로 받도록 변경
"""

import paramiko
import sys
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
        print("✅ Connected successfully!")
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

def fix_bashrc(client):
    """~/.bashrc 파일 수정"""
    print("\n=== Fixing ~/.bashrc ===")
    
    # 1. 현재 .bashrc 백업
    print("\n1. Backing up ~/.bashrc...")
    success, output, error = run_command(client, "cp ~/.bashrc ~/.bashrc.backup.$(date +%Y%m%d_%H%M%S)", timeout=5)
    if success:
        print("   ✅ Backup created")
    else:
        print("   ⚠️  Backup failed, but continuing...")
    
    # 2. 현재 .bashrc 읽기
    print("\n2. Reading current ~/.bashrc...")
    success, output, error = run_command(client, "cat ~/.bashrc", timeout=5)
    
    if not success:
        print(f"   ❌ Failed to read .bashrc: {error}")
        return False
    
    bashrc_content = output
    
    # 3. IP 주소 관련 설정 찾기 및 수정
    print("\n3. Modifying ROS IP settings...")
    
    # 수정할 패턴들
    replacements = [
        # DISPLAY 설정
        (r'export DISPLAY="192\.168\.\d+\.\d+:0"', 
         'export DISPLAY="$(hostname -I | awk \'{print $1}\'):0"'),
        
        # ROS_MASTER_URI 설정
        (r'export ROS_MASTER_URI="http://192\.168\.\d+\.\d+:11311"', 
         'export ROS_MASTER_URI="http://$(hostname -I | awk \'{print $1}\'):11311"'),
        
        # ROS_IP 설정
        (r'export ROS_IP="192\.168\.\d+\.\d+"', 
         'export ROS_IP="$(hostname -I | awk \'{print $1}\')"'),
        
        # ROS_HOSTNAME 설정
        (r'export ROS_HOSTNAME="192\.168\.\d+\.\d+"', 
         'export ROS_HOSTNAME="$(hostname -I | awk \'{print $1}\')"'),
        
        # TURTLEBOT3_IP 설정 (두 번째 IP를 사용하는 경우)
        (r'export TURTLEBOT3_IP=\$\(hostname -I \| awk \'\{print \$2\}\'\)', 
         'export TURTLEBOT3_IP=$(hostname -I | awk \'{print $1}\')'),
    ]
    
    modified = False
    new_content = bashrc_content
    
    for pattern, replacement in replacements:
        if re.search(pattern, new_content):
            new_content = re.sub(pattern, replacement, new_content)
            modified = True
            print(f"   ✅ Updated: {pattern}")
    
    # 주석 처리된 자동 설정이 있다면 활성화
    if '#export ROS_MASTER_URI="http://$(hostname -I | awk' in new_content:
        new_content = re.sub(
            r'#export ROS_MASTER_URI="http://\$\(hostname -I \| awk',
            'export ROS_MASTER_URI="http://$(hostname -I | awk',
            new_content
        )
        modified = True
        print("   ✅ Enabled commented ROS_MASTER_URI")
    
    if '#export ROS_IP="$(hostname -I | awk' in new_content:
        new_content = re.sub(
            r'#export ROS_IP="\$\(hostname -I \| awk',
            'export ROS_IP="$(hostname -I | awk',
            new_content
        )
        modified = True
        print("   ✅ Enabled commented ROS_IP")
    
    if not modified:
        print("   ℹ️  No changes needed (already using auto IP)")
        # 현재 IP 확인
        success, output, error = run_command(client, "hostname -I | awk '{print $1}'", timeout=3)
        if success:
            current_ip = output.strip()
            print(f"   Current IP: {current_ip}")
        return True
    
    # 4. 수정된 내용을 파일에 쓰기
    print("\n4. Writing modified ~/.bashrc...")
    
    # 임시 파일로 작성
    write_cmd = f"cat > /tmp/bashrc_new << 'ENDOFFILE'\n{new_content}\nENDOFFILE"
    success, output, error = run_command(client, write_cmd, timeout=5)
    
    if not success:
        print(f"   ❌ Failed to write temp file: {error}")
        return False
    
    # 원본 파일로 복사
    success, output, error = run_command(client, "cp /tmp/bashrc_new ~/.bashrc", timeout=5)
    
    if success:
        print("   ✅ ~/.bashrc updated successfully")
    else:
        print(f"   ❌ Failed to update .bashrc: {error}")
        return False
    
    # 5. 현재 IP 확인
    print("\n5. Checking current IP address...")
    success, output, error = run_command(client, "hostname -I", timeout=3)
    if success:
        ips = output.strip().split()
        print(f"   Available IPs: {', '.join(ips)}")
        if ips:
            print(f"   Will use first IP: {ips[0]}")
    
    # 6. 수정된 설정 확인
    print("\n6. Verifying changes...")
    success, output, error = run_command(client, "grep -E 'ROS_MASTER_URI|ROS_IP|ROS_HOSTNAME|DISPLAY' ~/.bashrc | grep -v '^#'", timeout=5)
    if success and output:
        print("   Current ROS settings:")
        for line in output.strip().split('\n'):
            if line.strip():
                print(f"     {line.strip()}")
    
    return True

def main():
    """메인 함수"""
    print("=== ROS IP Address Auto-Configuration Fix ===")
    print("This will modify ~/.bashrc to use automatic IP detection\n")
    
    client = create_ssh_client()
    if not client:
        return 1
    
    try:
        if fix_bashrc(client):
            print("\n" + "="*50)
            print("✅ Fix completed successfully!")
            print("="*50)
            print("\nNext steps:")
            print("1. Reload bashrc: source ~/.bashrc")
            print("2. Or restart terminal")
            print("3. Verify: echo $ROS_MASTER_URI")
            print("4. Verify: echo $ROS_IP")
            return 0
        else:
            print("\n❌ Fix failed")
            return 1
        
    except Exception as e:
        print(f"❌ Error: {e}")
        import traceback
        traceback.print_exc()
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



