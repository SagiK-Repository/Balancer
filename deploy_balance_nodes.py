#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
balance_robot_nodes 패키지를 원격 서버에 배포
"""

import paramiko
import sys
import os
from pathlib import Path

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

def get_home_directory(client):
    """원격 서버의 홈 디렉토리 경로 가져오기"""
    try:
        stdin, stdout, stderr = client.exec_command("echo $HOME")
        home_dir = stdout.read().decode().strip()
        return home_dir
    except:
        return f"/home/{USERNAME}"

def upload_directory(sftp, client, local_dir, remote_base, home_dir):
    """디렉토리를 재귀적으로 업로드"""
    local_path = Path(local_dir)
    
    if not local_path.exists():
        print(f"❌ Local directory not found: {local_dir}")
        return False
    
    uploaded = 0
    failed = 0
    
    for root, dirs, files in os.walk(local_path):
        for file in files:
            local_file = Path(root) / file
            relative_path = local_file.relative_to(local_path)
            # Windows 경로를 Unix 경로로 변환
            remote_file = f"{remote_base}/{str(relative_path).replace(chr(92), '/')}"
            
            try:
                # 원격 디렉토리 생성 (SSH 명령 사용)
                remote_dir = os.path.dirname(remote_file)
                
                # SSH로 디렉토리 생성
                mkdir_cmd = f"mkdir -p '{remote_dir}'"
                stdin, stdout, stderr = client.exec_command(mkdir_cmd, timeout=3)
                stdout.channel.recv_exit_status()
                
                # 파일 업로드
                sftp.put(str(local_file), remote_file)
                print(f"  ✅ {Path(local_file).name}")
                uploaded += 1
            except Exception as e:
                print(f"  ❌ {Path(local_file).name}: {e}")
                failed += 1
    
    print(f"\nUpload summary: {uploaded} files uploaded, {failed} failed")
    return failed == 0

def main():
    """메인 함수"""
    print("=== Deploy balance_robot_nodes ===")
    
    project_dir = Path("src/balance_robot_nodes")
    
    if not project_dir.exists():
        print(f"❌ balance_robot_nodes not found at: {project_dir}")
        print("   Make sure you're in the project root directory")
        return 1
    
    client = create_ssh_client()
    if not client:
        return 1
    
    try:
        home_dir = get_home_directory(client)
        remote_base = f"{home_dir}/catkin_ws/src/balance_robot_nodes"
        
        print(f"\nUploading to: {remote_base}")
        
        # 원격 디렉토리 생성
        commands = [
            f"mkdir -p ~/catkin_ws/src",
            f"mkdir -p {remote_base}"
        ]
        
        for cmd in commands:
            client.exec_command(cmd, timeout=5)
        
        # SFTP로 파일 업로드
        sftp = client.open_sftp()
        
        print("\nUploading files...")
        if upload_directory(sftp, client, project_dir, remote_base, home_dir):
            sftp.close()
            
            # 빌드
            print("\nBuilding package...")
            build_cmd = "cd ~/catkin_ws && source /opt/ros/noetic/setup.bash 2>/dev/null || source /opt/ros/melodic/setup.bash 2>/dev/null && catkin_make --only-pkg-with-deps balance_robot_nodes"
            
            stdin, stdout, stderr = client.exec_command(build_cmd, timeout=120)
            
            print("Build output:")
            for line in stdout:
                print(line.rstrip())
            
            exit_status = stdout.channel.recv_exit_status()
            
            if exit_status == 0:
                print("\n✅ Deployment and build completed successfully!")
                return 0
            else:
                print("\n❌ Build failed")
                print("Error:")
                for line in stderr:
                    print(line.rstrip())
                return 1
        else:
            sftp.close()
            return 1
        
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

