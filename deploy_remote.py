#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
원격 서버에 프로젝트 배포 및 빌드 스크립트
paramiko를 사용하여 SSH 연결 및 파일 전송
"""

import os
import sys
import paramiko
from pathlib import Path
import stat

# Windows에서 UTF-8 출력 설정
if sys.platform == 'win32':
    import io
    sys.stdout = io.TextIOWrapper(sys.stdout.buffer, encoding='utf-8')
    sys.stderr = io.TextIOWrapper(sys.stderr.buffer, encoding='utf-8')

# 원격 서버 정보
HOSTNAME = "192.168.0.28"
USERNAME = "tbot3"
PASSWORD = "1234"
REMOTE_BASE = "~/catkin_ws/src/integrated_balancer"
PROJECT_DIR = Path("src/integrated_balancer")

def get_home_directory(client):
    """원격 서버의 홈 디렉토리 경로 가져오기"""
    try:
        stdin, stdout, stderr = client.exec_command("echo $HOME")
        home_dir = stdout.read().decode().strip()
        return home_dir
    except:
        return f"/home/{USERNAME}"

def create_ssh_client():
    """SSH 클라이언트 생성 및 연결"""
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

def upload_file(sftp, local_path, remote_path):
    """파일을 원격 서버에 업로드"""
    try:
        # 원격 디렉토리 생성
        remote_dir = os.path.dirname(remote_path)
        
        # 디렉토리 생성 (절대 경로로)
        dirs = [d for d in remote_dir.split("/") if d]
        current_path = ""
        for d in dirs:
            current_path = f"{current_path}/{d}" if current_path else d
            try:
                sftp.mkdir(current_path)
            except:
                pass  # 디렉토리가 이미 존재할 수 있음
        
        # 파일 업로드
        sftp.put(local_path, remote_path)
        print(f"  ✅ {Path(local_path).name} -> {remote_path}")
        return True
    except Exception as e:
        print(f"  ❌ Failed to upload {Path(local_path).name}: {e}")
        return False

def create_remote_dirs(client, remote_base):
    """원격 디렉토리 생성"""
    commands = [
        f"mkdir -p ~/catkin_ws/src",
        f"mkdir -p {remote_base}",
        f"mkdir -p {remote_base}/include/integrated_balancer",
        f"mkdir -p {remote_base}/src",
        f"mkdir -p {remote_base}/test"
    ]
    
    for cmd in commands:
        try:
            stdin, stdout, stderr = client.exec_command(cmd)
            stdout.channel.recv_exit_status()
        except:
            pass

def upload_directory(sftp, local_dir, remote_base):
    """디렉토리를 재귀적으로 업로드"""
    local_path = Path(local_dir)
    
    if not local_path.exists():
        print(f"❌ Local directory not found: {local_dir}")
        return False
    
    uploaded = 0
    failed = 0
    
    # 모든 파일 찾기
    for root, dirs, files in os.walk(local_path):
        for file in files:
            local_file = Path(root) / file
            relative_path = local_file.relative_to(local_path)
            # Windows 경로를 Unix 경로로 변환
            remote_file = f"{remote_base}/{relative_path.as_posix().replace(chr(92), '/')}"
            
            if upload_file(sftp, str(local_file), remote_file):
                uploaded += 1
            else:
                failed += 1
    
    print(f"\nUpload summary: {uploaded} files uploaded, {failed} failed")
    return failed == 0

def build_remote(client):
    """원격 서버에서 빌드 실행"""
    print("\n=== Building on remote server ===")
    
    commands = [
        "cd ~/catkin_ws",
        "source /opt/ros/noetic/setup.bash 2>/dev/null || source /opt/ros/melodic/setup.bash 2>/dev/null || true",
        "catkin_make"
    ]
    
    command = " && ".join(commands)
    
    try:
        stdin, stdout, stderr = client.exec_command(command)
        
        # 출력 실시간 표시
        print("Build output:")
        print("-" * 50)
        for line in stdout:
            print(line.rstrip())
        
        exit_status = stdout.channel.recv_exit_status()
        
        if exit_status == 0:
            print("\n✅ Build completed successfully!")
            return True
        else:
            print("\n❌ Build failed!")
            print("Error output:")
            for line in stderr:
                print(line.rstrip())
            return False
            
    except Exception as e:
        print(f"❌ Build execution failed: {e}")
        return False

def main():
    """메인 함수"""
    print("=== Remote Deployment Script ===")
    print(f"Project: {PROJECT_DIR}")
    print(f"Remote: {USERNAME}@{HOSTNAME}:{REMOTE_BASE}")
    print()
    
    # 프로젝트 디렉토리 확인
    if not PROJECT_DIR.exists():
        print(f"❌ Project directory not found: {PROJECT_DIR}")
        return 1
    
    # SSH 연결
    client = create_ssh_client()
    if not client:
        return 1
    
    try:
        # 홈 디렉토리 확인
        home_dir = get_home_directory(client)
        remote_base_abs = REMOTE_BASE.replace("~", home_dir)
        print(f"Remote base path: {remote_base_abs}")
        
        # 원격 디렉토리 생성 (SSH 명령 사용)
        print(f"\n=== Creating remote directories ===")
        create_remote_dirs(client, remote_base_abs)
        
        # SFTP 클라이언트 생성
        sftp = client.open_sftp()
        
        # 파일 업로드
        print(f"\n=== Uploading files ===")
        if not upload_directory(sftp, PROJECT_DIR, remote_base_abs):
            print("❌ File upload failed")
            return 1
        
        sftp.close()
        
        # 빌드 실행
        if not build_remote(client):
            return 1
        
        print("\n✅ Deployment and build completed successfully!")
        return 0
        
    except Exception as e:
        print(f"❌ Error: {e}")
        return 1
    finally:
        client.close()

if __name__ == "__main__":
    # paramiko가 설치되어 있는지 확인
    try:
        import paramiko
    except ImportError:
        print("❌ paramiko not installed. Installing...")
        os.system("pip install paramiko")
        import paramiko
    
    sys.exit(main())

