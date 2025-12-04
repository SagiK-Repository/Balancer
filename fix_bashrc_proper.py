#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
.bashrc 파일을 올바르게 수정 (중복 제거 및 정리)
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
    print("=== Fixing ~/.bashrc (Proper) ===\n")
    
    client = create_ssh_client()
    if not client:
        return 1
    
    try:
        # 현재 bashrc 읽기
        print("1. Reading current ~/.bashrc...")
        success, output, error = run_command(client, "cat ~/.bashrc", timeout=5)
        
        if not success:
            print(f"   ❌ Failed: {error}")
            return 1
        
        bashrc_content = output
        lines = bashrc_content.split('\n')
        
        # ROS 관련 설정 찾기
        print("\n2. Finding ROS IP settings...")
        ros_section_start = -1
        ros_section_end = -1
        
        for i, line in enumerate(lines):
            if 'ROS_MASTER_URI' in line or 'ROS_IP' in line or 'ROS_HOSTNAME' in line or 'DISPLAY' in line:
                if ros_section_start == -1:
                    ros_section_start = i
                ros_section_end = i
        
        if ros_section_start == -1:
            print("   ⚠️  No ROS settings found, adding new section...")
            # 파일 끝에 추가
            new_section = '''
# ROS IP Auto-configuration (auto-detected)
export DISPLAY="$(hostname -I | awk '{print $1}'):0"
export ROS_MASTER_URI="http://$(hostname -I | awk '{print $1}'):11311"
export ROS_IP="$(hostname -I | awk '{print $1}')"
export ROS_HOSTNAME="$(hostname -I | awk '{print $1}')"
export TURTLEBOT3_IP=$(hostname -I | awk '{print $1}')
'''
            new_content = bashrc_content + new_section
        else:
            print(f"   Found ROS settings at lines {ros_section_start+1}-{ros_section_end+1}")
            
            # 기존 ROS 설정 제거하고 새로 추가
            new_lines = lines[:ros_section_start]
            
            # 중복 제거: 이미 자동 설정이 있는지 확인
            has_auto = False
            for line in lines[ros_section_start:ros_section_end+1]:
                if '$(hostname -I | awk' in line and 'export' in line:
                    has_auto = True
                    break
            
            if not has_auto:
                # 자동 설정 추가
                new_lines.append('')
                new_lines.append('# ROS IP Auto-configuration (auto-detected)')
                new_lines.append('export DISPLAY="$(hostname -I | awk \'{print $1}\'):0"')
                new_lines.append('export ROS_MASTER_URI="http://$(hostname -I | awk \'{print $1}\'):11311"')
                new_lines.append('export ROS_IP="$(hostname -I | awk \'{print $1}\')"')
                new_lines.append('export ROS_HOSTNAME="$(hostname -I | awk \'{print $1}\')"')
                new_lines.append('export TURTLEBOT3_IP=$(hostname -I | awk \'{print $1}\')')
            else:
                # 기존 자동 설정 유지
                for line in lines[ros_section_start:ros_section_end+1]:
                    if '$(hostname -I | awk' in line or ('export' in line and ('ROS' in line or 'DISPLAY' in line)):
                        new_lines.append(line)
            
            # 나머지 라인 추가
            new_lines.extend(lines[ros_section_end+1:])
            new_content = '\n'.join(new_lines)
        
        # 백업
        print("\n3. Creating backup...")
        run_command(client, "cp ~/.bashrc ~/.bashrc.backup.$(date +%Y%m%d_%H%M%S)", timeout=5)
        
        # 새 내용 작성
        print("4. Writing new ~/.bashrc...")
        write_cmd = f"cat > /tmp/bashrc_fixed << 'ENDOFFILE'\n{new_content}\nENDOFFILE"
        success, output, error = run_command(client, write_cmd, timeout=5)
        
        if not success:
            print(f"   ❌ Failed: {error}")
            return 1
        
        # 파일 복사
        success, output, error = run_command(client, "cp /tmp/bashrc_fixed ~/.bashrc", timeout=5)
        
        if not success:
            print(f"   ❌ Failed: {error}")
            return 1
        
        print("   ✅ ~/.bashrc updated")
        
        # 확인
        print("\n5. Verifying changes...")
        success, output, error = run_command(client, "grep -E 'ROS_MASTER_URI|ROS_IP|ROS_HOSTNAME|DISPLAY' ~/.bashrc | grep -v '^#' | tail -5", timeout=5)
        if success and output:
            print("   ROS settings:")
            for line in output.strip().split('\n'):
                if line.strip():
                    print(f"     {line.strip()}")
        
        print("\n" + "="*50)
        print("✅ Fix completed!")
        print("="*50)
        print("\nNote: Restart terminal or run 'source ~/.bashrc' to apply changes")
        
        return 0
        
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



