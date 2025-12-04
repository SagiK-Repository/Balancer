#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
.bashrc 파일 최종 수정 - 중복 제거 및 정리
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
    print("=== Final Fix for ~/.bashrc ===\n")
    
    client = create_ssh_client()
    if not client:
        return 1
    
    try:
        # 현재 bashrc 읽기
        print("1. Reading ~/.bashrc...")
        success, output, error = run_command(client, "cat ~/.bashrc", timeout=5)
        
        if not success:
            print(f"   ❌ Failed: {error}")
            return 1
        
        lines = output.split('\n')
        new_lines = []
        ros_section_found = False
        skip_ros = False
        
        # ROS 설정 부분 찾아서 한 번만 추가
        for i, line in enumerate(lines):
            # ROS 관련 설정인지 확인
            is_ros_line = any(keyword in line for keyword in ['ROS_MASTER_URI', 'ROS_IP', 'ROS_HOSTNAME', 'DISPLAY', 'TURTLEBOT3_IP'])
            
            if is_ros_line and not line.strip().startswith('#'):
                # 자동 설정이 이미 있으면 스킵
                if '$(hostname -I | awk' in line:
                    if not ros_section_found:
                        new_lines.append(line)
                        ros_section_found = True
                    # 중복은 스킵
                    continue
                # 수동 설정이면 제거
                else:
                    continue
            elif is_ros_line and line.strip().startswith('#'):
                # 주석 처리된 자동 설정이면 활성화
                if '$(hostname -I | awk' in line:
                    if not ros_section_found:
                        new_lines.append(line.lstrip('#'))
                        ros_section_found = True
                    continue
                else:
                    # 주석은 유지
                    new_lines.append(line)
            else:
                new_lines.append(line)
        
        # ROS 설정이 없었다면 추가
        if not ros_section_found:
            print("   Adding ROS auto-configuration section...")
            # 적절한 위치 찾기 (TURTLEBOT3_MODEL 근처)
            insert_pos = len(new_lines)
            for i, line in enumerate(new_lines):
                if 'TURTLEBOT3_MODEL' in line:
                    insert_pos = i + 1
                    break
            
            ros_config = [
                '',
                '# ROS IP Auto-configuration (auto-detected)',
                'export DISPLAY="$(hostname -I | awk \'{print $1}\'):0"',
                'export ROS_MASTER_URI="http://$(hostname -I | awk \'{print $1}\'):11311"',
                'export ROS_IP="$(hostname -I | awk \'{print $1}\')"',
                'export ROS_HOSTNAME="$(hostname -I | awk \'{print $1}\')"',
                'export TURTLEBOT3_IP=$(hostname -I | awk \'{print $1}\')'
            ]
            
            new_lines[insert_pos:insert_pos] = ros_config
        
        new_content = '\n'.join(new_lines)
        
        # 백업
        print("\n2. Creating backup...")
        run_command(client, "cp ~/.bashrc ~/.bashrc.backup.$(date +%Y%m%d_%H%M%S)", timeout=5)
        
        # 새 내용 작성
        print("3. Writing fixed ~/.bashrc...")
        write_cmd = f"cat > /tmp/bashrc_final << 'ENDOFFILE'\n{new_content}\nENDOFFILE"
        success, output, error = run_command(client, write_cmd, timeout=5)
        
        if not success:
            print(f"   ❌ Failed: {error}")
            return 1
        
        # 파일 복사
        success, output, error = run_command(client, "cp /tmp/bashrc_final ~/.bashrc", timeout=5)
        
        if not success:
            print(f"   ❌ Failed: {error}")
            return 1
        
        print("   ✅ ~/.bashrc updated")
        
        # 최종 확인
        print("\n4. Final verification...")
        success, output, error = run_command(client, "grep -E '^export (ROS_MASTER_URI|ROS_IP|ROS_HOSTNAME|DISPLAY|TURTLEBOT3_IP)=' ~/.bashrc | grep -v '^#'", timeout=5)
        if success and output:
            print("   ROS settings (no duplicates):")
            for line in output.strip().split('\n'):
                if line.strip():
                    print(f"     {line.strip()}")
        
        # 현재 IP 확인
        print("\n5. Current IP address:")
        success, output, error = run_command(client, "hostname -I | awk '{print $1}'", timeout=3)
        if success:
            current_ip = output.strip()
            print(f"   {current_ip}")
            print(f"   ROS_MASTER_URI will be: http://{current_ip}:11311")
        
        print("\n" + "="*50)
        print("✅ Fix completed successfully!")
        print("="*50)
        print("\nThe .bashrc file has been updated to use automatic IP detection.")
        print("IP address changes will be automatically handled.")
        
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



