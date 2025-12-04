#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
ROS 실행 상태 확인
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

def main():
    """메인 함수"""
    print("=== ROS Status Check ===\n")
    
    client = create_ssh_client()
    if not client:
        return 1
    
    try:
        # 현재 IP 확인
        print("1. Current IP address:")
        success, output, error = run_command(client, "hostname -I | awk '{print $1}'", timeout=3)
        if success:
            current_ip = output.strip()
            print(f"   {current_ip}\n")
        
        # ROS 환경 변수 확인
        print("2. ROS Environment Variables:")
        setup_cmd = "cd ~/catkin_ws && source devel/setup.bash"
        
        # bashrc 로드 후 환경 변수 확인
        check_env = f"bash -c 'source ~/.bashrc 2>/dev/null; {setup_cmd} 2>/dev/null; echo ROS_MASTER_URI=$ROS_MASTER_URI; echo ROS_IP=$ROS_IP; echo ROS_HOSTNAME=$ROS_HOSTNAME'"
        success, output, error = run_command(client, check_env, timeout=5)
        
        if success and output:
            for line in output.strip().split('\n'):
                if '=' in line:
                    print(f"   {line}")
        else:
            print("   ⚠️  Could not read environment variables")
        
        # roscore 실행 확인
        print("\n3. roscore Status:")
        success, output, error = run_command(client, "ps aux | grep roscore | grep -v grep", timeout=3)
        if success and "roscore" in output:
            print("   ✅ roscore is running")
            # PID 추출
            if output:
                pid = output.split()[1]
                print(f"   PID: {pid}")
        else:
            print("   ❌ roscore is NOT running")
        
        # ROS 노드 확인
        print("\n4. ROS Nodes:")
        success, output, error = run_command(client, f"export ROS_MASTER_URI='http://{current_ip}:11311' && export ROS_IP='{current_ip}' && {setup_cmd} && timeout 3 rosnode list 2>/dev/null", timeout=5)
        if success and output:
            nodes = [line.strip() for line in output.strip().split('\n') if line.strip()]
            if nodes:
                print(f"   ✅ Found {len(nodes)} nodes:")
                for node in nodes[:10]:  # 최대 10개만 표시
                    print(f"     - {node}")
            else:
                print("   ⚠️  No nodes found")
        else:
            print("   ⚠️  Could not list nodes (roscore may not be running)")
        
        # ROS 토픽 확인
        print("\n5. ROS Topics:")
        success, output, error = run_command(client, f"export ROS_MASTER_URI='http://{current_ip}:11311' && export ROS_IP='{current_ip}' && {setup_cmd} && timeout 3 rostopic list 2>/dev/null", timeout=5)
        if success and output:
            topics = [line.strip() for line in output.strip().split('\n') if line.strip()]
            if topics:
                print(f"   ✅ Found {len(topics)} topics:")
                for topic in topics[:15]:  # 최대 15개만 표시
                    print(f"     - {topic}")
            else:
                print("   ⚠️  No topics found")
        else:
            print("   ⚠️  Could not list topics")
        
        # ROS 서비스 확인
        print("\n6. ROS Services:")
        success, output, error = run_command(client, f"export ROS_MASTER_URI='http://{current_ip}:11311' && export ROS_IP='{current_ip}' && {setup_cmd} && timeout 3 rosservice list 2>/dev/null", timeout=5)
        if success and output:
            services = [line.strip() for line in output.strip().split('\n') if line.strip()]
            if services:
                print(f"   ✅ Found {len(services)} services")
                for service in services[:10]:  # 최대 10개만 표시
                    print(f"     - {service}")
            else:
                print("   ⚠️  No services found")
        else:
            print("   ⚠️  Could not list services")
        
        # ROS 파라미터 확인
        print("\n7. ROS Parameters:")
        success, output, error = run_command(client, f"export ROS_MASTER_URI='http://{current_ip}:11311' && export ROS_IP='{current_ip}' && {setup_cmd} && timeout 3 rosparam list 2>/dev/null", timeout=5)
        if success and output:
            params = [line.strip() for line in output.strip().split('\n') if line.strip()]
            if params:
                print(f"   ✅ Found {len(params)} parameters")
                for param in params[:10]:  # 최대 10개만 표시
                    print(f"     - {param}")
            else:
                print("   ⚠️  No parameters found")
        else:
            print("   ⚠️  Could not list parameters")
        
        # 통합 상태
        print("\n" + "="*50)
        print("ROS Status Summary")
        print("="*50)
        
        # roscore 실행 여부
        success, output, error = run_command(client, "ps aux | grep roscore | grep -v grep", timeout=3)
        roscore_running = success and "roscore" in output
        
        # 토픽 존재 여부
        success, output, error = run_command(client, f"export ROS_MASTER_URI='http://{current_ip}:11311' && export ROS_IP='{current_ip}' && {setup_cmd} && timeout 2 rostopic list 2>/dev/null | wc -l", timeout=5)
        topic_count = 0
        if success and output.strip().isdigit():
            topic_count = int(output.strip())
        
        if roscore_running and topic_count > 0:
            print("✅ ROS is running correctly!")
            print(f"   - roscore: Running")
            print(f"   - Topics: {topic_count} available")
        elif roscore_running:
            print("⚠️  roscore is running but no topics found")
        else:
            print("❌ ROS is NOT running properly")
            print("   - roscore: Not running")
            print("   - Please start roscore first")
        
        return 0
        
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



