#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
ROS 시작 및 상태 확인
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
    print("=== Starting ROS ===\n")
    
    client = create_ssh_client()
    if not client:
        return 1
    
    try:
        # 현재 IP 확인
        print("1. Getting current IP address...")
        success, output, error = run_command(client, "hostname -I | awk '{print $1}'", timeout=3)
        if success:
            current_ip = output.strip()
            print(f"   Current IP: {current_ip}\n")
        else:
            print("   ⚠️  Could not get IP, using default")
            current_ip = "192.168.0.28"
        
        setup_cmd = "cd ~/catkin_ws && source devel/setup.bash"
        
        # 기존 roscore 종료
        print("2. Cleaning up existing roscore...")
        run_command(client, "pkill -f roscore; sleep 1", timeout=3)
        
        # roscore 시작
        print("\n3. Starting roscore...")
        roscore_cmd = f"export ROS_MASTER_URI='http://{current_ip}:11311' && export ROS_IP='{current_ip}' && export ROS_HOSTNAME='{current_ip}' && {setup_cmd} && nohup roscore > /tmp/roscore.log 2>&1 &"
        client.exec_command(roscore_cmd, timeout=2)
        time.sleep(3)
        
        # roscore 확인
        success, output, error = run_command(client, "ps aux | grep roscore | grep -v grep", timeout=3)
        if success and "roscore" in output:
            print("   ✅ roscore started")
            pid = output.split()[1]
            print(f"   PID: {pid}")
        else:
            print("   ❌ Failed to start roscore")
            print("   Check log: tail -20 /tmp/roscore.log")
            success, output, error = run_command(client, "tail -20 /tmp/roscore.log 2>/dev/null", timeout=3)
            if output:
                print("   Log:")
                for line in output.strip().split('\n'):
                    if line.strip():
                        print(f"     {line.strip()}")
            return 1
        
        # ROS 연결 테스트
        print("\n4. Testing ROS connection...")
        time.sleep(2)
        
        test_cmd = f"export ROS_MASTER_URI='http://{current_ip}:11311' && export ROS_IP='{current_ip}' && {setup_cmd} && timeout 3 rostopic list 2>&1"
        success, output, error = run_command(client, test_cmd, timeout=5)
        
        if success and output:
            topics = [line.strip() for line in output.strip().split('\n') if line.strip() and not line.startswith('/')]
            if topics:
                print(f"   ✅ ROS is working! Found {len(topics)} topics")
                print("   Topics:")
                for topic in topics[:10]:
                    print(f"     - {topic}")
            else:
                print("   ⚠️  roscore is running but no topics found (this is normal for fresh roscore)")
        else:
            if "Connection refused" in error or "Connection refused" in output:
                print("   ❌ Connection refused - roscore may not be listening on correct IP")
            else:
                print(f"   ⚠️  Could not connect: {error}")
        
        # 노드 확인
        print("\n5. Checking ROS nodes...")
        test_cmd = f"export ROS_MASTER_URI='http://{current_ip}:11311' && export ROS_IP='{current_ip}' && {setup_cmd} && timeout 3 rosnode list 2>&1"
        success, output, error = run_command(client, test_cmd, timeout=5)
        
        if success and output:
            nodes = [line.strip() for line in output.strip().split('\n') if line.strip()]
            if nodes:
                print(f"   ✅ Found {len(nodes)} nodes:")
                for node in nodes:
                    print(f"     - {node}")
            else:
                print("   ℹ️  No nodes running (this is normal if no nodes are started)")
        
        # 최종 상태
        print("\n" + "="*50)
        print("ROS Status")
        print("="*50)
        print(f"✅ roscore: Running on {current_ip}:11311")
        print(f"✅ ROS_MASTER_URI: http://{current_ip}:11311")
        print(f"✅ ROS_IP: {current_ip}")
        print("\nROS is ready to use!")
        print("\nYou can now run ROS nodes and commands.")
        
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



