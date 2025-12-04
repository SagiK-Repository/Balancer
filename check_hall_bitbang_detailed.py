#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
BitBangSPI로 실행 중일 때 홀 센서 입력 상세 확인
토픽 목록 확인 및 실제 데이터 읽기
"""

import paramiko
import sys
import time
import signal

HOSTNAME = "192.168.0.28"
USERNAME = "tbot3"
PASSWORD = "1234"

ssh_client = None

def signal_handler(sig, frame):
    """Ctrl+C 핸들러"""
    print("\n\nInterrupted by user.")
    sys.exit(0)

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
    except KeyboardInterrupt:
        raise
    except Exception as e:
        return False, "", str(e)

def main():
    """메인 함수"""
    global ssh_client
    
    signal.signal(signal.SIGINT, signal_handler)
    if sys.platform != 'win32':
        signal.signal(signal.SIGTERM, signal_handler)
    
    print("=== Hall Sensor Check with BitBangSPI (Detailed) ===\n")
    
    client = create_ssh_client()
    if not client:
        return 1
    
    ssh_client = client
    
    try:
        # 현재 IP 확인
        success, output, error = run_command(client, "hostname -I | awk '{print $1}'", timeout=3)
        if success:
            current_ip = output.strip()
        else:
            current_ip = "192.168.0.28"
        
        setup_cmd = "cd ~/catkin_ws && source devel/setup.bash"
        env_cmd = f"export ROS_MASTER_URI='http://{current_ip}:11311' && export ROS_IP='{current_ip}' && {setup_cmd}"
        
        # 1. roscore 확인
        print("1. Checking roscore...")
        success, output, error = run_command(client, "ps aux | grep roscore | grep -v grep", timeout=3)
        if not success or "roscore" not in output:
            print("   ⚠️  roscore not running, starting...")
            roscore_cmd = f"{env_cmd} && nohup roscore > /tmp/roscore.log 2>&1 &"
            client.exec_command(roscore_cmd, timeout=2)
            time.sleep(3)
            print("   ✅ roscore started")
        else:
            print("   ✅ roscore is running")
        
        # 2. SPI Hardware Node 확인 및 시작
        print("\n2. Checking SPI Hardware Node (BitBangSPI)...")
        success, output, error = run_command(client, "ps aux | grep spi_hardware_node | grep -v grep", timeout=3)
        
        if not success or "spi_hardware_node" not in output:
            print("   ⚠️  SPI Hardware Node not running, starting...")
            node_cmd = f"{env_cmd} && rosrun balance_robot_nodes spi_hardware_node > /tmp/spi_node.log 2>&1 &"
            client.exec_command(node_cmd, timeout=2)
            time.sleep(3)
            print("   ✅ SPI Hardware Node started")
        else:
            print("   ✅ SPI Hardware Node is running")
        
        # 노드 로그 확인
        print("\n   Checking node log...")
        success, output, error = run_command(client, "tail -20 /tmp/spi_node.log 2>/dev/null", timeout=3)
        if output:
            print("   Log (last 20 lines):")
            for line in output.strip().split('\n')[-10:]:
                if line.strip():
                    print(f"     {line.strip()}")
        
        # 3. 토픽 목록 확인
        print("\n3. Checking available topics...")
        success, output, error = run_command(client, f"{env_cmd} && timeout 3 rostopic list 2>&1", timeout=5)
        
        if success and output:
            topics = [line.strip() for line in output.strip().split('\n') if line.strip() and not line.startswith('/')]
            print(f"   Found {len(topics)} topics:")
            for topic in topics:
                print(f"     - {topic}")
            
            # motor 관련 토픽 찾기
            motor_topics = [t for t in topics if 'motor' in t.lower() or 'hall' in t.lower() or 'wheel' in t.lower()]
            if motor_topics:
                print(f"\n   Motor-related topics ({len(motor_topics)}):")
                for topic in motor_topics:
                    print(f"     - {topic}")
        else:
            print("   ⚠️  Could not list topics")
            if error:
                print(f"   Error: {error}")
        
        # 4. 각 토픽에서 데이터 읽기 시도
        print("\n4. Trying to read from motor-related topics...")
        
        if success and output:
            topics = [line.strip() for line in output.strip().split('\n') if line.strip() and not line.startswith('/')]
            motor_topics = [t for t in topics if 'motor' in t.lower() or 'hall' in t.lower() or 'wheel' in t.lower()]
            
            if not motor_topics:
                # 모든 토픽에서 시도
                print("   No motor topics found, trying all topics...")
                motor_topics = topics[:5]  # 처음 5개만
            
            for topic in motor_topics:
                print(f"\n   Reading from: {topic}")
                cmd = f"{env_cmd} && timeout 2 rostopic echo {topic} -n 1 2>&1 | head -30"
                success, output, error = run_command(client, cmd, timeout=4)
                
                if success and output:
                    lines = output.strip().split('\n')[:20]
                    if lines:
                        print("   Sample data:")
                        for line in lines:
                            if line.strip():
                                print(f"     {line.strip()}")
                    else:
                        print("   (No data)")
                else:
                    print(f"   (Could not read: {error if error else 'timeout'})")
        
        # 5. 노드 정보 확인
        print("\n5. Checking ROS nodes...")
        success, output, error = run_command(client, f"{env_cmd} && timeout 3 rosnode list 2>&1", timeout=5)
        
        if success and output:
            nodes = [line.strip() for line in output.strip().split('\n') if line.strip()]
            print(f"   Found {len(nodes)} nodes:")
            for node in nodes:
                print(f"     - {node}")
                
                # 노드 정보 상세 확인
                if 'spi' in node.lower() or 'hardware' in node.lower():
                    print(f"\n   Getting info for {node}:")
                    cmd = f"{env_cmd} && timeout 2 rosnode info {node} 2>&1"
                    success2, output2, error2 = run_command(client, cmd, timeout=4)
                    if success2 and output2:
                        for line in output2.strip().split('\n')[:15]:
                            if line.strip():
                                print(f"     {line.strip()}")
        
        # 6. 토픽 타입 확인
        print("\n6. Checking topic types...")
        success, output, error = run_command(client, f"{env_cmd} && timeout 3 rostopic list -t 2>&1", timeout=5)
        
        if success and output:
            print("   Topic types:")
            for line in output.strip().split('\n')[:20]:
                if line.strip() and 'motor' in line.lower() or 'hall' in line.lower():
                    print(f"     {line.strip()}")
        
        print("\n" + "="*60)
        print("Summary")
        print("="*60)
        print("\nIf no motor_status topic is found:")
        print("  - Check if spi_hardware_node is publishing correctly")
        print("  - Check node logs: tail -f /tmp/spi_node.log")
        print("  - Verify topic name in the node source code")
        
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



