#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
수정된 BitBangSPI로 홀 센서 값 확인
"""

import paramiko
import sys
import time
import signal

HOSTNAME = "192.168.0.28"
USERNAME = "tbot3"
PASSWORD = "1234"

running = True

def signal_handler(sig, frame):
    """Ctrl+C 핸들러"""
    global running
    print("\n\nInterrupted by user.")
    running = False
    sys.exit(0)

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
    global running
    
    signal.signal(signal.SIGINT, signal_handler)
    if sys.platform != 'win32':
        signal.signal(signal.SIGTERM, signal_handler)
    
    print("="*60)
    print("수정된 BitBangSPI 홀 센서 테스트")
    print("="*60 + "\n")
    
    client = create_ssh_client()
    if not client:
        return 1
    
    try:
        # 현재 IP 확인
        success, output, error = run_command(client, "hostname -I | awk '{print $1}'", timeout=3)
        if success:
            current_ip = output.strip()
        else:
            current_ip = "192.168.0.28"
        
        setup_cmd = "cd ~/catkin_ws && source devel/setup.bash"
        env_cmd = f"export ROS_MASTER_URI='http://{current_ip}:11311' && export ROS_IP='{current_ip}' && {setup_cmd}"
        
        # 1. 기존 노드 종료
        print("1. Stopping existing nodes...")
        run_command(client, "pkill -f spi_hardware_node; sleep 1", timeout=3)
        print("   ✅ Stopped")
        
        # 2. 노드 재시작
        print("\n2. Starting SPI Hardware Node with fixed code...")
        node_cmd = f"{env_cmd} && rosrun balance_robot_nodes spi_hardware_node > /tmp/spi_node.log 2>&1 &"
        client.exec_command(node_cmd, timeout=2)
        time.sleep(3)
        print("   ✅ Started")
        
        # 3. 로그 확인
        print("\n3. Checking node log...")
        success, output, error = run_command(client, "tail -30 /tmp/spi_node.log 2>/dev/null", timeout=3)
        if success and output:
            lines = output.strip().split('\n')
            has_error = False
            has_warning = False
            
            for line in lines:
                if 'ERROR' in line or 'error' in line.lower():
                    print(f"   ❌ {line.strip()}")
                    has_error = True
                elif 'WARN' in line or 'warning' in line.lower():
                    print(f"   ⚠️  {line.strip()}")
                    has_warning = True
                elif 'CS GPIO' in line or 'latch' in line.lower() or 'hall' in line.lower():
                    print(f"   ℹ️  {line.strip()}")
            
            if not has_error and not has_warning:
                print("   ✅ No errors or warnings in recent log")
        
        # 4. 홀 센서 값 읽기
        print("\n4. Reading hall sensor values...")
        print("   (Try rotating wheels manually to see changes)")
        print()
        print("Time(s)  Motor0_Hall  Motor0_Count  Motor1_Hall  Motor1_Count  Changes")
        print("-" * 70)
        
        start_time = time.time()
        prev_data = {}
        sample_count = 0
        current_message = []
        
        def process_line(line):
            nonlocal prev_data, sample_count, current_message
            
            line = line.strip()
            if not line:
                return
            
            if line.startswith('---'):
                if current_message:
                    import re
                    message_text = '\n'.join(current_message)
                    
                    data = {}
                    match = re.search(r'motor0_hall_state:\s*(\d+)', message_text)
                    if match:
                        data['motor0_hall'] = int(match.group(1))
                    
                    match = re.search(r'motor0_hall_count:\s*(-?\d+)', message_text)
                    if match:
                        data['motor0_count'] = int(match.group(1))
                    
                    match = re.search(r'motor1_hall_state:\s*(\d+)', message_text)
                    if match:
                        data['motor1_hall'] = int(match.group(1))
                    
                    match = re.search(r'motor1_hall_count:\s*(-?\d+)', message_text)
                    if match:
                        data['motor1_count'] = int(match.group(1))
                    
                    if data:
                        sample_count += 1
                        elapsed = time.time() - start_time
                        
                        changes = []
                        if 'motor0_hall' in data and 'motor0_hall' in prev_data:
                            if data['motor0_hall'] != prev_data['motor0_hall']:
                                changes.append("M0_H")
                            if 'motor0_count' in data and 'motor0_count' in prev_data:
                                if data['motor0_count'] != prev_data['motor0_count']:
                                    changes.append("M0_C++")
                        
                        if 'motor1_hall' in data and 'motor1_hall' in prev_data:
                            if data['motor1_hall'] != prev_data['motor1_hall']:
                                changes.append("M1_H")
                            if 'motor1_count' in data and 'motor1_count' in prev_data:
                                if data['motor1_count'] != prev_data['motor1_count']:
                                    changes.append("M1_C++")
                        
                        change_str = ", ".join(changes) if changes else "-"
                        
                        m0_hall = data.get('motor0_hall', 'N/A')
                        m0_count = data.get('motor0_count', 'N/A')
                        m1_hall = data.get('motor1_hall', 'N/A')
                        m1_count = data.get('motor1_count', 'N/A')
                        
                        print(f"{elapsed:7.2f}    {str(m0_hall):11s}  {str(m0_count):12s}  {str(m1_hall):11s}  {str(m1_count):12s}  {change_str}")
                        
                        prev_data.update(data)
                
                current_message = []
            else:
                current_message.append(line)
        
        # rostopic echo 실행
        cmd = f"{env_cmd} && timeout 30 rostopic echo /motor_status"
        stdin, stdout, stderr = client.exec_command(cmd, timeout=35)
        
        start_time = time.time()
        while running and time.time() - start_time < 30:
            if stdout.channel.recv_ready():
                chunk = stdout.channel.recv(4096).decode('utf-8', errors='ignore')
                lines = chunk.split('\n')
                for line in lines:
                    process_line(line)
            time.sleep(0.1)
        
        print(f"\n\nRead {sample_count} samples")
        
        # 5. 최종 상태 확인
        print("\n5. Final Status")
        print("-" * 60)
        
        if sample_count > 0:
            if prev_data.get('motor0_hall', 0) != 0 or prev_data.get('motor1_hall', 0) != 0:
                print("   ✅ Hall sensor values are non-zero (hardware reading may be working)")
            else:
                print("   ⚠️  All hall sensor values are 0 (may still be using dummy data)")
            
            if any('M0_H' in str(prev_data) or 'M1_H' in str(prev_data) or 'M0_C++' in str(prev_data) or 'M1_C++' in str(prev_data)):
                print("   ✅ Hall sensor values changed during test (hardware is working!)")
            else:
                print("   ⚠️  Hall sensor values did not change (try rotating wheels)")
        else:
            print("   ❌ No samples read")
        
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
        running = False
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



