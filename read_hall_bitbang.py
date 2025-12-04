#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
BitBangSPI로 실행 중일 때 /motor_status 토픽에서 홀 센서 값 읽기
"""

import paramiko
import sys
import time
import signal
import re

HOSTNAME = "192.168.0.28"
USERNAME = "tbot3"
PASSWORD = "1234"

ssh_client = None
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
        print("✅ Connected successfully!")
        return client
    except Exception as e:
        print(f"❌ Connection failed: {e}")
        return None

def run_command_streaming(client, command, callback=None, timeout=30):
    """명령 실행 (스트리밍 출력)"""
    try:
        stdin, stdout, stderr = client.exec_command(command, timeout=timeout)
        start_time = time.time()
        
        output_lines = []
        
        while running:
            if time.time() - start_time > timeout:
                stdout.channel.close()
                stderr.channel.close()
                return False, output_lines, f"Timeout after {timeout}s"
            
            if stdout.channel.recv_ready():
                chunk = stdout.channel.recv(4096).decode('utf-8', errors='ignore')
                lines = chunk.split('\n')
                for line in lines:
                    if line.strip():
                        output_lines.append(line)
                        if callback:
                            callback(line)
            
            if stderr.channel.recv_stderr_ready():
                error_chunk = stderr.channel.recv_stderr(4096).decode('utf-8', errors='ignore')
                if error_chunk.strip():
                    output_lines.append(f"ERROR: {error_chunk}")
            
            if stdout.channel.exit_status_ready():
                # 남은 데이터 읽기
                while stdout.channel.recv_ready():
                    chunk = stdout.channel.recv(4096).decode('utf-8', errors='ignore')
                    lines = chunk.split('\n')
                    for line in lines:
                        if line.strip():
                            output_lines.append(line)
                            if callback:
                                callback(line)
                break
            
            time.sleep(0.05)
        
        return True, output_lines, ""
    except KeyboardInterrupt:
        raise
    except Exception as e:
        return False, output_lines, str(e)

def parse_motor_status(message_lines):
    """MotorStatus 메시지 파싱 (여러 줄)"""
    data = {}
    message_text = '\n'.join(message_lines)
    
    # motor0_hall_state: 0
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
    
    return data

def main():
    """메인 함수"""
    global ssh_client, running
    
    signal.signal(signal.SIGINT, signal_handler)
    if sys.platform != 'win32':
        signal.signal(signal.SIGTERM, signal_handler)
    
    print("=== Reading Hall Sensor from /motor_status (BitBangSPI) ===\n")
    
    client = create_ssh_client()
    if not client:
        return 1
    
    ssh_client = client
    
    try:
        # 현재 IP 확인
        success, output, error = run_command_streaming(client, "hostname -I | awk '{print $1}'", timeout=3)
        if success and output:
            current_ip = output[0].strip() if output else "192.168.0.28"
        else:
            current_ip = "192.168.0.28"
        
        setup_cmd = "cd ~/catkin_ws && source devel/setup.bash"
        env_cmd = f"export ROS_MASTER_URI='http://{current_ip}:11311' && export ROS_IP='{current_ip}' && {setup_cmd}"
        
        # 노드 확인
        print("Checking SPI Hardware Node...")
        success, output, error = run_command_streaming(client, "ps aux | grep spi_hardware_node | grep -v grep", timeout=3)
        
        if not success or not any("spi_hardware_node" in line for line in output):
            print("   ⚠️  SPI Hardware Node not running, starting...")
            node_cmd = f"{env_cmd} && rosrun balance_robot_nodes spi_hardware_node > /tmp/spi_node.log 2>&1 &"
            client.exec_command(node_cmd, timeout=2)
            time.sleep(3)
            print("   ✅ SPI Hardware Node started")
        else:
            print("   ✅ SPI Hardware Node is running")
        
        # 토픽 읽기
        print("\nReading from /motor_status topic...")
        print("(Press Ctrl+C to stop)\n")
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
            
            # 메시지 구분자 확인
            if line.startswith('---'):
                # 이전 메시지 파싱
                if current_message:
                    data = parse_motor_status(current_message)
                    
                    if data:
                        sample_count += 1
                        elapsed = time.time() - start_time
                        
                        # 변화 감지
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
                
                # 새 메시지 시작
                current_message = []
            else:
                # 메시지 라인 추가
                current_message.append(line)
        
        # rostopic echo 실행
        cmd = f"{env_cmd} && rostopic echo /motor_status"
        success, output, error = run_command_streaming(client, cmd, callback=process_line, timeout=60)
        
        print(f"\n\nRead {sample_count} samples")
        
        if not success:
            print(f"Error: {error}")
        
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

