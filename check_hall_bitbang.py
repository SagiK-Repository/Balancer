#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
BitBangSPI로 실행 중일 때 홀 센서 입력 확인
balance_robot_nodes의 spi_hardware_node가 실행 중일 때 모터 상태 토픽에서 홀 센서 값 읽기
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
    
    print("=== Hall Sensor Check with BitBangSPI ===\n")
    
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
        
        # 2. SPI Hardware Node 확인
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
        
        # 3. 모터 상태 토픽에서 홀 센서 값 읽기
        print("\n3. Reading hall sensor values from motor_status topic...")
        print("   (Reading for 10 seconds, try rotating wheels manually)")
        print()
        
        # 여러 샘플 읽기
        print("Time(s)  Motor0_Hall  Motor0_Count  Motor1_Hall  Motor1_Count")
        print("-" * 60)
        
        start_time = time.time()
        prev_wheel0_hall = -1
        prev_wheel1_hall = -1
        prev_wheel0_count = -1
        prev_wheel1_count = -1
        
        for i in range(20):  # 2초 간격으로 20번 (약 40초)
            cmd = f"{env_cmd} && timeout 1 rostopic echo /balance_robot/motor_status -n 1 2>/dev/null"
            success, output, error = run_command(client, cmd, timeout=3)
            
            if success and output:
                # 값 추출
                wheel0_hall = None
                wheel0_count = None
                wheel1_hall = None
                wheel1_count = None
                
                for line in output.split('\n'):
                    line = line.strip()
                    if 'motor0_hall_state:' in line:
                        try:
                            wheel0_hall = int(line.split(':')[1].strip())
                        except:
                            pass
                    elif 'motor0_hall_count:' in line:
                        try:
                            wheel0_count = int(line.split(':')[1].strip())
                        except:
                            pass
                    elif 'motor1_hall_state:' in line:
                        try:
                            wheel1_hall = int(line.split(':')[1].strip())
                        except:
                            pass
                    elif 'motor1_hall_count:' in line:
                        try:
                            wheel1_count = int(line.split(':')[1].strip())
                        except:
                            pass
                
                elapsed = time.time() - start_time
                
                # 변화 감지
                change0 = ""
                change1 = ""
                if wheel0_hall is not None and prev_wheel0_hall != -1:
                    if wheel0_hall != prev_wheel0_hall:
                        change0 = " ⚠️ CHANGED"
                    if wheel0_count is not None and prev_wheel0_count != -1:
                        if wheel0_count != prev_wheel0_count:
                            change0 += " (COUNT++)"
                
                if wheel1_hall is not None and prev_wheel1_hall != -1:
                    if wheel1_hall != prev_wheel1_hall:
                        change1 = " ⚠️ CHANGED"
                    if wheel1_count is not None and prev_wheel1_count != -1:
                        if wheel1_count != prev_wheel1_count:
                            change1 += " (COUNT++)"
                
                # 출력
                w0_hall_str = str(wheel0_hall) if wheel0_hall is not None else "N/A"
                w0_count_str = str(wheel0_count) if wheel0_count is not None else "N/A"
                w1_hall_str = str(wheel1_hall) if wheel1_hall is not None else "N/A"
                w1_count_str = str(wheel1_count) if wheel1_count is not None else "N/A"
                
                print(f"{elapsed:7.2f}    {w0_hall_str:11s}  {w0_count_str:12s}  {w1_hall_str:11s}  {w1_count_str:12s}{change0}{change1}")
                
                prev_wheel0_hall = wheel0_hall if wheel0_hall is not None else prev_wheel0_hall
                prev_wheel1_hall = wheel1_hall if wheel1_hall is not None else prev_wheel1_hall
                prev_wheel0_count = wheel0_count if wheel0_count is not None else prev_wheel0_count
                prev_wheel1_count = wheel1_count if wheel1_count is not None else prev_wheel1_count
            else:
                elapsed = time.time() - start_time
                print(f"{elapsed:7.2f}    (No data)")
            
            time.sleep(2)  # 2초 간격
        
        # 4. 요약
        print("\n" + "="*60)
        print("Summary")
        print("="*60)
        print("\nHall Sensor Status:")
        if prev_wheel0_hall is not None or prev_wheel1_hall is not None:
            print("  ✅ Hall sensor data is being read")
            if prev_wheel0_hall == 0 and prev_wheel1_hall == 0:
                print("  ⚠️  All values are 0 - check hall sensor connections")
            else:
                print("  ✅ Non-zero values detected")
        else:
            print("  ❌ No hall sensor data found")
        
        print("\nRecommendations:")
        print("  - If values changed (⚠️): Hall sensors are working!")
        print("  - If all values are 0: Check hall sensor connections")
        print("  - If values don't change: Try rotating wheels manually")
        
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



