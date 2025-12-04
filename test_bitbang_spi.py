#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
BitBangSPI를 사용하여 모터 테스트
기존 balance_robot_nodes의 BitBangSPI 사용
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
    print("\n\nInterrupted by user. Stopping...")
    stop_motors()
    sys.exit(0)

def stop_motors():
    """모터 정지"""
    global ssh_client
    if ssh_client:
        try:
            # ROS 노드 종료
            cmd = "pkill -f spi_hardware_node || true"
            ssh_client.exec_command(cmd, timeout=2)
            time.sleep(0.5)
            # 모터 명령으로 0 출력
            cmd = "rostopic pub -1 /balance_robot/motor_command balance_robot_nodes/MotorCommand '{motor0_output: 0.0, motor1_output: 0.0, emergency_stop: true}' 2>/dev/null || true"
            ssh_client.exec_command(cmd, timeout=2)
        except:
            pass

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

def test_with_bitbang_spi(client):
    """BitBangSPI를 사용한 모터 테스트"""
    print("\n=== BitBangSPI Motor Test ===")
    
    setup_cmd = "cd ~/catkin_ws && source devel/setup.bash"
    
    # 1. 기존 노드가 실행 중인지 확인
    print("\n1. Checking if balance_robot_nodes is available...")
    success, output, error = run_command(client, f"{setup_cmd} && rospack find balance_robot_nodes", timeout=5)
    
    if not success:
        print("   ❌ balance_robot_nodes package not found")
        print("   Trying to use integrated_balancer with BitBangSPI simulation...")
        return False
    
    print("   ✅ balance_robot_nodes found")
    
    # 2. roscore 실행 확인
    print("\n2. Checking roscore...")
    success, output, error = run_command(client, "rostopic list 2>/dev/null | head -1", timeout=3)
    
    if not success or not output.strip():
        print("   ⚠️  roscore not running, starting...")
        # roscore를 백그라운드로 시작
        client.exec_command("roscore > /tmp/roscore.log 2>&1 &", timeout=2)
        time.sleep(3)
        print("   ✅ roscore started")
    else:
        print("   ✅ roscore is running")
    
    # 3. SPI Hardware Node 실행
    print("\n3. Starting SPI Hardware Node (BitBangSPI)...")
    node_cmd = f"{setup_cmd} && rosrun balance_robot_nodes spi_hardware_node > /tmp/spi_node.log 2>&1 &"
    client.exec_command(node_cmd, timeout=2)
    time.sleep(2)
    
    # 노드가 실행 중인지 확인
    success, output, error = run_command(client, "ps aux | grep spi_hardware_node | grep -v grep", timeout=3)
    if "spi_hardware_node" in output:
        print("   ✅ SPI Hardware Node started")
    else:
        print("   ❌ Failed to start SPI Hardware Node")
        print("   Check log: /tmp/spi_node.log")
        return False
    
    # 4. 모터 명령 전송
    print("\n4. Sending motor commands...")
    print("   Motor 0: 30% for 10 seconds")
    print("   Motor 1: 30% for 10 seconds")
    
    # 모터 명령 발행
    motor_cmd = f"{setup_cmd} && rostopic pub /balance_robot/motor_command balance_robot_nodes/MotorCommand '{{motor0_output: 30.0, motor1_output: 30.0, motor0_direction: true, motor1_direction: true, emergency_stop: false}}'"
    client.exec_command(motor_cmd, timeout=1)
    
    print("\n   Motors should be running now...")
    print("   Watch the wheels with your camera!")
    print("   (Running for 10 seconds)")
    
    # 10초 대기
    for i in range(10):
        time.sleep(1)
        print(f"   {10-i} seconds remaining...", end='\r')
    
    print("\n\n5. Stopping motors...")
    
    # 모터 정지
    stop_cmd = f"{setup_cmd} && rostopic pub -1 /balance_robot/motor_command balance_robot_nodes/MotorCommand '{{motor0_output: 0.0, motor1_output: 0.0, emergency_stop: true}}'"
    success, output, error = run_command(client, stop_cmd, timeout=5)
    
    # 노드 종료
    run_command(client, "pkill -f spi_hardware_node", timeout=2)
    
    print("   ✅ Motors stopped")
    
    # 5. 로그 확인
    print("\n6. Checking node logs...")
    success, output, error = run_command(client, "tail -20 /tmp/spi_node.log 2>/dev/null", timeout=3)
    if success and output:
        print("   Recent log entries:")
        print(output)
    
    return True

def main():
    """메인 함수"""
    global ssh_client
    
    signal.signal(signal.SIGINT, signal_handler)
    if sys.platform != 'win32':
        signal.signal(signal.SIGTERM, signal_handler)
    
    print("=== BitBangSPI Motor Test ===")
    print("Using balance_robot_nodes with BitBangSPI\n")
    
    client = create_ssh_client()
    if not client:
        return 1
    
    ssh_client = client
    
    try:
        if test_with_bitbang_spi(client):
            print("\n" + "="*50)
            print("✅ BitBangSPI test completed!")
            print("="*50)
            print("\nCheck if wheels rotated:")
            print("  - If YES: BitBangSPI works, HardwareSPI may have issues")
            print("  - If NO: Check motor driver and connections")
            return 0
        else:
            print("\n❌ BitBangSPI test failed")
            return 1
        
    except KeyboardInterrupt:
        print("\n\nInterrupted by user.")
        stop_motors()
        return 1
    except Exception as e:
        print(f"❌ Error: {e}")
        import traceback
        traceback.print_exc()
        stop_motors()
        return 1
    finally:
        try:
            stop_motors()
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



