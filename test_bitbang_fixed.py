#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
BitBangSPI 모터 테스트 (ROS IP 수정 후)
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
    stop_all()
    sys.exit(0)

def stop_all():
    """모든 프로세스 정리"""
    global ssh_client
    if ssh_client:
        try:
            ssh_client.exec_command("pkill -f roscore; pkill -f spi_hardware_node; pkill -f balance_controller", timeout=2)
            time.sleep(1)
            ssh_client.exec_command("rostopic pub -1 /balance_robot/motor_command balance_robot_nodes/MotorCommand '{motor0_output: 0.0, motor1_output: 0.0, emergency_stop: true}' 2>/dev/null || true", timeout=2)
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

def main():
    """메인 함수"""
    global ssh_client
    
    signal.signal(signal.SIGINT, signal_handler)
    if sys.platform != 'win32':
        signal.signal(signal.SIGTERM, signal_handler)
    
    print("=== BitBangSPI Motor Test (Fixed ROS IP) ===\n")
    
    client = create_ssh_client()
    if not client:
        return 1
    
    ssh_client = client
    
    try:
        setup_cmd = "cd ~/catkin_ws && source devel/setup.bash"
        
        # 1. 기존 프로세스 정리
        print("1. Cleaning up existing processes...")
        run_command(client, "pkill -f roscore; pkill -f spi_hardware_node; pkill -f balance_controller; sleep 1", timeout=5)
        
        # 2. 현재 IP 확인
        print("\n2. Checking current IP...")
        success, output, error = run_command(client, "hostname -I | awk '{print $1}'", timeout=3)
        if success:
            current_ip = output.strip()
            print(f"   Current IP: {current_ip}")
        
        # 3. ROS 환경 변수 확인
        print("\n3. Checking ROS environment...")
        check_cmd = f"bash -c 'source ~/.bashrc && echo ROS_MASTER_URI=$ROS_MASTER_URI && echo ROS_IP=$ROS_IP'"
        success, output, error = run_command(client, check_cmd, timeout=5)
        if success and output:
            print("   Environment variables:")
            for line in output.strip().split('\n'):
                if line.strip():
                    print(f"     {line.strip()}")
        
        # 4. roscore 시작 (환경 변수 적용)
        print("\n4. Starting roscore with correct IP...")
        # IP 직접 설정
        roscore_cmd = f"export ROS_MASTER_URI='http://{current_ip}:11311' && export ROS_IP='{current_ip}' && export ROS_HOSTNAME='{current_ip}' && {setup_cmd} && roscore > /tmp/roscore.log 2>&1 &"
        client.exec_command(roscore_cmd, timeout=2)
        time.sleep(3)
        
        # roscore 확인
        success, output, error = run_command(client, "ps aux | grep roscore | grep -v grep", timeout=3)
        if "roscore" in output:
            print("   ✅ roscore started")
        else:
            print("   ⚠️  roscore may not be running")
        
        # 5. SPI Hardware Node 시작
        print("\n5. Starting SPI Hardware Node (BitBangSPI)...")
        node_cmd = f"export ROS_MASTER_URI='http://{current_ip}:11311' && export ROS_IP='{current_ip}' && export ROS_HOSTNAME='{current_ip}' && {setup_cmd} && rosrun balance_robot_nodes spi_hardware_node > /tmp/spi_node.log 2>&1 &"
        client.exec_command(node_cmd, timeout=2)
        time.sleep(3)
        
        # 노드 확인
        success, output, error = run_command(client, "ps aux | grep spi_hardware_node | grep -v grep", timeout=3)
        if "spi_hardware_node" in output:
            print("   ✅ SPI Hardware Node started")
        else:
            print("   ⚠️  SPI Hardware Node may not be running")
            print("   Check log: tail -20 /tmp/spi_node.log")
        
        # 6. 토픽 확인
        print("\n6. Checking ROS topics...")
        time.sleep(2)
        success, output, error = run_command(client, f"export ROS_MASTER_URI='http://{current_ip}:11311' && export ROS_IP='{current_ip}' && {setup_cmd} && timeout 2 rostopic list 2>/dev/null", timeout=5)
        if success and output:
            topics = [line.strip() for line in output.strip().split('\n') if line.strip()]
            if topics:
                print(f"   ✅ Found {len(topics)} topics")
                if '/balance_robot/motor_command' in topics:
                    print("   ✅ Motor command topic available")
            else:
                print("   ⚠️  No topics found")
        
        # 7. 모터 명령 전송
        print("\n7. Sending motor commands (30% for 10 seconds)...")
        print("   Watch the wheels with your camera!")
        
        motor_cmd = f"export ROS_MASTER_URI='http://{current_ip}:11311' && export ROS_IP='{current_ip}' && {setup_cmd} && rostopic pub /balance_robot/motor_command balance_robot_nodes/MotorCommand '{{motor0_output: 30.0, motor1_output: 30.0, motor0_direction: true, motor1_direction: true, emergency_stop: false}}' &"
        client.exec_command(motor_cmd, timeout=1)
        
        # 10초 대기
        for i in range(10):
            time.sleep(1)
            print(f"   {10-i} seconds remaining...", end='\r')
        
        print("\n\n8. Stopping motors...")
        
        # 모터 정지
        stop_cmd = f"export ROS_MASTER_URI='http://{current_ip}:11311' && export ROS_IP='{current_ip}' && {setup_cmd} && rostopic pub -1 /balance_robot/motor_command balance_robot_nodes/MotorCommand '{{motor0_output: 0.0, motor1_output: 0.0, emergency_stop: true}}'"
        success, output, error = run_command(client, stop_cmd, timeout=5)
        
        # 프로세스 정리
        run_command(client, "pkill -f spi_hardware_node", timeout=2)
        
        print("   ✅ Motors stopped")
        
        # 9. 로그 확인
        print("\n9. Checking node logs...")
        success, output, error = run_command(client, "tail -30 /tmp/spi_node.log 2>/dev/null | grep -E 'ERROR|WARN|INFO|initialized|Motor' | tail -10", timeout=5)
        if success and output:
            print("   Recent log entries:")
            for line in output.strip().split('\n'):
                if line.strip():
                    print(f"     {line.strip()}")
        
        print("\n" + "="*50)
        print("✅ BitBangSPI test completed!")
        print("="*50)
        print("\nCheck if wheels rotated:")
        print("  - If YES: BitBangSPI works correctly")
        print("  - If NO: Check motor driver and connections")
        
        return 0
        
    except KeyboardInterrupt:
        print("\n\nInterrupted by user.")
        stop_all()
        return 1
    except Exception as e:
        print(f"❌ Error: {e}")
        import traceback
        traceback.print_exc()
        stop_all()
        return 1
    finally:
        try:
            stop_all()
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

