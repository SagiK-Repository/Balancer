#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Raw 홀 센서 비트 데이터 확인 스크립트
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
    
    print("="*80)
    print("Raw 홀 센서 비트 데이터 확인")
    print("="*80)
    print("\n⚠️  바퀴를 직접 돌려보면서 비트 변화를 확인하세요.")
    print("   30초간 모니터링합니다.\n")
    
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
        
        # 빌드
        print("코드 빌드 중...")
        build_cmd = f"{env_cmd} && cd ~/catkin_ws && catkin_make --pkg balance_robot_nodes 2>&1 | tail -20"
        success, output, error = run_command(client, build_cmd, timeout=60)
        if not success:
            print(f"❌ 빌드 실패:\n{error}")
            return 1
        print("✅ 빌드 완료\n")
        
        # 기존 프로세스 종료
        print("기존 ROS 프로세스 종료 중...")
        run_command(client, "pkill -f spi_hardware_node", timeout=3)
        run_command(client, "pkill -f roscore", timeout=3)
        time.sleep(2)
        
        # roscore 시작
        print("roscore 시작...")
        roscore_cmd = f"{env_cmd} && nohup roscore > /tmp/roscore.log 2>&1 &"
        client.exec_command(roscore_cmd, timeout=2)
        time.sleep(3)
        
        # roscore 실행 확인
        print("roscore 실행 확인...")
        for i in range(5):
            success, output, error = run_command(client, "ps aux | grep roscore | grep -v grep", timeout=3)
            if success and output.strip():
                print("   ✅ roscore 실행 중")
                break
            time.sleep(1)
        
        # ROS 마스터 연결 확인
        print("ROS 마스터 연결 확인...")
        for i in range(5):
            cmd = f"{env_cmd} && timeout 2 rostopic list > /dev/null 2>&1"
            success, output, error = run_command(client, cmd, timeout=3)
            if success:
                print("   ✅ ROS 마스터 연결 성공")
                break
            time.sleep(1)
        
        # SPI Hardware Node 시작
        print("SPI Hardware Node 시작...")
        spi_node_cmd = f"{env_cmd} && nohup rosrun balance_robot_nodes spi_hardware_node > /tmp/spi_node.log 2>&1 &"
        client.exec_command(spi_node_cmd, timeout=2)
        time.sleep(3)
        
        # spi_hardware_node 실행 확인
        print("spi_hardware_node 실행 확인...")
        for i in range(5):
            success, output, error = run_command(client, "ps aux | grep spi_hardware_node | grep -v grep", timeout=3)
            if success and output.strip():
                print("   ✅ spi_hardware_node 실행 중")
                break
            time.sleep(1)
        
        # 토픽 발행 확인
        print("토픽 발행 확인...")
        time.sleep(2)
        cmd = f"{env_cmd} && timeout 3 rostopic list | grep motor_status"
        success, output, error = run_command(client, cmd, timeout=5)
        if success and 'motor_status' in output:
            print("   ✅ /motor_status 토픽 존재")
        
        duration = 30  # 30초
        
        print(f"\n✅ 준비 완료! 이제 바퀴를 직접 돌려보세요.")
        print(f"   모니터링 시간: {duration}초")
        print()
        print("="*80)
        print("Raw 홀 센서 비트 데이터 (30초간)")
        print("="*80)
        print()
        print("Time(s)  Raw(Hex)  Raw(Bin)      Bit[7:0]              M0_State  M1_State  M0_Count  M1_Count")
        print("-" * 100)
        
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
                    message_text = '\n'.join(current_message)
                    
                    data = {}
                    import re
                    
                    match = re.search(r'raw_hall_data:\s*(\d+)', message_text)
                    if match:
                        data['raw_data'] = int(match.group(1))
                    
                    match = re.search(r'motor0_hall_state:\s*(\d+)', message_text)
                    if match:
                        data['motor0_state'] = int(match.group(1))
                    
                    match = re.search(r'motor1_hall_state:\s*(\d+)', message_text)
                    if match:
                        data['motor1_state'] = int(match.group(1))
                    
                    match = re.search(r'motor0_hall_count:\s*(-?\d+)', message_text)
                    if match:
                        data['motor0_count'] = int(match.group(1))
                    
                    match = re.search(r'motor1_hall_count:\s*(-?\d+)', message_text)
                    if match:
                        data['motor1_count'] = int(match.group(1))
                    
                    if 'raw_data' in data:
                        sample_count += 1
                        elapsed = time.time() - start_time
                        
                        raw = data['raw_data']
                        raw_hex = f"0x{raw:02X}"
                        raw_bin = f"0b{raw:08b}"
                        bits = f"[{raw>>7&1} {raw>>6&1} {raw>>5&1} {raw>>4&1} {raw>>3&1} {raw>>2&1} {raw>>1&1} {raw>>0&1}]"
                        
                        m0_state = data.get('motor0_state', 'N/A')
                        m1_state = data.get('motor1_state', 'N/A')
                        m0_count = data.get('motor0_count', 'N/A')
                        m1_count = data.get('motor1_count', 'N/A')
                        
                        # 변화가 있을 때만 표시
                        if 'raw_data' not in prev_data or prev_data['raw_data'] != raw:
                            print(f"{elapsed:7.2f}  {raw_hex:8s}  {raw_bin:12s}  {bits:20s}  {str(m0_state):9s}  {str(m1_state):9s}  {str(m0_count):9s}  {str(m1_count):9s}")
                            prev_data = data.copy()
                
                current_message = []
            else:
                current_message.append(line)
        
        # 홀 센서 읽기 시작
        print("rostopic echo 시작...")
        cmd = f"{env_cmd} && timeout {duration + 3} rostopic echo /motor_status 2>&1"
        stdin, stdout, stderr = client.exec_command(cmd, timeout=duration + 8)
        time.sleep(1)
        
        start_time = time.time()
        last_print_time = 0
        max_duration = duration + 6
        
        print("데이터 수신 대기 중...")
        
        try:
            while running and time.time() - start_time < max_duration:
                current_time = time.time() - start_time
                
                # 타임아웃 체크
                if current_time >= max_duration:
                    print(f"\n   [Timeout reached: {max_duration:.1f}s]")
                    break
                
                # 데이터 수신 확인
                if stdout.channel.recv_ready():
                    try:
                        chunk = stdout.channel.recv(4096).decode('utf-8', errors='ignore')
                        if chunk:
                            lines = chunk.split('\n')
                            for line in lines:
                                process_line(line)
                    except Exception as e:
                        print(f"\n   [Read error: {e}]")
                
                # 주기적으로 진행 상황 출력
                if current_time - last_print_time >= 5.0:
                    if current_time < duration:
                        remaining = duration - current_time
                        status = f"[진행 중: {current_time:.1f}s / {duration}s 남음: {remaining:.1f}s]"
                        if sample_count > 0:
                            status += f" (샘플: {sample_count})"
                        print(f"\n   {status}")
                    else:
                        print(f"\n   [모니터링 종료: {current_time:.1f}s] (샘플: {sample_count})")
                    last_print_time = current_time
                
                # 채널이 닫혔는지 확인
                if stdout.channel.exit_status_ready():
                    exit_status = stdout.channel.recv_exit_status()
                    print(f"\n   [Command finished with status: {exit_status}]")
                    # 남은 데이터 읽기
                    while stdout.channel.recv_ready():
                        try:
                            chunk = stdout.channel.recv(4096).decode('utf-8', errors='ignore')
                            lines = chunk.split('\n')
                            for line in lines:
                                process_line(line)
                        except:
                            break
                    break
                
                time.sleep(0.1)
                
        except KeyboardInterrupt:
            print("\n\nInterrupted by user.")
            running = False
        except Exception as e:
            print(f"\n   [Exception in read loop: {e}]")
        finally:
            try:
                if not stdout.channel.closed:
                    stdout.channel.close()
                if not stderr.channel.closed:
                    stderr.channel.close()
            except:
                pass
            
            print(f"\n   [데이터 수신 종료. 총 샘플: {sample_count}]")
        
        print("\n" + "="*80)
        print("비트 위치 분석 가이드")
        print("="*80)
        print()
        print("위의 출력을 보면서 다음을 확인하세요:")
        print("1. 바퀴를 돌릴 때 어떤 비트가 변화하는지")
        print("2. Motor0의 홀 센서가 어떤 비트 위치에 있는지 (0-2 또는 3-5 또는 다른 위치)")
        print("3. Motor1의 홀 센서가 어떤 비트 위치에 있는지")
        print()
        print("현재 코드는 다음과 같이 파싱하고 있습니다:")
        print("  - Motor0: 하위 3비트 (bit 0, 1, 2)")
        print("  - Motor1: 상위 3비트 (bit 3, 4, 5)")
        print()
        print("만약 실제 하드웨어 배치가 다르다면, spi_hardware_node.cpp의")
        print("readHallSensors() 함수에서 비트 마스킹을 수정해야 합니다.")
        print()
        
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
        
        print("\n프로그램 종료.")

if __name__ == "__main__":
    if sys.platform == 'win32':
        import io
        try:
            sys.stdout = io.TextIOWrapper(sys.stdout.buffer, encoding='utf-8')
            sys.stderr = io.TextIOWrapper(sys.stderr.buffer, encoding='utf-8')
        except:
            pass
    
    sys.exit(main())


