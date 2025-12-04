#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Raw 홀 센서 비트 데이터 확인 (간단 버전 - rosout 확인)
"""

import paramiko
import sys
import time
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
    except Exception as e:
        return False, "", str(e)

def main():
    """메인 함수"""
    print("="*80)
    print("Raw 홀 센서 비트 데이터 확인 (rosout 로그 확인)")
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
        
        # SPI Hardware Node 시작
        print("SPI Hardware Node 시작...")
        spi_node_cmd = f"{env_cmd} && nohup rosrun balance_robot_nodes spi_hardware_node > /tmp/spi_node.log 2>&1 &"
        client.exec_command(spi_node_cmd, timeout=2)
        time.sleep(3)
        
        duration = 30  # 30초
        
        print(f"\n✅ 준비 완료! 이제 바퀴를 직접 돌려보세요.")
        print(f"   모니터링 시간: {duration}초")
        print()
        print("="*80)
        print("Raw 홀 센서 비트 데이터 (rosout 로그)")
        print("="*80)
        print()
        print("Time(s)  Raw(Hex)  Raw(Bin)      Bit[7:0]")
        print("-" * 60)
        
        start_time = time.time()
        last_data = None
        
        try:
            while time.time() - start_time < duration:
                current_time = time.time() - start_time
                
                # 로그 파일에서 Raw Hall Data 찾기
                cmd = f"grep 'Raw Hall Data' /tmp/spi_node.log | tail -1"
                success, output, error = run_command(client, cmd, timeout=2)
                
                if success and output.strip():
                    # Raw Hall Data: 0xXX (0bXXXXXXXX) | Bits: [7:X] [6:X] ...
                    match = re.search(r'Raw Hall Data: 0x([0-9A-Fa-f]{2}) \(0b([01]{8})\)', output)
                    if match:
                        hex_val = match.group(1)
                        bin_val = match.group(2)
                        raw_int = int(hex_val, 16)
                        
                        if last_data != raw_int:
                            bits = f"[{bin_val[0]} {bin_val[1]} {bin_val[2]} {bin_val[3]} {bin_val[4]} {bin_val[5]} {bin_val[6]} {bin_val[7]}]"
                            print(f"{current_time:7.2f}  0x{hex_val:2s}    0b{bin_val:8s}  {bits}")
                            last_data = raw_int
                
                time.sleep(0.1)
                
        except KeyboardInterrupt:
            print("\n\nInterrupted by user.")
        except Exception as e:
            print(f"\n   [Exception: {e}]")
        
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
        
        return 0
        
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


