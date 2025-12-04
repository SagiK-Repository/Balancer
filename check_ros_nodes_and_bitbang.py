#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
ROS 노드 상태 확인 및 BitBangSPI 연결 점검
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
    print("="*60)
    print("ROS 노드 상태 및 BitBangSPI 연결 점검")
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
        
        # 1. ROS 노드 확인
        print("1. ROS 노드 상태 확인")
        print("-" * 60)
        
        success, output, error = run_command(client, f"{env_cmd} && timeout 3 rosnode list 2>&1", timeout=5)
        
        if success and output:
            nodes = [line.strip() for line in output.strip().split('\n') if line.strip()]
            print(f"   ✅ Found {len(nodes)} nodes:")
            for node in nodes:
                print(f"      - {node}")
                
                # 각 노드의 상세 정보
                if 'spi' in node.lower() or 'hardware' in node.lower():
                    print(f"\n      📋 Node Info for {node}:")
                    success2, output2, error2 = run_command(client, f"{env_cmd} && timeout 2 rosnode info {node} 2>&1", timeout=4)
                    if success2 and output2:
                        lines = output2.strip().split('\n')
                        for line in lines[:20]:  # 처음 20줄만
                            if line.strip():
                                print(f"         {line.strip()}")
        else:
            print("   ❌ Could not list nodes")
            if error:
                print(f"   Error: {error}")
        
        # 2. ROS 토픽 확인
        print("\n2. ROS 토픽 확인")
        print("-" * 60)
        
        success, output, error = run_command(client, f"{env_cmd} && timeout 3 rostopic list 2>&1", timeout=5)
        
        if success and output:
            topics = [line.strip() for line in output.strip().split('\n') if line.strip()]
            print(f"   ✅ Found {len(topics)} topics:")
            for topic in topics:
                print(f"      - {topic}")
                
                # motor_status 토픽 상세 확인
                if 'motor_status' in topic.lower():
                    print(f"\n      📋 Topic Info for {topic}:")
                    success2, output2, error2 = run_command(client, f"{env_cmd} && timeout 2 rostopic info {topic} 2>&1", timeout=4)
                    if success2 and output2:
                        for line in output2.strip().split('\n')[:10]:
                            if line.strip():
                                print(f"         {line.strip()}")
                    
                    # 최신 메시지 샘플
                    print(f"\n      📊 Latest message sample:")
                    success3, output3, error3 = run_command(client, f"{env_cmd} && timeout 2 rostopic echo {topic} -n 1 2>&1 | head -30", timeout=4)
                    if success3 and output3:
                        for line in output3.strip().split('\n')[:15]:
                            if line.strip():
                                print(f"         {line.strip()}")
        else:
            print("   ⚠️  Could not list topics")
        
        # 3. BitBangSPI 노드 로그 확인
        print("\n3. BitBangSPI 노드 로그 확인")
        print("-" * 60)
        
        success, output, error = run_command(client, "tail -50 /tmp/spi_node.log 2>/dev/null", timeout=3)
        
        if success and output:
            print("   📋 Recent log entries (last 50 lines):")
            lines = output.strip().split('\n')
            error_count = 0
            warning_count = 0
            
            for line in lines:
                if 'ERROR' in line or 'error' in line.lower():
                    print(f"      ❌ {line.strip()}")
                    error_count += 1
                elif 'WARN' in line or 'warning' in line.lower():
                    print(f"      ⚠️  {line.strip()}")
                    warning_count += 1
                elif 'GPIO' in line or 'SPI' in line or 'hall' in line.lower():
                    print(f"      ℹ️  {line.strip()}")
            
            if error_count == 0 and warning_count == 0:
                print("      ✅ No errors or warnings found in recent logs")
                print("      (Showing last 10 lines):")
                for line in lines[-10:]:
                    if line.strip():
                        print(f"         {line.strip()}")
        else:
            print("   ⚠️  Could not read log file")
        
        # 4. GPIO 상태 확인
        print("\n4. GPIO 상태 확인")
        print("-" * 60)
        
        # gpiod 라이브러리 확인
        success, output, error = run_command(client, "ldconfig -p | grep gpiod", timeout=3)
        if success and output:
            print("   ✅ libgpiod found")
        else:
            print("   ⚠️  libgpiod not found in library path")
        
        # GPIO 칩 확인
        success, output, error = run_command(client, "gpiodetect 2>&1", timeout=3)
        if success and output:
            print("   📋 GPIO chips:")
            for line in output.strip().split('\n'):
                if line.strip():
                    print(f"      {line.strip()}")
        else:
            print("   ⚠️  Could not detect GPIO chips")
            if error:
                print(f"      Error: {error}")
        
        # 5. SPI 하드웨어 확인
        print("\n5. SPI 하드웨어 확인")
        print("-" * 60)
        
        # SPI 장치 확인
        success, output, error = run_command(client, "ls -l /dev/spi* 2>&1", timeout=3)
        if success and output:
            print("   📋 SPI devices:")
            for line in output.strip().split('\n'):
                if line.strip():
                    print(f"      {line.strip()}")
        else:
            print("   ⚠️  No SPI devices found or permission denied")
            if error and 'No such file' not in error:
                print(f"      Error: {error}")
        
        # SPI 정보 확인
        success, output, error = run_command(client, "lsmod | grep spi", timeout=3)
        if success and output:
            print("\n   📋 SPI kernel modules:")
            for line in output.strip().split('\n'):
                if line.strip():
                    print(f"      {line.strip()}")
        else:
            print("   ⚠️  No SPI modules loaded")
        
        # 6. 노드 설정 확인 (launch 파일 또는 파라미터)
        print("\n6. ROS 파라미터 확인")
        print("-" * 60)
        
        success, output, error = run_command(client, f"{env_cmd} && timeout 3 rosparam list 2>&1", timeout=5)
        
        if success and output:
            params = [line.strip() for line in output.strip().split('\n') if line.strip()]
            if params:
                print(f"   ✅ Found {len(params)} parameters")
                spi_params = [p for p in params if 'spi' in p.lower() or 'gpio' in p.lower() or 'hall' in p.lower()]
                if spi_params:
                    print("   📋 SPI/GPIO related parameters:")
                    for param in spi_params[:10]:
                        print(f"      - {param}")
                        # 파라미터 값 확인
                        success2, output2, error2 = run_command(client, f"{env_cmd} && timeout 2 rosparam get {param} 2>&1", timeout=4)
                        if success2 and output2:
                            print(f"         Value: {output2.strip()}")
                else:
                    print("   ℹ️  No SPI/GPIO related parameters found")
            else:
                print("   ℹ️  No parameters found")
        else:
            print("   ⚠️  Could not list parameters")
        
        # 7. 프로세스 상태 확인
        print("\n7. 프로세스 상태 확인")
        print("-" * 60)
        
        success, output, error = run_command(client, "ps aux | grep -E 'spi_hardware|roscore' | grep -v grep", timeout=3)
        
        if success and output:
            print("   📋 Running processes:")
            for line in output.strip().split('\n'):
                if line.strip():
                    parts = line.split()
                    pid = parts[1]
                    cmd = ' '.join(parts[10:])
                    print(f"      PID {pid}: {cmd[:80]}")
        else:
            print("   ⚠️  No matching processes found")
        
        # 8. 요약
        print("\n" + "="*60)
        print("점검 요약")
        print("="*60)
        
        # 노드 실행 여부
        success, output, error = run_command(client, "ps aux | grep spi_hardware_node | grep -v grep", timeout=3)
        node_running = success and "spi_hardware_node" in output
        
        # 토픽 발행 여부
        success, output, error = run_command(client, f"{env_cmd} && timeout 2 rostopic list 2>&1 | grep motor_status", timeout=4)
        topic_exists = success and "motor_status" in output
        
        # 에러 여부
        success, output, error = run_command(client, "tail -20 /tmp/spi_node.log 2>/dev/null | grep -i error", timeout=3)
        has_errors = success and output and "error" in output.lower()
        
        print(f"\n✅ ROS 노드 실행: {'Yes' if node_running else 'No'}")
        print(f"✅ 토픽 발행: {'Yes' if topic_exists else 'No'}")
        print(f"{'❌' if has_errors else '✅'} 에러 발생: {'Yes' if has_errors else 'No'}")
        
        if node_running and topic_exists and not has_errors:
            print("\n✅ BitBangSPI 연결 상태: 정상")
        elif node_running and topic_exists:
            print("\n⚠️  BitBangSPI 연결 상태: 부분적 문제 (에러 있음)")
        elif node_running:
            print("\n⚠️  BitBangSPI 연결 상태: 노드는 실행 중이지만 토픽 발행 안 됨")
        else:
            print("\n❌ BitBangSPI 연결 상태: 노드가 실행되지 않음")
        
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



