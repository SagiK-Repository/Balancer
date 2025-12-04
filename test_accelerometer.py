#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
가속도 센서 입력 테스트
"""

import paramiko
import sys
import time
import signal
import re

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
    
    print("="*70)
    print("가속도 센서 입력 테스트")
    print("="*70 + "\n")
    
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
        
        # 1. I2C 센서 노드 확인
        print("1. Checking I2C Sensor Node...")
        print("-" * 70)
        
        success, output, error = run_command(client, "ps aux | grep i2c_sensor_node | grep -v grep", timeout=3)
        
        if not success or "i2c_sensor_node" not in output:
            print("   ⚠️  I2C Sensor Node not running, starting...")
            node_cmd = f"{env_cmd} && rosrun balance_robot_nodes i2c_sensor_node > /tmp/i2c_node.log 2>&1 &"
            client.exec_command(node_cmd, timeout=2)
            time.sleep(3)
            print("   ✅ I2C Sensor Node started")
        else:
            print("   ✅ I2C Sensor Node is running")
        
        # 노드 로그 확인
        success, output, error = run_command(client, "tail -20 /tmp/i2c_node.log 2>/dev/null", timeout=3)
        if success and output:
            lines = output.strip().split('\n')
            for line in lines[-5:]:
                if line.strip() and ('ERROR' in line or 'WARN' in line or 'INFO' in line):
                    if 'ERROR' in line:
                        print(f"   ❌ {line.strip()}")
                    elif 'WARN' in line:
                        print(f"   ⚠️  {line.strip()}")
                    else:
                        print(f"   ℹ️  {line.strip()}")
        
        # 2. 센서 데이터 토픽 확인
        print("\n2. Checking sensor data topics...")
        print("-" * 70)
        
        success, output, error = run_command(client, f"{env_cmd} && timeout 3 rostopic list 2>&1", timeout=5)
        
        if success and output:
            topics = [line.strip() for line in output.strip().split('\n') if line.strip()]
            sensor_topics = [t for t in topics if 'sensor' in t.lower() or 'imu' in t.lower()]
            
            if sensor_topics:
                print("   📋 Sensor related topics:")
                for topic in sensor_topics:
                    print(f"      - {topic}")
            else:
                print("   ⚠️  No sensor topics found")
        
        # 3. 가속도 센서 데이터 읽기
        print("\n3. Reading accelerometer data...")
        print("   (Try moving the robot to see changes)")
        print()
        print("Time(s)  Accel_X(m/s²)  Accel_Y(m/s²)  Accel_Z(m/s²)  Roll(°)  Pitch(°)  Temp(℃)")
        print("-" * 85)
        
        start_time = time.time()
        sample_count = 0
        current_message = []
        accel_data = []
        
        def process_line(line):
            nonlocal sample_count, current_message, accel_data
            
            line = line.strip()
            if not line:
                return
            
            if line.startswith('---'):
                if current_message:
                    message_text = '\n'.join(current_message)
                    
                    data = {}
                    
                    # 가속도 데이터
                    match = re.search(r'accel_x:\s*([-\d.]+)', message_text)
                    if match:
                        data['accel_x'] = float(match.group(1))
                    
                    match = re.search(r'accel_y:\s*([-\d.]+)', message_text)
                    if match:
                        data['accel_y'] = float(match.group(1))
                    
                    match = re.search(r'accel_z:\s*([-\d.]+)', message_text)
                    if match:
                        data['accel_z'] = float(match.group(1))
                    
                    # 각도 데이터
                    match = re.search(r'roll_angle:\s*([-\d.]+)', message_text)
                    if match:
                        data['roll'] = float(match.group(1))
                    
                    match = re.search(r'pitch_angle:\s*([-\d.]+)', message_text)
                    if match:
                        data['pitch'] = float(match.group(1))
                    
                    # 온도
                    match = re.search(r'temperature:\s*([-\d.]+)', message_text)
                    if match:
                        data['temp'] = float(match.group(1))
                    
                    # I2C 통신 상태
                    match = re.search(r'i2c_communication_ok:\s*(\w+)', message_text)
                    if match:
                        data['i2c_ok'] = match.group(1).lower() == 'true'
                    
                    if data:
                        sample_count += 1
                        elapsed = time.time() - start_time
                        accel_data.append((elapsed, data))
                        
                        accel_x = data.get('accel_x', 0.0)
                        accel_y = data.get('accel_y', 0.0)
                        accel_z = data.get('accel_z', 0.0)
                        roll = data.get('roll', 0.0)
                        pitch = data.get('pitch', 0.0)
                        temp = data.get('temp', 0.0)
                        
                        # 변화 감지 (이전 값과 비교)
                        change_marker = ""
                        if len(accel_data) > 1:
                            prev_data = accel_data[-2][1]
                            if abs(accel_x - prev_data.get('accel_x', 0)) > 0.5:
                                change_marker += " X"
                            if abs(accel_y - prev_data.get('accel_y', 0)) > 0.5:
                                change_marker += " Y"
                            if abs(accel_z - prev_data.get('accel_z', 0)) > 0.5:
                                change_marker += " Z"
                        
                        if change_marker:
                            print(f"{elapsed:7.2f}    {accel_x:13.3f}  {accel_y:13.3f}  {accel_z:13.3f}  {roll:6.2f}   {pitch:6.2f}   {temp:6.2f} ***{change_marker}")
                        else:
                            # 처음 몇 개와 주기적으로만 표시
                            if sample_count <= 10 or elapsed % 2.0 < 0.1:
                                print(f"{elapsed:7.2f}    {accel_x:13.3f}  {accel_y:13.3f}  {accel_z:13.3f}  {roll:6.2f}   {pitch:6.2f}   {temp:6.2f}")
                
                current_message = []
            else:
                current_message.append(line)
        
        # /sensor_data 토픽 읽기
        cmd = f"{env_cmd} && timeout 20 rostopic echo /sensor_data"
        stdin, stdout, stderr = client.exec_command(cmd, timeout=25)
        
        start_time = time.time()
        last_print_time = 0
        
        while running and time.time() - start_time < 20:
            if stdout.channel.recv_ready():
                chunk = stdout.channel.recv(4096).decode('utf-8', errors='ignore')
                lines = chunk.split('\n')
                for line in lines:
                    process_line(line)
            
            # 주기적으로 진행 상황 출력
            current_time = time.time() - start_time
            if current_time - last_print_time >= 5.0:
                print(f"\n   [Progress: {current_time:.1f}s / 20s]")
                last_print_time = current_time
            
            time.sleep(0.05)
        
        # 4. 결과 분석
        print("\n" + "="*70)
        print("가속도 센서 테스트 결과")
        print("="*70)
        
        print(f"\n📊 총 샘플 수: {sample_count}")
        
        if accel_data:
            # 통계 계산
            accel_x_values = [d[1].get('accel_x', 0) for d in accel_data if 'accel_x' in d[1]]
            accel_y_values = [d[1].get('accel_y', 0) for d in accel_data if 'accel_y' in d[1]]
            accel_z_values = [d[1].get('accel_z', 0) for d in accel_data if 'accel_z' in d[1]]
            
            if accel_x_values:
                print(f"\n📈 Accel_X 통계:")
                print(f"   평균: {sum(accel_x_values)/len(accel_x_values):.3f} m/s²")
                print(f"   최소: {min(accel_x_values):.3f} m/s²")
                print(f"   최대: {max(accel_x_values):.3f} m/s²")
                print(f"   범위: {max(accel_x_values) - min(accel_x_values):.3f} m/s²")
            
            if accel_y_values:
                print(f"\n📈 Accel_Y 통계:")
                print(f"   평균: {sum(accel_y_values)/len(accel_y_values):.3f} m/s²")
                print(f"   최소: {min(accel_y_values):.3f} m/s²")
                print(f"   최대: {max(accel_y_values):.3f} m/s²")
                print(f"   범위: {max(accel_y_values) - min(accel_y_values):.3f} m/s²")
            
            if accel_z_values:
                print(f"\n📈 Accel_Z 통계:")
                print(f"   평균: {sum(accel_z_values)/len(accel_z_values):.3f} m/s²")
                print(f"   최소: {min(accel_z_values):.3f} m/s²")
                print(f"   최대: {max(accel_z_values):.3f} m/s²")
                print(f"   범위: {max(accel_z_values) - min(accel_z_values):.3f} m/s²")
            
            # 중력 벡터 확인 (정지 상태에서 Z축이 약 9.81 m/s²)
            if accel_z_values:
                avg_z = sum(accel_z_values) / len(accel_z_values)
                if abs(avg_z - 9.81) < 2.0 or abs(avg_z + 9.81) < 2.0:
                    print(f"\n✅ 중력 벡터 감지: Z축 평균 = {avg_z:.3f} m/s² (예상: ±9.81 m/s²)")
                else:
                    print(f"\n⚠️  중력 벡터 이상: Z축 평균 = {avg_z:.3f} m/s² (예상: ±9.81 m/s²)")
            
            # 변화 확인
            changes_detected = False
            if len(accel_data) > 1:
                for i in range(1, len(accel_data)):
                    prev = accel_data[i-1][1]
                    curr = accel_data[i][1]
                    if 'accel_x' in prev and 'accel_x' in curr:
                        if abs(curr['accel_x'] - prev['accel_x']) > 0.5:
                            changes_detected = True
                            break
                    if 'accel_y' in prev and 'accel_y' in curr:
                        if abs(curr['accel_y'] - prev['accel_y']) > 0.5:
                            changes_detected = True
                            break
                    if 'accel_z' in prev and 'accel_z' in curr:
                        if abs(curr['accel_z'] - prev['accel_z']) > 0.5:
                            changes_detected = True
                            break
            
            if changes_detected:
                print("\n✅ 가속도 값 변화 감지됨 (센서가 정상 작동 중)")
            else:
                print("\n⚠️  가속도 값 변화가 거의 없음 (정지 상태이거나 센서 문제 가능)")
            
            # I2C 통신 상태
            i2c_ok_count = sum(1 for d in accel_data if d[1].get('i2c_ok', False))
            if i2c_ok_count > 0:
                print(f"\n✅ I2C 통신: 정상 ({i2c_ok_count}/{len(accel_data)} 샘플)")
            else:
                print(f"\n❌ I2C 통신: 실패")
        
        else:
            print("\n❌ 가속도 데이터를 읽을 수 없음")
            print("   - I2C 센서 노드가 실행 중인지 확인")
            print("   - /sensor_data 토픽이 발행되는지 확인")
            print("   - I2C 연결 상태 확인")
        
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



