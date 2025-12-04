#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
모터 출력하면서 30초간 홀 센서 움직임 파악
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
    print("모터 출력 + 홀 센서 움직임 파악 (30초)")
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
        
        # 1. 노드 확인 및 시작
        print("1. Checking ROS nodes...")
        success, output, error = run_command(client, "ps aux | grep spi_hardware_node | grep -v grep", timeout=3)
        
        if not success or "spi_hardware_node" not in output:
            print("   ⚠️  SPI Hardware Node not running, starting...")
            node_cmd = f"{env_cmd} && rosrun balance_robot_nodes spi_hardware_node > /tmp/spi_node.log 2>&1 &"
            client.exec_command(node_cmd, timeout=2)
            time.sleep(3)
            print("   ✅ SPI Hardware Node started")
        else:
            print("   ✅ SPI Hardware Node is running")
        
        # 2. 모터 명령 토픽 확인
        print("\n2. Checking motor command topic...")
        success, output, error = run_command(client, f"{env_cmd} && timeout 2 rostopic list 2>&1 | grep motor_command", timeout=4)
        
        if success and "motor_command" in output:
            print("   ✅ /motor_command topic found")
        else:
            print("   ⚠️  /motor_command topic not found")
        
        # 3. 모터 출력 시작 (30% 속도)
        print("\n3. Starting motor output (30% speed)...")
        
        # 모터 명령 발행 스크립트 작성
        motor_cmd_script = f'''#!/usr/bin/env python2
import rospy
from balance_robot_nodes.msg import MotorCommand
import sys

rospy.init_node('motor_test', anonymous=True)
pub = rospy.Publisher('/motor_command', MotorCommand, queue_size=10)
rospy.sleep(1)

cmd = MotorCommand()
cmd.motor0_output = 30.0
cmd.motor1_output = 30.0
cmd.emergency_stop = False

duration = 30.0
start_time = rospy.Time.now()

rate = rospy.Rate(10)
while not rospy.is_shutdown() and (rospy.Time.now() - start_time).to_sec() < duration:
    pub.publish(cmd)
    rate.sleep()

# 정지 명령
cmd.motor0_output = 0.0
cmd.motor1_output = 0.0
for i in range(5):
    pub.publish(cmd)
    rate.sleep()
'''
        
        write_cmd = f"cat > /tmp/motor_test.py << 'ENDOFFILE'\n{motor_cmd_script}\nENDOFFILE"
        success, _, _ = run_command(client, write_cmd, timeout=5)
        
        if success:
            chmod_cmd = "chmod +x /tmp/motor_test.py"
            run_command(client, chmod_cmd, timeout=2)
            
            # 모터 명령 발행 시작 (백그라운드)
            motor_cmd = f"{env_cmd} && python2 /tmp/motor_test.py > /tmp/motor_test.log 2>&1 &"
            client.exec_command(motor_cmd, timeout=2)
            time.sleep(1)
            print("   ✅ Motor command started (30% speed for 30 seconds)")
        
        # 4. 홀 센서 값 읽기 및 분석
        print("\n4. Reading hall sensor values during motor operation...")
        print("   (Monitoring for 30 seconds)")
        print()
        print("Time(s)  Motor0_Hall  Motor0_Count  Motor1_Hall  Motor1_Count  Changes")
        print("-" * 70)
        
        start_time = time.time()
        prev_data = {}
        sample_count = 0
        current_message = []
        hall_changes = {
            'motor0_hall': [],
            'motor0_count': [],
            'motor1_hall': [],
            'motor1_count': []
        }
        
        def process_line(line):
            nonlocal prev_data, sample_count, current_message, hall_changes
            
            line = line.strip()
            if not line:
                return
            
            if line.startswith('---'):
                if current_message:
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
                        
                        # 변화 감지 및 기록
                        changes = []
                        if 'motor0_hall' in data:
                            if 'motor0_hall' in prev_data and data['motor0_hall'] != prev_data['motor0_hall']:
                                changes.append("M0_H")
                                hall_changes['motor0_hall'].append((elapsed, data['motor0_hall']))
                            elif 'motor0_hall' not in prev_data:
                                hall_changes['motor0_hall'].append((elapsed, data['motor0_hall']))
                            
                            if 'motor0_count' in data:
                                if 'motor0_count' in prev_data and data['motor0_count'] != prev_data['motor0_count']:
                                    changes.append("M0_C++")
                                    hall_changes['motor0_count'].append((elapsed, data['motor0_count']))
                                elif 'motor0_count' not in prev_data:
                                    hall_changes['motor0_count'].append((elapsed, data['motor0_count']))
                        
                        if 'motor1_hall' in data:
                            if 'motor1_hall' in prev_data and data['motor1_hall'] != prev_data['motor1_hall']:
                                changes.append("M1_H")
                                hall_changes['motor1_hall'].append((elapsed, data['motor1_hall']))
                            elif 'motor1_hall' not in prev_data:
                                hall_changes['motor1_hall'].append((elapsed, data['motor1_hall']))
                            
                            if 'motor1_count' in data:
                                if 'motor1_count' in prev_data and data['motor1_count'] != prev_data['motor1_count']:
                                    changes.append("M1_C++")
                                    hall_changes['motor1_count'].append((elapsed, data['motor1_count']))
                                elif 'motor1_count' not in prev_data:
                                    hall_changes['motor1_count'].append((elapsed, data['motor1_count']))
                        
                        change_str = ", ".join(changes) if changes else "-"
                        
                        m0_hall = data.get('motor0_hall', 'N/A')
                        m0_count = data.get('motor0_count', 'N/A')
                        m1_hall = data.get('motor1_hall', 'N/A')
                        m1_count = data.get('motor1_count', 'N/A')
                        
                        # 변화가 있을 때만 강조 표시
                        if changes:
                            print(f"{elapsed:7.2f}    {str(m0_hall):11s}  {str(m0_count):12s}  {str(m1_hall):11s}  {str(m1_count):12s}  *** {change_str}")
                        else:
                            # 처음 몇 개만 표시, 그 다음은 변화 있을 때만
                            if sample_count <= 10 or elapsed % 5.0 < 0.5:
                                print(f"{elapsed:7.2f}    {str(m0_hall):11s}  {str(m0_count):12s}  {str(m1_hall):11s}  {str(m1_count):12s}  {change_str}")
                        
                        prev_data.update(data)
                
                current_message = []
            else:
                current_message.append(line)
        
        # rostopic echo 실행
        cmd = f"{env_cmd} && timeout 35 rostopic echo /motor_status"
        stdin, stdout, stderr = client.exec_command(cmd, timeout=40)
        
        start_time = time.time()
        last_print_time = 0
        
        while running and time.time() - start_time < 35:
            if stdout.channel.recv_ready():
                chunk = stdout.channel.recv(4096).decode('utf-8', errors='ignore')
                lines = chunk.split('\n')
                for line in lines:
                    process_line(line)
            
            # 주기적으로 진행 상황 출력
            current_time = time.time() - start_time
            if current_time - last_print_time >= 5.0:
                print(f"\n   [Progress: {current_time:.1f}s / 30s]")
                last_print_time = current_time
            
            time.sleep(0.05)
        
        # 5. 결과 분석
        print("\n" + "="*70)
        print("결과 분석")
        print("="*70)
        
        print(f"\n📊 총 샘플 수: {sample_count}")
        
        print(f"\n🔄 Motor0 Hall State 변화:")
        if hall_changes['motor0_hall']:
            print(f"   총 {len(hall_changes['motor0_hall'])}회 변화")
            print(f"   첫 변화: {hall_changes['motor0_hall'][0][0]:.2f}초 (값: {hall_changes['motor0_hall'][0][1]})")
            if len(hall_changes['motor0_hall']) > 1:
                print(f"   마지막 변화: {hall_changes['motor0_hall'][-1][0]:.2f}초 (값: {hall_changes['motor0_hall'][-1][1]})")
                # 변화 간격 계산
                intervals = [hall_changes['motor0_hall'][i+1][0] - hall_changes['motor0_hall'][i][0] 
                           for i in range(len(hall_changes['motor0_hall'])-1)]
                if intervals:
                    avg_interval = sum(intervals) / len(intervals)
                    print(f"   평균 변화 간격: {avg_interval:.3f}초")
        else:
            print("   ⚠️  변화 없음")
        
        print(f"\n🔄 Motor0 Hall Count 변화:")
        if hall_changes['motor0_count']:
            print(f"   총 {len(hall_changes['motor0_count'])}회 증가")
            if len(hall_changes['motor0_count']) > 1:
                start_count = hall_changes['motor0_count'][0][1]
                end_count = hall_changes['motor0_count'][-1][1]
                print(f"   시작: {start_count}, 종료: {end_count}, 증가량: {end_count - start_count}")
        else:
            print("   ⚠️  변화 없음")
        
        print(f"\n🔄 Motor1 Hall State 변화:")
        if hall_changes['motor1_hall']:
            print(f"   총 {len(hall_changes['motor1_hall'])}회 변화")
            print(f"   첫 변화: {hall_changes['motor1_hall'][0][0]:.2f}초 (값: {hall_changes['motor1_hall'][0][1]})")
            if len(hall_changes['motor1_hall']) > 1:
                print(f"   마지막 변화: {hall_changes['motor1_hall'][-1][0]:.2f}초 (값: {hall_changes['motor1_hall'][-1][1]})")
                intervals = [hall_changes['motor1_hall'][i+1][0] - hall_changes['motor1_hall'][i][0] 
                           for i in range(len(hall_changes['motor1_hall'])-1)]
                if intervals:
                    avg_interval = sum(intervals) / len(intervals)
                    print(f"   평균 변화 간격: {avg_interval:.3f}초")
        else:
            print("   ⚠️  변화 없음")
        
        print(f"\n🔄 Motor1 Hall Count 변화:")
        if hall_changes['motor1_count']:
            print(f"   총 {len(hall_changes['motor1_count'])}회 증가")
            if len(hall_changes['motor1_count']) > 1:
                start_count = hall_changes['motor1_count'][0][1]
                end_count = hall_changes['motor1_count'][-1][1]
                print(f"   시작: {start_count}, 종료: {end_count}, 증가량: {end_count - start_count}")
        else:
            print("   ⚠️  변화 없음")
        
        # 최종 상태
        print(f"\n📈 최종 상태:")
        if prev_data:
            print(f"   Motor0: Hall={prev_data.get('motor0_hall', 'N/A')}, Count={prev_data.get('motor0_count', 'N/A')}")
            print(f"   Motor1: Hall={prev_data.get('motor1_hall', 'N/A')}, Count={prev_data.get('motor1_count', 'N/A')}")
        
        # 종합 판단
        print(f"\n✅ 종합 판단:")
        total_changes = (len(hall_changes['motor0_hall']) + len(hall_changes['motor0_count']) + 
                        len(hall_changes['motor1_hall']) + len(hall_changes['motor1_count']))
        
        if total_changes > 0:
            print(f"   ✅ 홀 센서가 정상적으로 작동하고 있습니다!")
            print(f"   ✅ 모터 회전이 홀 센서에 감지되었습니다!")
        else:
            print(f"   ⚠️  홀 센서 값이 변하지 않았습니다.")
            print(f"   ⚠️  모터가 실제로 회전하지 않았거나 홀 센서 연결 문제일 수 있습니다.")
        
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
        # 모터 정지 확인
        try:
            print("\n5. Stopping motors...")
            stop_cmd = f"{env_cmd} && rostopic pub -1 /motor_command balance_robot_nodes/MotorCommand '{{motor0_output: 0.0, motor1_output: 0.0, emergency_stop: false}}' 2>/dev/null"
            run_command(client, stop_cmd, timeout=3)
            print("   ✅ Motors stopped")
        except:
            pass
        
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



