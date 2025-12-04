#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
양쪽 모터를 반대 방향으로 실행 + 홀 센서 확인
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
    
    print("="*80)
    print("양쪽 모터 반대 방향 실행 + 홀 센서 확인")
    print("="*80 + "\n")
    
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
        
        # 모터 속도 설정
        speed = 30.0
        duration = 15  # 15초
        
        print(f"모터 설정:")
        print(f"  Motor0: {speed}% (정방향)")
        print(f"  Motor1: {speed}% (역방향)")
        print(f"  시간: {duration}초")
        print()
        
        # 모터 명령 발행 스크립트 작성
        motor_script = f'''#!/usr/bin/env python2
# -*- coding: utf-8 -*-
import rospy
from balance_robot_nodes.msg import MotorCommand
import sys
import time

rospy.init_node('motor_reverse', anonymous=True)
pub = rospy.Publisher('/motor_command', MotorCommand, queue_size=10)
rospy.sleep(1)

cmd = MotorCommand()
cmd.motor0_output = {speed}
cmd.motor1_output = {speed}
cmd.motor0_direction = True   # Motor0: forward
cmd.motor1_direction = False  # Motor1: reverse
cmd.emergency_stop = False

start_time = time.time()
rate = rospy.Rate(10)

while not rospy.is_shutdown() and (time.time() - start_time) < {duration}:
    pub.publish(cmd)
    rate.sleep()

# Stop
cmd.motor0_output = 0.0
cmd.motor1_output = 0.0
for i in range(5):
    pub.publish(cmd)
    rate.sleep()
'''
        
        # 스크립트 업로드
        write_cmd = f"cat > /tmp/motor_reverse.py << 'ENDOFFILE'\n{motor_script}\nENDOFFILE"
        success, _, _ = run_command(client, write_cmd, timeout=5)
        
        if not success:
            print("❌ Failed to create motor script")
            return 1
        
        chmod_cmd = "chmod +x /tmp/motor_reverse.py"
        run_command(client, chmod_cmd, timeout=2)
        
        # 모터 실행 시작 (백그라운드)
        print("모터 시작 (반대 방향)...")
        motor_cmd = f"{env_cmd} && python2 /tmp/motor_reverse.py > /tmp/motor_reverse.log 2>&1 &"
        client.exec_command(motor_cmd, timeout=2)
        time.sleep(1)
        
        print(f"✅ 모터 명령 시작 (Motor0: 정방향, Motor1: 역방향, {duration}초)")
        print()
        print("="*80)
        print("홀 센서 실시간 모니터링 (Step 변화 분석)")
        print("="*80)
        print()
        print("Time(s)  Motor0_Hall  Motor0_Count  Motor1_Hall  Motor1_Count  Step_Change")
        print("-" * 80)
        
        start_time = time.time()
        prev_data = {}
        sample_count = 0
        current_message = []
        step_changes = []  # Step 변화량 기록
        
        def process_line(line):
            nonlocal prev_data, sample_count, current_message, step_changes
            
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
                        
                        # Step 변화량 계산
                        step_change = ""
                        if 'motor0_count' in data and 'motor0_count' in prev_data:
                            change = data['motor0_count'] - prev_data['motor0_count']
                            if change != 0:
                                step_changes.append(('M0', elapsed, change))
                                step_change += f"M0:+{change} " if change > 0 else f"M0:{change} "
                        
                        if 'motor1_count' in data and 'motor1_count' in prev_data:
                            change = data['motor1_count'] - prev_data['motor1_count']
                            if change != 0:
                                step_changes.append(('M1', elapsed, change))
                                step_change += f"M1:+{change} " if change > 0 else f"M1:{change} "
                        
                        # 변화 감지
                        changes = []
                        if 'motor0_hall' in data and 'motor0_hall' in prev_data:
                            if data['motor0_hall'] != prev_data['motor0_hall']:
                                changes.append("M0_H")
                        
                        if 'motor1_hall' in data and 'motor1_hall' in prev_data:
                            if data['motor1_hall'] != prev_data['motor1_hall']:
                                changes.append("M1_H")
                        
                        change_str = ", ".join(changes) if changes else "-"
                        
                        m0_hall = data.get('motor0_hall', 'N/A')
                        m0_count = data.get('motor0_count', 'N/A')
                        m1_hall = data.get('motor1_hall', 'N/A')
                        m1_count = data.get('motor1_count', 'N/A')
                        
                        # Step 변화가 있을 때만 표시
                        if step_change:
                            print(f"{elapsed:7.2f}    {str(m0_hall):11s}  {str(m0_count):12s}  {str(m1_hall):11s}  {str(m1_count):12s}  *** {step_change.strip()}")
                        elif changes:
                            # Hall State 변화만 있을 때
                            if sample_count <= 5 or elapsed % 3.0 < 0.1:
                                print(f"{elapsed:7.2f}    {str(m0_hall):11s}  {str(m0_count):12s}  {str(m1_hall):11s}  {str(m1_count):12s}  {change_str}")
                        
                        prev_data.update(data)
                
                current_message = []
            else:
                current_message.append(line)
        
        # 홀 센서 읽기 시작
        cmd = f"{env_cmd} && timeout {duration + 5} rostopic echo /motor_status"
        stdin, stdout, stderr = client.exec_command(cmd, timeout=duration + 10)
        
        start_time = time.time()
        last_print_time = 0
        
        while running and time.time() - start_time < (duration + 5):
            if stdout.channel.recv_ready():
                chunk = stdout.channel.recv(4096).decode('utf-8', errors='ignore')
                lines = chunk.split('\n')
                for line in lines:
                    process_line(line)
            
            # 주기적으로 진행 상황 출력
            current_time = time.time() - start_time
            if current_time - last_print_time >= 3.0:
                if current_time < duration:
                    print(f"\n   [Motor Running: {current_time:.1f}s / {duration}s]")
                else:
                    print(f"\n   [Motor Stopped: {current_time:.1f}s]")
                last_print_time = current_time
            
            time.sleep(0.05)
        
        # 결과 분석
        print("\n" + "="*80)
        print("결과 분석")
        print("="*80)
        
        print(f"\n📊 총 샘플 수: {sample_count}")
        
        # Step 변화량 통계
        if step_changes:
            m0_changes = [s for s in step_changes if s[0] == 'M0']
            m1_changes = [s for s in step_changes if s[0] == 'M1']
            
            print(f"\n📈 Motor0 Step 변화량 분석:")
            if m0_changes:
                change_values = [s[2] for s in m0_changes]
                print(f"   총 변화 횟수: {len(m0_changes)}")
                print(f"   변화량 범위: {min(change_values)} ~ {max(change_values)}")
                print(f"   평균 변화량: {sum(change_values)/len(change_values):.2f}")
                
                # 변화량 분포
                change_dist = {}
                for val in change_values:
                    change_dist[val] = change_dist.get(val, 0) + 1
                
                print(f"   변화량 분포:")
                for val in sorted(change_dist.keys()):
                    count = change_dist[val]
                    percentage = (count / len(change_values)) * 100
                    print(f"     +{val}: {count}회 ({percentage:.1f}%)")
                
                # 1이 아닌 변화량 확인
                non_one_changes = [s for s in m0_changes if abs(s[2]) != 1]
                if non_one_changes:
                    print(f"   ⚠️  1이 아닌 변화량: {len(non_one_changes)}회")
                    print(f"   예시:")
                    for s in non_one_changes[:5]:
                        print(f"     {s[1]:.2f}초: {s[2]}")
            else:
                print("   ⚠️  변화 없음")
            
            print(f"\n📈 Motor1 Step 변화량 분석:")
            if m1_changes:
                change_values = [s[2] for s in m1_changes]
                print(f"   총 변화 횟수: {len(m1_changes)}")
                print(f"   변화량 범위: {min(change_values)} ~ {max(change_values)}")
                print(f"   평균 변화량: {sum(change_values)/len(change_values):.2f}")
                
                # 변화량 분포
                change_dist = {}
                for val in change_values:
                    change_dist[val] = change_dist.get(val, 0) + 1
                
                print(f"   변화량 분포:")
                for val in sorted(change_dist.keys()):
                    count = change_dist[val]
                    percentage = (count / len(change_values)) * 100
                    print(f"     +{val}: {count}회 ({percentage:.1f}%)")
                
                # 1이 아닌 변화량 확인
                non_one_changes = [s for s in m1_changes if abs(s[2]) != 1]
                if non_one_changes:
                    print(f"   ⚠️  1이 아닌 변화량: {len(non_one_changes)}회")
                    print(f"   예시:")
                    for s in non_one_changes[:5]:
                        print(f"     {s[1]:.2f}초: {s[2]}")
            else:
                print("   ⚠️  변화 없음")
        
        # 최종 상태
        print(f"\n📈 최종 상태:")
        if prev_data:
            print(f"   Motor0: Hall={prev_data.get('motor0_hall', 'N/A')}, Count={prev_data.get('motor0_count', 'N/A')}")
            print(f"   Motor1: Hall={prev_data.get('motor1_hall', 'N/A')}, Count={prev_data.get('motor1_count', 'N/A')}")
        
        # Step이 1로 떨어지지 않는 이유 분석
        print(f"\n🔍 Step이 1로 떨어지지 않는 이유 분석:")
        print(f"   가능한 원인:")
        print(f"   1. 홀 센서 읽기 주기가 빠름 (100Hz) - 여러 샘플이 한 번에 증가")
        print(f"   2. 홀 센서 상태 변화가 빠름 - 한 샘플 사이에 여러 상태 변화")
        print(f"   3. 카운터 업데이트 로직 - Hall State 변화 시마다 카운트 증가")
        print(f"   4. 노이즈 또는 바운싱 - 센서 신호 불안정")
        
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
            print("\n모터 정지 확인...")
            stop_cmd = f"{env_cmd} && rostopic pub -1 /motor_command balance_robot_nodes/MotorCommand '{{motor0_output: 0.0, motor1_output: 0.0, motor0_direction: true, motor1_direction: true, emergency_stop: false}}' 2>/dev/null"
            run_command(client, stop_cmd, timeout=3)
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



