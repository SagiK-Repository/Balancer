#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
모터 정방향/역방향 동작 테스트 - 홀 센서 움직임 확인
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
    print("모터 정방향/역방향 동작 테스트 - 홀 센서 움직임 확인")
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
        
        # 모터 속도 설정
        speed = 30.0
        duration_per_direction = 10  # 각 방향당 10초
        
        print(f"\n모터 설정:")
        print(f"  속도: {speed}%")
        print(f"  정방향: {duration_per_direction}초")
        print(f"  역방향: {duration_per_direction}초")
        print(f"  총 시간: {duration_per_direction * 2}초")
        print()
        
        # 모터 명령 발행 스크립트 작성 (정방향 -> 역방향)
        motor_script = f'''#!/usr/bin/env python2
# -*- coding: utf-8 -*-
import rospy
from balance_robot_nodes.msg import MotorCommand
import sys
import time

rospy.init_node('motor_both_directions_test', anonymous=True)
pub = rospy.Publisher('/motor_command', MotorCommand, queue_size=10)
rospy.sleep(1)

cmd = MotorCommand()
cmd.motor0_output = {speed}
cmd.motor1_output = {speed}
cmd.emergency_stop = False

# 정방향
print("정방향 시작...")
cmd.motor0_direction = True
cmd.motor1_direction = True
start_time = time.time()
rate = rospy.Rate(10)

while not rospy.is_shutdown() and (time.time() - start_time) < {duration_per_direction}:
    pub.publish(cmd)
    rate.sleep()

# 역방향
print("역방향 시작...")
cmd.motor0_direction = False
cmd.motor1_direction = False
start_time = time.time()

while not rospy.is_shutdown() and (time.time() - start_time) < {duration_per_direction}:
    pub.publish(cmd)
    rate.sleep()

# Stop
print("모터 정지...")
cmd.motor0_output = 0.0
cmd.motor1_output = 0.0
for i in range(5):
    pub.publish(cmd)
    rate.sleep()
'''
        
        # 스크립트 업로드
        write_cmd = f"cat > /tmp/motor_both_directions_test.py << 'ENDOFFILE'\n{motor_script}\nENDOFFILE"
        success, _, _ = run_command(client, write_cmd, timeout=5)
        
        if not success:
            print("❌ Failed to create motor script")
            return 1
        
        chmod_cmd = "chmod +x /tmp/motor_both_directions_test.py"
        run_command(client, chmod_cmd, timeout=2)
        
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
        
        # 모터 실행 시작 (백그라운드)
        print("\n모터 시작 (정방향 -> 역방향)...")
        motor_cmd = f"{env_cmd} && python2 /tmp/motor_both_directions_test.py > /tmp/motor_both_directions_test.log 2>&1 &"
        client.exec_command(motor_cmd, timeout=2)
        time.sleep(1)
        
        print(f"✅ 모터 명령 시작")
        print()
        print("="*80)
        print("홀 센서 실시간 모니터링 (정방향 -> 역방향)")
        print("="*80)
        print()
        print("Time(s)  Motor0_Count  Motor1_Count  M0_Change  M1_Change  Direction")
        print("-" * 80)
        
        start_time = time.time()
        prev_data = {}
        sample_count = 0
        current_message = []
        count_changes = []
        direction_changes = []  # 방향 변화 기록
        
        def process_line(line):
            nonlocal prev_data, sample_count, current_message, count_changes, direction_changes
            
            line = line.strip()
            if not line:
                return
            
            if line.startswith('---'):
                if current_message:
                    message_text = '\n'.join(current_message)
                    
                    data = {}
                    match = re.search(r'motor0_hall_count:\s*(-?\d+)', message_text)
                    if match:
                        data['motor0_count'] = int(match.group(1))
                    
                    match = re.search(r'motor1_hall_count:\s*(-?\d+)', message_text)
                    if match:
                        data['motor1_count'] = int(match.group(1))
                    
                    if data:
                        sample_count += 1
                        elapsed = time.time() - start_time
                        
                        # 카운트 변화량 계산
                        change_0 = ""
                        change_1 = ""
                        direction = ""
                        
                        if 'motor0_count' in data:
                            if 'motor0_count' in prev_data:
                                diff = data['motor0_count'] - prev_data['motor0_count']
                                if diff != 0:
                                    count_changes.append(('M0', elapsed, diff))
                                    change_0 = f"+{diff}" if diff > 0 else str(diff)
                                    direction += "M0:F" if diff > 0 else "M0:R"
                            
                            if 'motor1_count' in data:
                                if 'motor1_count' in prev_data:
                                    diff = data['motor1_count'] - prev_data['motor1_count']
                                    if diff != 0:
                                        count_changes.append(('M1', elapsed, diff))
                                        change_1 = f"+{diff}" if diff > 0 else str(diff)
                                        direction += " M1:F" if diff > 0 else " M1:R"
                        
                        # 변화가 있을 때만 표시
                        if change_0 or change_1:
                            m0_count = data.get('motor0_count', 'N/A')
                            m1_count = data.get('motor1_count', 'N/A')
                            change_0_str = change_0 if change_0 else "-"
                            change_1_str = change_1 if change_1 else "-"
                            direction_str = direction if direction else "-"
                            
                            # 방향 표시 (정방향/역방향 구간)
                            phase = "정방향" if elapsed < duration_per_direction else "역방향"
                            print(f"{elapsed:7.2f} [{phase:4s}] {str(m0_count):13s}  {str(m1_count):13s}  {change_0_str:9s}  {change_1_str:9s}  {direction_str}")
                        
                        prev_data.update(data)
                
                current_message = []
            else:
                current_message.append(line)
        
        # 홀 센서 읽기 시작
        print("rostopic echo 시작...")
        total_duration = duration_per_direction * 2 + 3
        cmd = f"{env_cmd} && timeout {total_duration} rostopic echo /motor_status 2>&1"
        stdin, stdout, stderr = client.exec_command(cmd, timeout=total_duration + 8)
        time.sleep(1)
        
        start_time = time.time()
        last_print_time = 0
        last_data_time = start_time
        max_duration = total_duration + 6
        no_data_warning = False
        
        print("데이터 수신 대기 중...")
        
        try:
            while running and time.time() - start_time < max_duration:
                current_time = time.time() - start_time
                
                # 타임아웃 체크
                if current_time >= max_duration:
                    print(f"\n   [Timeout reached: {max_duration:.1f}s]")
                    break
                
                # 데이터 수신 확인
                data_received = False
                if stdout.channel.recv_ready():
                    try:
                        chunk = stdout.channel.recv(4096).decode('utf-8', errors='ignore')
                        if chunk:
                            data_received = True
                            last_data_time = time.time()
                            no_data_warning = False
                            lines = chunk.split('\n')
                            for line in lines:
                                process_line(line)
                    except Exception as e:
                        print(f"\n   [Read error: {e}]")
                
                # 주기적으로 진행 상황 출력
                if current_time - last_print_time >= 3.0:
                    phase = "정방향" if current_time < duration_per_direction else "역방향"
                    if current_time < total_duration - 3:
                        status = f"[{phase} 구간: {current_time:.1f}s / {total_duration}s]"
                        if sample_count > 0:
                            status += f" (샘플: {sample_count})"
                        print(f"\n   {status}")
                    else:
                        print(f"\n   [모터 정지: {current_time:.1f}s] (샘플: {sample_count})")
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
        
        # 결과 분석
        print("\n" + "="*80)
        print("결과 분석")
        print("="*80)
        
        print(f"\n📊 총 샘플 수: {sample_count}")
        
        if sample_count == 0:
            print("\n⚠️  경고: 데이터를 수신하지 못했습니다!")
            return 1
        
        # 정방향/역방향 구간별 분석
        forward_changes = [s for s in count_changes if s[1] < duration_per_direction]
        reverse_changes = [s for s in count_changes if s[1] >= duration_per_direction]
        
        print(f"\n📈 정방향 구간 (0 ~ {duration_per_direction}초):")
        if forward_changes:
            m0_forward = [s for s in forward_changes if s[0] == 'M0']
            m1_forward = [s for s in forward_changes if s[0] == 'M1']
            
            if m0_forward:
                change_values = [s[2] for s in m0_forward]
                positive = [v for v in change_values if v > 0]
                negative = [v for v in change_values if v < 0]
                print(f"   Motor0: 증가 {len(positive)}회, 감소 {len(negative)}회")
                print(f"   ✅ 정방향인데 증가: {'있음 (정상)' if positive else '없음 (문제)'}")
                print(f"   ⚠️  정방향인데 감소: {'있음 (문제)' if negative else '없음 (정상)'}")
            
            if m1_forward:
                change_values = [s[2] for s in m1_forward]
                positive = [v for v in change_values if v > 0]
                negative = [v for v in change_values if v < 0]
                print(f"   Motor1: 증가 {len(positive)}회, 감소 {len(negative)}회")
                print(f"   ✅ 정방향인데 증가: {'있음 (정상)' if positive else '없음 (문제)'}")
                print(f"   ⚠️  정방향인데 감소: {'있음 (문제)' if negative else '없음 (정상)'}")
        else:
            print("   ⚠️  변화 없음")
        
        print(f"\n📈 역방향 구간 ({duration_per_direction} ~ {duration_per_direction * 2}초):")
        if reverse_changes:
            m0_reverse = [s for s in reverse_changes if s[0] == 'M0']
            m1_reverse = [s for s in reverse_changes if s[0] == 'M1']
            
            if m0_reverse:
                change_values = [s[2] for s in m0_reverse]
                positive = [v for v in change_values if v > 0]
                negative = [v for v in change_values if v < 0]
                print(f"   Motor0: 증가 {len(positive)}회, 감소 {len(negative)}회")
                print(f"   ✅ 역방향인데 감소: {'있음 (정상)' if negative else '없음 (문제)'}")
                print(f"   ⚠️  역방향인데 증가: {'있음 (문제)' if positive else '없음 (정상)'}")
            
            if m1_reverse:
                change_values = [s[2] for s in m1_reverse]
                positive = [v for v in change_values if v > 0]
                negative = [v for v in change_values if v < 0]
                print(f"   Motor1: 증가 {len(positive)}회, 감소 {len(negative)}회")
                print(f"   ✅ 역방향인데 감소: {'있음 (정상)' if negative else '없음 (문제)'}")
                print(f"   ⚠️  역방향인데 증가: {'있음 (문제)' if positive else '없음 (정상)'}")
        else:
            print("   ⚠️  변화 없음")
        
        # 최종 상태
        print(f"\n📈 최종 상태:")
        if prev_data:
            print(f"   Motor0: Count={prev_data.get('motor0_count', 'N/A')}")
            print(f"   Motor1: Count={prev_data.get('motor1_count', 'N/A')}")
        
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
            run_command(client, stop_cmd, timeout=2)
        except:
            pass
        
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


