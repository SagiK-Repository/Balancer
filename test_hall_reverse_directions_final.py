#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
모터 방향 반전 테스트 (Motor0: 역방향, Motor1: 정방향) - 최종 버전
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
    print("모터 방향 반전 테스트 (Motor0: 역방향, Motor1: 정방향) - 최종 버전")
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
        else:
            print("   ⚠️  roscore가 시작되지 않았습니다. 계속 진행합니다...")
        
        # ROS 마스터 연결 확인
        print("ROS 마스터 연결 확인...")
        for i in range(5):
            cmd = f"{env_cmd} && timeout 2 rostopic list > /dev/null 2>&1"
            success, output, error = run_command(client, cmd, timeout=3)
            if success:
                print("   ✅ ROS 마스터 연결 성공")
                break
            time.sleep(1)
        else:
            print("   ⚠️  ROS 마스터 연결 실패. 계속 진행합니다...")
        
        # 모터 속도 설정
        speed = 30.0
        duration = 15  # 15초
        
        print(f"모터 설정:")
        print(f"  Motor0: {speed}% (역방향)")
        print(f"  Motor1: {speed}% (정방향)")
        print(f"  시간: {duration}초")
        print()
        
        # 모터 명령 발행 스크립트 작성
        motor_script = f'''#!/usr/bin/env python2
# -*- coding: utf-8 -*-
import rospy
from balance_robot_nodes.msg import MotorCommand
import sys
import time

rospy.init_node('motor_reverse_test', anonymous=True)
pub = rospy.Publisher('/motor_command', MotorCommand, queue_size=10)
rospy.sleep(1)

cmd = MotorCommand()
cmd.motor0_output = {speed}
cmd.motor1_output = {speed}
cmd.motor0_direction = False  # Motor0: reverse
cmd.motor1_direction = True   # Motor1: forward
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
        write_cmd = f"cat > /tmp/motor_reverse_test.py << 'ENDOFFILE'\n{motor_script}\nENDOFFILE"
        success, _, _ = run_command(client, write_cmd, timeout=5)
        
        if not success:
            print("❌ Failed to create motor script")
            return 1
        
        chmod_cmd = "chmod +x /tmp/motor_reverse_test.py"
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
        else:
            print("   ⚠️  spi_hardware_node가 시작되지 않았습니다.")
            print("   로그 확인:")
            success, output, error = run_command(client, "tail -20 /tmp/spi_node.log 2>/dev/null || echo '로그 없음'", timeout=3)
            if success:
                print(f"   {output}")
        
        # 토픽 발행 확인
        print("토픽 발행 확인...")
        time.sleep(2)
        cmd = f"{env_cmd} && timeout 3 rostopic list | grep motor_status"
        success, output, error = run_command(client, cmd, timeout=5)
        if success and 'motor_status' in output:
            print("   ✅ /motor_status 토픽 존재")
        else:
            print("   ⚠️  /motor_status 토픽이 아직 없습니다. 계속 진행합니다...")
        
        # 모터 실행 시작 (백그라운드)
        print("모터 시작 (방향 반전)...")
        motor_cmd = f"{env_cmd} && python2 /tmp/motor_reverse_test.py > /tmp/motor_reverse_test.log 2>&1 &"
        client.exec_command(motor_cmd, timeout=2)
        time.sleep(1)
        
        print(f"✅ 모터 명령 시작 (Motor0: 역방향, Motor1: 정방향, {duration}초)")
        print()
        print("="*80)
        print("홀 센서 실시간 모니터링 (방향 반전 확인)")
        print("="*80)
        print()
        print("Time(s)  Motor0_Count  Motor1_Count  M0_Change  M1_Change  Direction")
        print("-" * 80)
        
        start_time = time.time()
        prev_data = {}
        sample_count = 0
        current_message = []
        count_changes = []
        
        def process_line(line):
            nonlocal prev_data, sample_count, current_message, count_changes
            
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
                            
                            print(f"{elapsed:7.2f}    {str(m0_count):13s}  {str(m1_count):13s}  {change_0_str:9s}  {change_1_str:9s}  {direction_str}")
                        
                        prev_data.update(data)
                
                current_message = []
            else:
                current_message.append(line)
        
        # 홀 센서 읽기 시작
        print("rostopic echo 시작...")
        cmd = f"{env_cmd} && timeout {duration + 3} rostopic echo /motor_status 2>&1"
        stdin, stdout, stderr = client.exec_command(cmd, timeout=duration + 8)
        time.sleep(1)  # 초기화 대기
        
        start_time = time.time()
        last_print_time = 0
        last_data_time = start_time
        max_duration = duration + 6  # 최대 실행 시간
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
                
                # 에러 메시지 확인
                if stderr.channel.recv_stderr_ready():
                    try:
                        error_chunk = stderr.channel.recv_stderr(4096).decode('utf-8', errors='ignore')
                        if error_chunk.strip():
                            print(f"\n   [Error: {error_chunk.strip()}]")
                    except:
                        pass
                
                # 데이터가 없을 때 경고
                if not data_received and current_time > 3.0:
                    if time.time() - last_data_time > 5.0 and not no_data_warning:
                        print(f"\n   ⚠️  [경고: {current_time:.1f}s 동안 데이터 수신 없음]")
                        print(f"   [토픽 확인 중...]")
                        # 토픽 확인
                        check_cmd = f"{env_cmd} && timeout 2 rostopic list | grep motor_status"
                        check_success, check_output, _ = run_command(client, check_cmd, timeout=3)
                        if check_success and 'motor_status' in check_output:
                            print(f"   ✅ 토픽 존재함")
                        else:
                            print(f"   ❌ 토픽이 없거나 접근 불가")
                        no_data_warning = True
                
                # 주기적으로 진행 상황 출력
                if current_time - last_print_time >= 3.0:
                    if current_time < duration:
                        status = f"[Motor Running: {current_time:.1f}s / {duration}s]"
                        if sample_count > 0:
                            status += f" (샘플: {sample_count})"
                        print(f"\n   {status}")
                    else:
                        print(f"\n   [Motor Stopped: {current_time:.1f}s] (샘플: {sample_count})")
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
            # 명시적으로 채널 닫기
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
            print("   가능한 원인:")
            print("   1. spi_hardware_node가 실행되지 않음")
            print("   2. /motor_status 토픽이 발행되지 않음")
            print("   3. 네트워크 연결 문제")
            print("\n   로그 확인:")
            print("   - /tmp/spi_node.log")
            print("   - /tmp/motor_reverse_test.log")
            return 1
        
        # 카운트 변화량 통계
        if count_changes:
            m0_changes = [s for s in count_changes if s[0] == 'M0']
            m1_changes = [s for s in count_changes if s[0] == 'M1']
            
            print(f"\n📈 Motor0 (역방향) 카운트 변화:")
            if m0_changes:
                change_values = [s[2] for s in m0_changes]
                positive = [v for v in change_values if v > 0]
                negative = [v for v in change_values if v < 0]
                print(f"   총 변화 횟수: {len(m0_changes)}")
                print(f"   증가: {len(positive)}회, 감소: {len(negative)}회")
                if positive:
                    print(f"   증가 범위: {min(positive)} ~ {max(positive)}")
                if negative:
                    print(f"   감소 범위: {min(negative)} ~ {max(negative)}")
                print(f"   ✅ 역방향인데 감소가 발생: {'있음 (정상)' if negative else '없음 (문제)'}")
                print(f"   ⚠️  역방향인데 증가가 발생: {'있음 (문제)' if positive else '없음 (정상)'}")
            else:
                print("   ⚠️  변화 없음")
            
            print(f"\n📈 Motor1 (정방향) 카운트 변화:")
            if m1_changes:
                change_values = [s[2] for s in m1_changes]
                positive = [v for v in change_values if v > 0]
                negative = [v for v in change_values if v < 0]
                print(f"   총 변화 횟수: {len(m1_changes)}")
                print(f"   증가: {len(positive)}회, 감소: {len(negative)}회")
                if positive:
                    print(f"   증가 범위: {min(positive)} ~ {max(positive)}")
                if negative:
                    print(f"   감소 범위: {min(negative)} ~ {max(negative)}")
                print(f"   ✅ 정방향인데 증가가 발생: {'있음 (정상)' if positive else '없음 (문제)'}")
                print(f"   ⚠️  정방향인데 감소가 발생: {'있음 (문제)' if negative else '없음 (정상)'}")
            else:
                print("   ⚠️  변화 없음")
        
        # 최종 상태
        print(f"\n📈 최종 상태:")
        if prev_data:
            print(f"   Motor0: Count={prev_data.get('motor0_count', 'N/A')}")
            print(f"   Motor1: Count={prev_data.get('motor1_count', 'N/A')}")
        
        # 종합 판단
        print(f"\n✅ 종합 판단:")
        if count_changes:
            m0_changes = [s for s in count_changes if s[0] == 'M0']
            m1_changes = [s for s in count_changes if s[0] == 'M1']
            
            m0_negative = any(s[2] < 0 for s in m0_changes)
            m0_positive = any(s[2] > 0 for s in m0_changes)
            m1_negative = any(s[2] < 0 for s in m1_changes)
            m1_positive = any(s[2] > 0 for s in m1_changes)
            
            print(f"\n   현재 테스트 (Motor0: 역방향, Motor1: 정방향):")
            if m0_negative:
                print(f"     ✅ Motor0 (역방향)에서 감소 발생 - 정상")
            else:
                print(f"     ⚠️  Motor0 (역방향)에서 감소 없음 - 문제")
            
            if m0_positive:
                print(f"     ⚠️  Motor0 (역방향)에서 증가 발생 - 문제")
            else:
                print(f"     ✅ Motor0 (역방향)에서 증가 없음 - 정상")
            
            if m1_positive:
                print(f"     ✅ Motor1 (정방향)에서 증가 발생 - 정상")
            else:
                print(f"     ⚠️  Motor1 (정방향)에서 증가 없음 - 문제")
            
            if m1_negative:
                print(f"     ⚠️  Motor1 (정방향)에서 감소 발생 - 문제")
            else:
                print(f"     ✅ Motor1 (정방향)에서 감소 없음 - 정상")
            
            # 방향 반전 확인
            print(f"\n   🔄 방향 반전 확인:")
            if m0_negative and not m0_positive and m1_positive and not m1_negative:
                print(f"     ✅ 완벽하게 반전됨! Motor0는 감소만, Motor1는 증가만 발생")
            elif m0_negative and m1_positive:
                print(f"     ⚠️  부분적으로 반전됨. Motor0는 감소 발생, Motor1는 증가 발생")
                if m0_positive:
                    print(f"        하지만 Motor0에서 증가도 발생 (문제)")
                if m1_negative:
                    print(f"        하지만 Motor1에서 감소도 발생 (문제)")
            else:
                print(f"     ❌ 반전이 제대로 작동하지 않음")
        
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

