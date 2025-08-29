# 🚗 모터 제어 사용법

## 📋 개요
이 패키지는 **모터0과 모터1을 독립적으로 제어**할 수 있는 다양한 방법을 제공합니다.

## 🛠️ 컴파일 및 실행

### 1. 컴파일
```bash
cd ~/catkin_ws
catkin_make
source devel/setup.bash
```
```bash
# 개별 전송
scp -r catkin_ws/src/motor_controller/src tbot3@192.168.0.28:~/catkin_ws/src/motor_controller/
scp -r catkin_ws/src/motor_controller/include tbot3@192.168.0.28:~/catkin_ws/src/motor_controller/
scp -r catkin_ws/src/motor_controller/launch tbot3@192.168.0.28:~/catkin_ws/src/motor_controller/
scp -r catkin_ws/src/motor_controller/scripts tbot3@192.168.0.28:~/catkin_ws/src/motor_controller/

scp catkin_ws/src/motor_controller/CMakeLists.txt tbot3@192.168.0.28:~/catkin_ws/src/motor_controller/
scp catkin_ws/src/motor_controller/package.xml tbot3@192.168.0.28:~/catkin_ws/src/motor_controller/

# 방법 5: rsync 사용 (변경된 파일만 동기화)
rsync -avz --include='src/' --include='include/' --include='launch/' --include='scripts/' --include='CMakeLists.txt' --include='package.xml' --exclude='*' catkin_ws/src/motor_controller/ tbot3@192.168.0.28:~/catkin_ws/src/motor_controller/
```

### 2. 모터 컨트롤러 실행
```powershell
ssh tbot3@192.168.0.28
```
```bash
cd ~/catkin_ws
source devel/setup.bash
roslaunch motor_controller motor_controller.launch
```

## 🎮 모터 제어 방법

### 방법 1: 포괄적인 두 바퀴 테스트 (추천) 🚗
```bash
# 새 터미널에서
rosrun motor_controller dual_motor_test
```

**테스트 메뉴:**
1. **기본 테스트** - 각 모터 개별 테스트
2. **동시 테스트** - 두 모터 동일 속도 테스트
3. **방향 테스트** - 전진/후진/회전 테스트
4. **부드러운 제어 테스트** - 점진적 가속/감속
5. **속도 단계별 테스트** - 다양한 속도 레벨
6. **수동 제어 모드** - 실시간 두 모터 제어
7. **상태 확인** - 현재 모터 상태 확인

### 방법 2: 자동화된 테스트 스크립트 🤖
```bash
# 전체 테스트 스위트 (모든 테스트 자동 실행)
python3 ~/catkin_ws/src/motor_controller/scripts/auto_dual_test.py

# 특정 테스트만 실행
python3 ~/catkin_ws/src/motor_controller/scripts/auto_dual_test.py individual
python3 ~/catkin_ws/src/motor_controller/scripts/auto_dual_test.py smooth
python3 ~/catkin_ws/src/motor_controller/scripts/auto_dual_test.py direction
```

**자동 테스트 타입:**
- `individual` - 개별 모터 테스트
- `simultaneous` - 동시 모터 테스트  
- `direction` - 방향 제어 테스트
- `smooth` - 부드러운 제어 테스트
- `steps` - 속도 단계별 테스트
- `differential` - 차동 제어 테스트
- `full` - 전체 테스트 스위트 (기본값)

### 방법 3: 간단한 대화형 제어
```bash
# 새 터미널에서
rosrun motor_controller test_motor_control
```

**사용법:**
- `0 [속도]` - 모터0 속도 설정 (예: `0 10.5`)
- `1 [속도]` - 모터1 속도 설정 (예: `1 -5.2`)
- `both [속도]` - 두 모터 동일 속도 (예: `both 15.0`)
- `stop` - 모든 모터 정지
- `help` - 도움말 표시
- `quit` - 프로그램 종료

### 방법 4: Python 스크립트 (한 번 실행)
```bash
# 모터0을 10.5로 설정
python3 ~/catkin_ws/src/motor_controller/scripts/motor_control.py 0 10.5

# 모터1을 -5.2로 설정
python3 ~/catkin_ws/src/motor_controller/scripts/motor_control.py 1 -5.2

# 두 모터를 15.0으로 설정
python3 ~/catkin_ws/src/motor_controller/scripts/motor_control.py both 15.0

# 모든 모터 정지
python3 ~/catkin_ws/src/motor_controller/scripts/motor_control.py stop
```

### 방법 5: ROS 토픽 직접 발행
```bash
# 모터0 속도 설정
rostopic pub /target_speed0 std_msgs/Float32 "data: 10.5"

# 모터1 속도 설정
rostopic pub /target_speed1 std_msgs/Float32 "data: -5.2"

# 모든 모터 정지
rostopic pub /target_speed0 std_msgs/Float32 "data: 0.0"
rostopic pub /target_speed1 std_msgs/Float32 "data: 0.0"
```

### 방법 6: 하드웨어 버튼
- **증가 버튼**: 두 모터 속도를 +5씩 증가
- **감소 버튼**: 두 모터 속도를 -5씩 감소

## 📊 모터 상태 모니터링

### 실시간 속도 확인
```bash
# 모터0 실제 속도
rostopic echo /actual_speed0

# 모터1 실제 속도
rostopic echo /actual_speed1

# 센서 데이터
rostopic echo /sensor_roll
rostopic echo /sensor_pitch
rostopic echo /sensor_confidence
```

### RQT 플롯으로 시각화
```bash
rqt_plot /actual_speed0/data /actual_speed1/data
```

## ⚙️ 부드러운 제어 설정

현재 설정 (launch 파일):
- **최대 가속도**: 10%/초
- **최대 감속도**: 20%/초
- **속도 필터**: 0.3 (부드러움)
- **출력 필터**: 0.5 (안정성)

### 설정 변경
`catkin_ws/src/motor_controller/launch/motor_controller.launch` 파일에서:
```xml
<param name="smooth_max_acceleration" value="10.0"/>  <!-- 가속도 조정 -->
<param name="smooth_max_deceleration" value="20.0"/>  <!-- 감속도 조정 -->
<param name="smooth_velocity_filter" value="0.3"/>    <!-- 속도 필터 -->
<param name="smooth_output_filter" value="0.5"/>      <!-- 출력 필터 -->
```

## 🔧 제어 모드

### 현재 설정
- **PID 제어**: 비활성화 (`use_pid_control: false`)
- **밸런스 제어**: 비활성화 (`use_balance_control: false`)
- **부드러운 제어**: 활성화 (`use_smooth_control: true`)

### 모드 변경
launch 파일에서 다음 매개변수를 수정:
```xml
<param name="use_pid_control" value="false"/>     <!-- 직접 제어 -->
<param name="use_balance_control" value="false"/>  <!-- 밸런스 제어 -->
<param name="use_smooth_control" value="true"/>   <!-- 부드러운 제어 -->
```

## 🚨 안전 설정

### 속도 제한
- **최대 속도**: 20.0 (launch 파일에서 설정)
- **범위**: -20.0 ~ +20.0

### 긴급 정지
```bash
# 즉시 모든 모터 정지
python3 ~/catkin_ws/src/motor_controller/scripts/motor_control.py stop
```

## 📈 테스트 시나리오

### 기본 테스트
```bash
# 1. 모터0만 테스트
python3 ~/catkin_ws/src/motor_controller/scripts/motor_control.py 0 5.0
sleep 3
python3 ~/catkin_ws/src/motor_controller/scripts/motor_control.py 0 0.0

# 2. 모터1만 테스트
python3 ~/catkin_ws/src/motor_controller/scripts/motor_control.py 1 5.0
sleep 3
python3 ~/catkin_ws/src/motor_controller/scripts/motor_control.py 1 0.0

# 3. 두 모터 동시 테스트
python3 ~/catkin_ws/src/motor_controller/scripts/motor_control.py both 5.0
sleep 3
python3 ~/catkin_ws/src/motor_controller/scripts/motor_control.py stop
```

### 부드러운 제어 테스트
```bash
# 포괄적인 두 바퀴 테스트 (추천)
rosrun motor_controller dual_motor_test
# 메뉴에서 4번 선택 (부드러운 제어 테스트)

# 자동화된 부드러운 제어 테스트
python3 ~/catkin_ws/src/motor_controller/scripts/auto_dual_test.py smooth

# 대화형 모드로 점진적 테스트
rosrun motor_controller test_motor_control
# 명령 예시:
# 0 2.0    (모터0을 2.0으로)
# 0 5.0    (모터0을 5.0으로 - 부드럽게 증가)
# 0 0.0    (모터0을 0.0으로 - 부드럽게 감소)
```

### 두 바퀴 차동 제어 테스트
```bash
# 차동 제어 전용 테스트
python3 ~/catkin_ws/src/motor_controller/scripts/auto_dual_test.py differential

# 포괄적인 테스트에서 수동 제어 모드
rosrun motor_controller dual_motor_test
# 메뉴에서 6번 선택 후:
# 5.0 3.0    (모터0=5.0, 모터1=3.0 - 우회전)
# 3.0 5.0    (모터0=3.0, 모터1=5.0 - 좌회전)
# 8.0 -3.0   (모터0 전진, 모터1 후진 - 제자리 회전)
```

## 🔍 문제 해결

### 센서 신뢰도 0% 문제
- **원인**: 급격한 모터 변화로 인한 진동
- **해결**: 부드러운 제어 활성화 및 가속도 제한 강화

### 모터 응답 없음
1. **하드웨어 연결 확인**
2. **GPIO 권한 확인**
3. **ROS 토픽 확인**: `rostopic list`
4. **로그 확인**: `rosnode info motor_controller`

### 컴파일 오류
```bash
cd ~/catkin_ws
catkin_make clean
catkin_make
```

## 📝 로그 분석

모터 컨트롤러는 다음 정보를 실시간으로 출력합니다:
- **센서 각도**: Roll/Pitch 각도
- **센서 신뢰도**: 0-100% 신뢰도
- **목표/실제 속도**: 각 모터의 목표 및 실제 속도
- **모터 출력**: 실제 모터 출력값
- **온도**: 센서 온도

이 정보를 통해 부드러운 제어의 효과를 실시간으로 확인할 수 있습니다. 