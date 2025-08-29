# 모터 제어 명령 메시지
# 컨트롤러 노드에서 SPI 하드웨어 노드로 전송

Header header

# 모터 출력 값 (-100.0 ~ 100.0)
float32 motor0_output
float32 motor1_output

# 모터 방향 (true: 정방향, false: 역방향)
bool motor0_direction
bool motor1_direction

# 제어 모드
uint8 DIRECT_MODE = 0
uint8 PID_MODE = 1
uint8 BALANCE_MODE = 2
uint8 control_mode

# 비상 정지 플래그
bool emergency_stop 