# 컨트롤러 상태 메시지
# 밸런스 컨트롤러 노드에서 발행

Header header

# 제어 모드
uint8 DIRECT_MODE = 0
uint8 PID_MODE = 1
uint8 BALANCE_MODE = 2
uint8 current_control_mode

# 목표 속도
float32 target_speed_0
float32 target_speed_1

# 실제 속도
float32 actual_speed_0
float32 actual_speed_1

# PID 출력
float32 pid_output_0
float32 pid_output_1
float32 balance_output

# PID 오차
float32 pid_error_0
float32 pid_error_1
float32 balance_error

# 목표 각도와 현재 각도
float32 target_roll_angle
float32 current_roll_angle

# 제어 상태 플래그
bool use_pid_control
bool use_balance_control
bool use_smooth_control
bool emergency_stop_active

# 제어 파라미터
float32 motor_deadzone
float32 deadzone_compensation
float32 brake_zone_threshold
float32 brake_duration

# 시스템 상태
bool system_ready
bool sensors_ok
bool motors_ok
uint32 control_loop_frequency 