# 모터 상태 메시지
# SPI 하드웨어 노드에서 컨트롤러 노드로 전송

Header header

# 모터 속도 (RPM)
float32 motor0_speed
float32 motor1_speed

# 홀센서 상태 (0~7)
uint8 motor0_hall_state
uint8 motor1_hall_state

# 홀센서 틱 카운트
int32 motor0_hall_count
int32 motor1_hall_count

# 모터 출력 상태
float32 motor0_actual_output
float32 motor1_actual_output

# 하드웨어 상태
bool spi_communication_ok
bool dac0_ok
bool dac1_ok
bool gpio_ok

# 오류 코드
uint8 ERROR_NONE = 0
uint8 ERROR_SPI = 1
uint8 ERROR_DAC = 2
uint8 ERROR_GPIO = 3
uint8 error_code

string error_message 