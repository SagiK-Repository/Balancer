# IMU 센서 데이터 메시지
# I2C 센서 노드에서 컨트롤러 노드로 전송

Header header

# 가속도계 데이터 (m/s²)
float32 accel_x
float32 accel_y
float32 accel_z

# 자이로스코프 데이터 (rad/s)
float32 gyro_x
float32 gyro_y
float32 gyro_z

# 자력계 데이터 (uT)
float32 mag_x
float32 mag_y
float32 mag_z

# 계산된 각도 (도)
float32 roll_angle          # 즉시 응답 각도
float32 pitch_angle
float32 yaw_angle

float32 stable_roll_angle   # 안정화된 각도
float32 stable_pitch_angle

# 센서 상태
bool is_stable              # 센서 안정 상태
float32 tilt_confidence     # 기울기 신뢰도 (0-100%)
float32 temperature         # 온도 (℃)

# I2C 통신 상태
bool i2c_communication_ok
uint8 sensor_error_code
string sensor_error_message 