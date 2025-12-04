#!/bin/bash
# ROS에서 홀 센서 데이터 확인 스크립트

HOSTNAME="192.168.0.28"
USERNAME="tbot3"
PASSWORD="1234"

echo "=========================================="
echo "ROS 홀 센서 데이터 확인"
echo "=========================================="
echo ""

# SSH로 접속하여 ROS 명령 실행
sshpass -p "$PASSWORD" ssh -o StrictHostKeyChecking=no $USERNAME@$HOSTNAME << 'EOF'
    # 현재 IP 확인
    CURRENT_IP=$(hostname -I | awk '{print $1}')
    
    # ROS 환경 설정
    export ROS_MASTER_URI="http://${CURRENT_IP}:11311"
    export ROS_IP="${CURRENT_IP}"
    source ~/catkin_ws/devel/setup.bash
    
    echo "1. 사용 가능한 토픽 목록:"
    echo "----------------------------------------"
    rostopic list | grep -E "(motor|hall)" || rostopic list
    echo ""
    
    echo "2. /motor_status 토픽 타입 확인:"
    echo "----------------------------------------"
    rostopic type /motor_status 2>/dev/null || echo "토픽이 없습니다."
    echo ""
    
    echo "3. /motor_status 토픽 발행 주기:"
    echo "----------------------------------------"
    timeout 3 rostopic hz /motor_status 2>&1 | head -5 || echo "토픽이 발행되지 않습니다."
    echo ""
    
    echo "4. 홀 센서 데이터 실시간 확인 (5초간):"
    echo "----------------------------------------"
    echo "Ctrl+C를 눌러 종료할 수 있습니다."
    echo ""
    timeout 5 rostopic echo /motor_status 2>/dev/null | grep -E "(motor0_hall|motor1_hall)" || echo "데이터를 받을 수 없습니다."
EOF

echo ""
echo "=========================================="
echo "완료"
echo "=========================================="


