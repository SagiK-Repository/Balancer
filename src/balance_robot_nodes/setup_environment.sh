#!/bin/bash
# 밸런싱 로봇 환경 설정 스크립트

echo "🚀 밸런싱 로봇 환경 설정 중..."

# ROS 환경 설정
source /opt/ros/noetic/setup.bash
source ~/catkin_ws/devel/setup.bash

# 환경 변수 설정
export ROS_MASTER_URI=http://localhost:11311
export ROS_HOSTNAME=localhost

# 패키지 등록 확인
if rospack find balance_robot_nodes > /dev/null 2>&1; then
    echo "✅ balance_robot_nodes 패키지가 성공적으로 등록되었습니다!"
    echo "📍 패키지 경로: $(rospack find balance_robot_nodes)"
else
    echo "❌ 패키지 등록 실패. catkin_make를 다시 실행해주세요."
    exit 1
fi

# 사용 가능한 런치 파일 표시
echo ""
echo "🚀 사용 가능한 런치 파일:"
echo "  roslaunch balance_robot_nodes balance_robot.launch"
echo "  roslaunch balance_robot_nodes test_hardware_only.launch"
echo "  roslaunch balance_robot_nodes test_sensors_only.launch"

echo ""
echo "🎉 환경 설정 완료!" 