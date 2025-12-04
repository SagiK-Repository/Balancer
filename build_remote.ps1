# 원격 서버 빌드 스크립트
$hostname = "tbot3@192.168.0.28"
$password = "1234"

Write-Host "Connecting to $hostname..."

# SSH 연결 및 빌드 명령 실행
$commands = @"
cd ~/catkin_ws || { echo 'catkin_ws not found, creating...'; mkdir -p ~/catkin_ws/src; }
cd ~/catkin_ws/src
if [ ! -d 'Balancer' ]; then
    echo 'Balancer project not found in ~/catkin_ws/src'
    echo 'Please copy the project first using:'
    echo '  scp -r /path/to/Balancer/src tbot3@192.168.0.28:~/catkin_ws/src/'
    exit 1
fi
cd ~/catkin_ws
source /opt/ros/noetic/setup.bash 2>/dev/null || source /opt/ros/melodic/setup.bash 2>/dev/null || echo 'ROS setup not found'
catkin_make
"@

# plink를 사용하거나, expect 스크립트를 사용해야 합니다
# Windows에서는 직접 비밀번호를 전달하기 어려우므로
# 사용자에게 수동으로 입력하도록 안내하거나
# SSH 키를 설정하는 것이 좋습니다

Write-Host "Please run the following commands manually:"
Write-Host "1. Copy project: scp -r src/integrated_balancer tbot3@192.168.0.28:~/catkin_ws/src/"
Write-Host "2. SSH and build: ssh tbot3@192.168.0.28"
Write-Host "   Then run: cd ~/catkin_ws && catkin_make"



