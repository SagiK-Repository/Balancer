#!/bin/bash
# 원격 서버 빌드 스크립트

HOST="tbot3@192.168.0.28"
PASSWORD="1234"
PROJECT_DIR="src/integrated_balancer"
REMOTE_DIR="~/catkin_ws/src/integrated_balancer"

echo "Copying project to remote server..."

# expect를 사용하여 비밀번호 자동 입력
expect << EOF
set timeout 30
spawn scp -r $PROJECT_DIR $HOST:$REMOTE_DIR
expect {
    "password:" {
        send "$PASSWORD\r"
        exp_continue
    }
    "Permission denied" {
        puts "Permission denied. Please check password."
        exit 1
    }
    eof
}
EOF

if [ $? -eq 0 ]; then
    echo "Project copied successfully!"
    echo "Building on remote server..."
    
    # 빌드 명령 실행
    expect << EOF
set timeout 300
spawn ssh $HOST "cd ~/catkin_ws && source /opt/ros/noetic/setup.bash 2>/dev/null || source /opt/ros/melodic/setup.bash 2>/dev/null && catkin_make"
expect {
    "password:" {
        send "$PASSWORD\r"
        exp_continue
    }
    "Permission denied" {
        puts "Permission denied. Please check password."
        exit 1
    }
    eof
}
EOF
else
    echo "Failed to copy project."
    exit 1
fi

echo "Build completed!"



