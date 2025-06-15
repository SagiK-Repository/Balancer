#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
from std_msgs.msg import Float32
import sys

class MotorController:
    def __init__(self):
        rospy.init_node('motor_control_script', anonymous=True)
        
        self.motor0_pub = rospy.Publisher('target_speed0', Float32, queue_size=10)
        self.motor1_pub = rospy.Publisher('target_speed1', Float32, queue_size=10)
        
        rospy.sleep(0.5)  # 퍼블리셔 초기화 대기
        
    def set_motor0_speed(self, speed):
        """모터0 속도 설정"""
        msg = Float32()
        msg.data = float(speed)
        self.motor0_pub.publish(msg)
        rospy.loginfo(f"모터0 속도 설정: {speed}")
        
    def set_motor1_speed(self, speed):
        """모터1 속도 설정"""
        msg = Float32()
        msg.data = float(speed)
        self.motor1_pub.publish(msg)
        rospy.loginfo(f"모터1 속도 설정: {speed}")
        
    def set_both_motors(self, speed):
        """두 모터 동일 속도 설정"""
        self.set_motor0_speed(speed)
        self.set_motor1_speed(speed)
        rospy.loginfo(f"두 모터 속도 설정: {speed}")
        
    def stop_all_motors(self):
        """모든 모터 정지"""
        self.set_both_motors(0.0)
        rospy.loginfo("모든 모터 정지")

def print_usage():
    print("사용법:")
    print("  python3 motor_control.py 0 [속도]     - 모터0 속도 설정")
    print("  python3 motor_control.py 1 [속도]     - 모터1 속도 설정")
    print("  python3 motor_control.py both [속도]  - 두 모터 동일 속도")
    print("  python3 motor_control.py stop         - 모든 모터 정지")
    print("")
    print("예시:")
    print("  python3 motor_control.py 0 10.5       - 모터0을 10.5로 설정")
    print("  python3 motor_control.py 1 -5.2       - 모터1을 -5.2로 설정")
    print("  python3 motor_control.py both 15.0     - 두 모터를 15.0으로 설정")
    print("  python3 motor_control.py stop          - 모든 모터 정지")

def main():
    if len(sys.argv) < 2:
        print_usage()
        return
        
    try:
        controller = MotorController()
        
        command = sys.argv[1].lower()
        
        if command == "stop":
            controller.stop_all_motors()
        elif command == "0":
            if len(sys.argv) < 3:
                print("모터0 속도 값을 입력하세요.")
                return
            speed = float(sys.argv[2])
            controller.set_motor0_speed(speed)
        elif command == "1":
            if len(sys.argv) < 3:
                print("모터1 속도 값을 입력하세요.")
                return
            speed = float(sys.argv[2])
            controller.set_motor1_speed(speed)
        elif command == "both":
            if len(sys.argv) < 3:
                print("모터 속도 값을 입력하세요.")
                return
            speed = float(sys.argv[2])
            controller.set_both_motors(speed)
        else:
            print(f"알 수 없는 명령: {command}")
            print_usage()
            return
            
        rospy.sleep(0.1)  # 메시지 전송 대기
        
    except ValueError:
        print("속도 값은 숫자여야 합니다.")
    except rospy.ROSException as e:
        print(f"ROS 오류: {e}")
    except Exception as e:
        print(f"오류: {e}")

if __name__ == '__main__':
    main() 