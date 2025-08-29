#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
from std_msgs.msg import Float32
import time
import sys

class AutoDualMotorTester:
    def __init__(self):
        rospy.init_node('auto_dual_motor_tester', anonymous=True)
        
        self.motor0_pub = rospy.Publisher('target_speed0', Float32, queue_size=10)
        self.motor1_pub = rospy.Publisher('target_speed1', Float32, queue_size=10)
        
        # 상태 구독
        self.actual_speed0 = 0.0
        self.actual_speed1 = 0.0
        self.sensor_confidence = 100.0
        
        rospy.Subscriber('actual_speed0', Float32, self.actual_speed0_callback)
        rospy.Subscriber('actual_speed1', Float32, self.actual_speed1_callback)
        rospy.Subscriber('sensor_confidence', Float32, self.sensor_confidence_callback)
        
        rospy.sleep(1.0)  # 초기화 대기
        
        rospy.loginfo("=== 🤖 자동 두 바퀴 모터 테스트 시작 ===")
        
    def actual_speed0_callback(self, msg):
        self.actual_speed0 = msg.data
        
    def actual_speed1_callback(self, msg):
        self.actual_speed1 = msg.data
        
    def sensor_confidence_callback(self, msg):
        self.sensor_confidence = msg.data
        
    def set_motor_speeds(self, speed0, speed1):
        """두 모터 속도 설정"""
        msg0 = Float32()
        msg1 = Float32()
        msg0.data = float(speed0)
        msg1.data = float(speed1)
        
        self.motor0_pub.publish(msg0)
        self.motor1_pub.publish(msg1)
        
        rospy.loginfo(f"모터 속도 설정: [{speed0:.1f}, {speed1:.1f}]")
        
    def stop_all_motors(self):
        """모든 모터 정지"""
        self.set_motor_speeds(0.0, 0.0)
        rospy.loginfo("🛑 모든 모터 정지")
        
    def wait_and_monitor(self, duration):
        """지정된 시간 동안 대기하며 상태 모니터링"""
        start_time = time.time()
        
        while time.time() - start_time < duration and not rospy.is_shutdown():
            rospy.sleep(0.1)
            
        # 최종 상태 출력
        rospy.loginfo(f"📊 실제속도[{self.actual_speed0:.1f}, {self.actual_speed1:.1f}] 신뢰도:{self.sensor_confidence:.0f}%")
        
        # 센서 신뢰도 경고
        if self.sensor_confidence < 50.0:
            rospy.logwarn(f"⚠️ 센서 신뢰도 낮음: {self.sensor_confidence:.0f}%")
            
    def test_individual_motors(self):
        """개별 모터 테스트"""
        rospy.loginfo("🔧 개별 모터 테스트 시작")
        
        # 모터0만 테스트
        rospy.loginfo("1️⃣ 모터0 단독 테스트 (5초)")
        self.set_motor_speeds(5.0, 0.0)
        self.wait_and_monitor(5.0)
        
        self.stop_all_motors()
        self.wait_and_monitor(2.0)
        
        # 모터1만 테스트
        rospy.loginfo("2️⃣ 모터1 단독 테스트 (5초)")
        self.set_motor_speeds(0.0, 5.0)
        self.wait_and_monitor(5.0)
        
        self.stop_all_motors()
        rospy.loginfo("✅ 개별 모터 테스트 완료")
        
    def test_simultaneous_motors(self):
        """동시 모터 테스트"""
        rospy.loginfo("🔧 동시 모터 테스트 시작")
        
        test_speeds = [3.0, 6.0, 10.0, -5.0, -10.0]
        
        for speed in test_speeds:
            rospy.loginfo(f"⚡ 두 모터 동일 속도: {speed:.1f} (3초)")
            self.set_motor_speeds(speed, speed)
            self.wait_and_monitor(3.0)
            
        self.stop_all_motors()
        rospy.loginfo("✅ 동시 모터 테스트 완료")
        
    def test_direction_control(self):
        """방향 제어 테스트"""
        rospy.loginfo("🔧 방향 제어 테스트 시작")
        
        # 전진
        rospy.loginfo("⬆️ 전진 테스트 (5초)")
        self.set_motor_speeds(8.0, 8.0)
        self.wait_and_monitor(5.0)
        
        self.stop_all_motors()
        self.wait_and_monitor(1.0)
        
        # 후진
        rospy.loginfo("⬇️ 후진 테스트 (5초)")
        self.set_motor_speeds(-8.0, -8.0)
        self.wait_and_monitor(5.0)
        
        self.stop_all_motors()
        self.wait_and_monitor(1.0)
        
        # 좌회전
        rospy.loginfo("⬅️ 좌회전 테스트 (3초)")
        self.set_motor_speeds(3.0, 8.0)
        self.wait_and_monitor(3.0)
        
        self.stop_all_motors()
        self.wait_and_monitor(1.0)
        
        # 우회전
        rospy.loginfo("➡️ 우회전 테스트 (3초)")
        self.set_motor_speeds(8.0, 3.0)
        self.wait_and_monitor(3.0)
        
        self.stop_all_motors()
        rospy.loginfo("✅ 방향 제어 테스트 완료")
        
    def test_smooth_control(self):
        """부드러운 제어 테스트"""
        rospy.loginfo("🔧 부드러운 제어 테스트 시작")
        
        # 점진적 가속
        rospy.loginfo("📈 점진적 가속 테스트")
        for speed in [0.0, 2.0, 4.0, 6.0, 8.0, 10.0]:
            rospy.loginfo(f"속도 증가: {speed:.1f}")
            self.set_motor_speeds(speed, speed)
            self.wait_and_monitor(2.0)
            
        # 점진적 감속
        rospy.loginfo("📉 점진적 감속 테스트")
        for speed in [8.0, 6.0, 4.0, 2.0, 0.0]:
            rospy.loginfo(f"속도 감소: {speed:.1f}")
            self.set_motor_speeds(speed, speed)
            self.wait_and_monitor(2.0)
            
        # 방향 전환
        rospy.loginfo("🔄 방향 전환 테스트")
        self.set_motor_speeds(8.0, 8.0)
        self.wait_and_monitor(3.0)
        
        self.set_motor_speeds(-8.0, -8.0)
        self.wait_and_monitor(3.0)
        
        self.stop_all_motors()
        rospy.loginfo("✅ 부드러운 제어 테스트 완료")
        
    def test_speed_steps(self):
        """속도 단계별 테스트"""
        rospy.loginfo("🔧 속도 단계별 테스트 시작")
        
        test_speeds = [1.0, 3.0, 5.0, 8.0, 12.0, 15.0]
        
        for speed in test_speeds:
            rospy.loginfo(f"🎯 속도 단계: {speed:.1f} (4초)")
            self.set_motor_speeds(speed, speed)
            self.wait_and_monitor(4.0)
            
            # 센서 신뢰도 체크
            if self.sensor_confidence < 50.0:
                rospy.logwarn(f"⚠️ 센서 신뢰도 낮음 ({self.sensor_confidence:.0f}%) - 속도 조정 권장")
                
        self.stop_all_motors()
        rospy.loginfo("✅ 속도 단계별 테스트 완료")
        
    def test_differential_control(self):
        """차동 제어 테스트 (두 바퀴 다른 속도)"""
        rospy.loginfo("🔧 차동 제어 테스트 시작")
        
        test_cases = [
            (5.0, 3.0, "모터0 빠름"),
            (3.0, 5.0, "모터1 빠름"),
            (8.0, -3.0, "모터0 전진, 모터1 후진"),
            (-3.0, 8.0, "모터0 후진, 모터1 전진"),
            (10.0, 0.0, "모터0만 동작"),
            (0.0, 10.0, "모터1만 동작")
        ]
        
        for speed0, speed1, description in test_cases:
            rospy.loginfo(f"🎮 {description}: [{speed0:.1f}, {speed1:.1f}] (3초)")
            self.set_motor_speeds(speed0, speed1)
            self.wait_and_monitor(3.0)
            
            self.stop_all_motors()
            self.wait_and_monitor(1.0)
            
        rospy.loginfo("✅ 차동 제어 테스트 완료")
        
    def run_full_test_suite(self):
        """전체 테스트 스위트 실행"""
        rospy.loginfo("🚀 전체 두 바퀴 테스트 스위트 시작")
        
        try:
            # 1. 개별 모터 테스트
            self.test_individual_motors()
            rospy.sleep(2.0)
            
            # 2. 동시 모터 테스트
            self.test_simultaneous_motors()
            rospy.sleep(2.0)
            
            # 3. 방향 제어 테스트
            self.test_direction_control()
            rospy.sleep(2.0)
            
            # 4. 부드러운 제어 테스트
            self.test_smooth_control()
            rospy.sleep(2.0)
            
            # 5. 속도 단계별 테스트
            self.test_speed_steps()
            rospy.sleep(2.0)
            
            # 6. 차동 제어 테스트
            self.test_differential_control()
            
            rospy.loginfo("🎉 전체 테스트 스위트 완료!")
            
        except Exception as e:
            rospy.logerr(f"테스트 중 오류 발생: {e}")
        finally:
            self.stop_all_motors()
            rospy.loginfo("🏁 테스트 종료 - 모든 모터 정지")

def print_usage():
    print("사용법:")
    print("  python3 auto_dual_test.py [테스트_타입]")
    print("")
    print("테스트 타입:")
    print("  individual  - 개별 모터 테스트")
    print("  simultaneous - 동시 모터 테스트")
    print("  direction   - 방향 제어 테스트")
    print("  smooth      - 부드러운 제어 테스트")
    print("  steps       - 속도 단계별 테스트")
    print("  differential - 차동 제어 테스트")
    print("  full        - 전체 테스트 스위트 (기본값)")
    print("")
    print("예시:")
    print("  python3 auto_dual_test.py")
    print("  python3 auto_dual_test.py individual")
    print("  python3 auto_dual_test.py smooth")

def main():
    if len(sys.argv) > 1 and sys.argv[1] in ['-h', '--help', 'help']:
        print_usage()
        return
        
    try:
        tester = AutoDualMotorTester()
        
        test_type = sys.argv[1].lower() if len(sys.argv) > 1 else 'full'
        
        if test_type == 'individual':
            tester.test_individual_motors()
        elif test_type == 'simultaneous':
            tester.test_simultaneous_motors()
        elif test_type == 'direction':
            tester.test_direction_control()
        elif test_type == 'smooth':
            tester.test_smooth_control()
        elif test_type == 'steps':
            tester.test_speed_steps()
        elif test_type == 'differential':
            tester.test_differential_control()
        elif test_type == 'full':
            tester.run_full_test_suite()
        else:
            print(f"알 수 없는 테스트 타입: {test_type}")
            print_usage()
            return
            
    except rospy.ROSException as e:
        rospy.logerr(f"ROS 오류: {e}")
    except KeyboardInterrupt:
        rospy.loginfo("사용자에 의해 테스트 중단됨")
    except Exception as e:
        rospy.logerr(f"오류: {e}")

if __name__ == '__main__':
    main() 