#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import Imu
import smbus2
import time

class MPU6050:
    def __init__(self, bus_number=1, address=0x68):
        self.bus = smbus2.SMBus(bus_number)
        self.address = address
        # MPU6050 초기화
        self.bus.write_byte_data(self.address, 0x6B, 0)  # Power management
        
    def read_raw_data(self, addr):
        high = self.bus.read_byte_data(self.address, addr)
        low = self.bus.read_byte_data(self.address, addr+1)
        value = ((high << 8) | low)
        if value > 32768:
            value = value - 65536
        return value
    
    def get_accel_data(self):
        acc_x = self.read_raw_data(0x3B) / 16384.0
        acc_y = self.read_raw_data(0x3D) / 16384.0
        acc_z = self.read_raw_data(0x3F) / 16384.0
        return acc_x, acc_y, acc_z
    
    def get_gyro_data(self):
        gyro_x = self.read_raw_data(0x43) / 131.0
        gyro_y = self.read_raw_data(0x45) / 131.0
        gyro_z = self.read_raw_data(0x47) / 131.0
        return gyro_x, gyro_y, gyro_z

def main():
    rospy.init_node('sensor_test')
    pub = rospy.Publisher('/imu/data', Imu, queue_size=10)
    
    mpu = MPU6050(bus_number=1)
    rate = rospy.Rate(100)
    
    while not rospy.is_shutdown():
        accel_data = mpu.get_accel_data()
        gyro_data = mpu.get_gyro_data()
        
        imu_msg = Imu()
        imu_msg.header.stamp = rospy.Time.now()
        imu_msg.header.frame_id = "imu"
        
        # 가속도계 데이터 (m/s^2)
        imu_msg.linear_acceleration.x = accel_data[0] * 9.81
        imu_msg.linear_acceleration.y = accel_data[1] * 9.81
        imu_msg.linear_acceleration.z = accel_data[2] * 9.81
        
        # 자이로스코프 데이터 (rad/s)
        imu_msg.angular_velocity.x = gyro_data[0]
        imu_msg.angular_velocity.y = gyro_data[1]
        imu_msg.angular_velocity.z = gyro_data[2]
        
        pub.publish(imu_msg)
        
        rospy.loginfo("Accel: %.2f, %.2f, %.2f", accel_data[0], accel_data[1], accel_data[2])
        rate.sleep()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass