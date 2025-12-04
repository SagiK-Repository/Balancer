#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Bool.h>
#include <sensor_msgs/Imu.h>
#include "integrated_balancer/balancer_core.h"

int main(int argc, char** argv) {
    ros::init(argc, argv, "integrated_balancer");
    ros::NodeHandle nh;
    ros::NodeHandle pnh("~");
    
    ROS_INFO("=== Integrated Balancer Starting ===");
    
    // 1. Native Control Core 시작 (별도 스레드에서 실행)
    BalancerCore robot;
    
    if (!robot.initialize()) {
        ROS_ERROR("Failed to initialize BalancerCore");
        return -1;
    }
    
    robot.start();
    ROS_INFO("BalancerCore started in Real-Time thread");
    
    // PID 게인 로드 (기본값은 BalancerCore 생성자에서 설정됨)
    double kp, ki, kd;
    pnh.param("pid_kp_balance", kp, 9.0);
    pnh.param("pid_ki_balance", ki, 2.0);
    pnh.param("pid_kd_balance", kd, 0.5);
    robot.setBalancePIDGains(kp, ki, kd);
    ROS_INFO("PID Gains: Kp=%.2f, Ki=%.2f, Kd=%.2f", kp, ki, kd);
    
    // 목표 각도 로드
    double target_angle;
    pnh.param("target_roll_angle", target_angle, -0.7);
    robot.setTargetAngle(static_cast<float>(target_angle));
    ROS_INFO("Target roll angle: %.2f°", target_angle);
    
    // 2. ROS Communication Loop (메인 스레드)
    ros::Rate rate(10); // 10Hz (Telemetry용, 제어와 무관)
    
    // Subscriber: cmd_vel 등을 받아 robot 객체의 atomic 변수 업데이트
    ros::Subscriber cmd_vel_sub = nh.subscribe<geometry_msgs::Twist>(
        "cmd_vel", 10,
        [&robot](const geometry_msgs::Twist::ConstPtr& msg) {
            // cmd_vel을 목표 속도로 변환 (간단한 구현)
            // 실제로는 바퀴 속도로 변환 필요
            float linear_vel = static_cast<float>(msg->linear.x);
            robot.setTargetSpeed(linear_vel);
        });
    
    // Publisher: robot 객체의 상태(각도, 속도)를 읽어 토픽으로 발행
    ros::Publisher imu_pub = nh.advertise<sensor_msgs::Imu>("imu/data", 10);
    ros::Publisher angle_pub = nh.advertise<std_msgs::Float32>("sensor_roll", 10);
    ros::Publisher status_pub = nh.advertise<std_msgs::Bool>("system_ready", 10);
    
    ROS_INFO("=== Integrated Balancer started successfully ===");
    ROS_INFO("ROS Bridge running at 10Hz (Telemetry only)");
    ROS_INFO("Control loop running at 100Hz (Real-Time thread)");
    
    while (ros::ok() && robot.isRunning()) {
        // Publisher: robot 객체의 상태를 읽어 토픽으로 발행
        float current_angle = robot.getCurrentAngle();
        
        // IMU 메시지 발행
        sensor_msgs::Imu imu_msg;
        imu_msg.header.stamp = ros::Time::now();
        imu_msg.header.frame_id = "imu_link";
        
        // 각속도는 센서에서 직접 읽을 수 없으므로 0으로 설정
        // 실제로는 BalancerCore에서 gyro_rate를 제공해야 함
        imu_msg.angular_velocity.x = 0.0;
        imu_msg.angular_velocity.y = 0.0;
        imu_msg.angular_velocity.z = 0.0;
        
        // 자세 데이터 (Roll만)
        double roll_rad = current_angle * M_PI / 180.0;
        double pitch_rad = 0.0;
        double yaw_rad = 0.0;
        
        // 쿼터니언 변환
        double cy = cos(yaw_rad * 0.5);
        double sy = sin(yaw_rad * 0.5);
        double cp = cos(pitch_rad * 0.5);
        double sp = sin(pitch_rad * 0.5);
        double cr = cos(roll_rad * 0.5);
        double sr = sin(roll_rad * 0.5);
        
        imu_msg.orientation.w = cr * cp * cy + sr * sp * sy;
        imu_msg.orientation.x = sr * cp * cy - cr * sp * sy;
        imu_msg.orientation.y = cr * sp * cy + sr * cp * sy;
        imu_msg.orientation.z = cr * cp * sy - sr * sp * cy;
        
        imu_pub.publish(imu_msg);
        
        // 각도 메시지 발행
        std_msgs::Float32 angle_msg;
        angle_msg.data = current_angle;
        angle_pub.publish(angle_msg);
        
        // 시스템 준비 상태 발행
        std_msgs::Bool status_msg;
        status_msg.data = robot.isRunning();
        status_pub.publish(status_msg);
        
        ros::spinOnce();
        rate.sleep();
    }
    
    ROS_INFO("Shutting down Integrated Balancer...");
    robot.stop();
    
    ROS_INFO("Integrated Balancer shutdown complete");
    return 0;
}




