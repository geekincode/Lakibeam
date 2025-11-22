#ifndef sentry_SERIAL__sentry_SERIAL_DRIVER_HPP_
#define sentry_SERIAL__sentry_SERIAL_DRIVER_HPP_

#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <rclcpp/publisher.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/subscription.hpp>
#include <std_msgs/msg/float64.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <future>
#include <memory>
#include <string>
#include <thread>
#include <vector>
#include <sentry_serial/sentry_packect.hpp>
#include "sentry_interfaces/msg/gimbal.hpp"
#include "sentry_interfaces/msg/final_angle.hpp"
#include "serial/serial.h"
#include "geometry_msgs/msg/twist.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "nav_msgs/msg/odometry.hpp"


namespace sentry_serial
{
class sentry_serial_node: public rclcpp::Node
{
public:
    // 构造函数
    sentry_serial_node();

private:
    //创建一个serial对象
    serial::Serial sp; 

    // tf广播
    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

    // 云台状态发布者
    rclcpp::Publisher<sentry_interfaces::msg::Gimbal>::SharedPtr gimbal_pub_;

    // 云台状态消息
    sentry_interfaces::msg::Gimbal gimbal_msg;

    // IMU发布者
    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub_;

    // IMU消息
    sensor_msgs::msg::Imu imu_msg;

    // odometer发布者
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odometer_pub_;

    // odometer消息
    nav_msgs::msg::Odometry odometer_msg;

    // 最终目标yawpitch轴角度信息接收节点
    rclcpp::Subscription<sentry_interfaces::msg::FinalAngle>::SharedPtr final_angle_sub_;

    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;

    void cmd_vel_callback(const geometry_msgs::msg::Twist::SharedPtr cmd_vel_msg);
    //const geometry_msgs::msg::Twist::SharedPtr cmd_vel_msg

    // 串口接收线程
    void serial_receive_thread(void);

    // 定时器
    void on_timer(void);

    // 里程计初始化
    void odometer_init(void);

    // 里程计计算
    void odometer_calulation(void);

    // 创建时钟对象
    rclcpp::Clock clock;
    // 保存上次时间戳
    uint64_t last_time;

    // 串口接收数据包
    Serial_ReceivePacket packect;

    // 里程计计算结构体
    Odometer_Calculation odometer;

    // 里程计定时器
    rclcpp::TimerBase::SharedPtr timer_odometer_;

    // imu定时器
    rclcpp::TimerBase::SharedPtr timer_;

    //发送的内容
    long int senddata[2]; 
    uint8_t shoot_flag;
    //接收的内容
    int16_t receivedata[7]; 

    // 发送的包头
    uint8_t send_header = 0xA0;
    // 接收的包头
    uint8_t receive_header = 0xA1;

    // 串口接收超时等待计数器
    uint16_t serial_timed_out_count;

    // test
    uint8_t test;

    // param
    // 通信延时
    double timestamp_offset_ = 0;
    // 默认出弹速度
    double default_speed = 30.0;
    // 串口接收超时等待(ms)
    uint16_t serial_timed_out_liimt = 1000;

    // 使用的串口设备(0:CH340 ; 1:ACM0)
    uint8_t Serial_COM = 0;
};

}

#endif 