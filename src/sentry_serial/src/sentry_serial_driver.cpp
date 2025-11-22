#include <tf2/LinearMath/Quaternion.h>
#include <rclcpp/logging.hpp>
#include <rclcpp/qos.hpp>
#include <rclcpp/utilities.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <cstdint>
#include <functional>
#include <map>
#include <memory>
#include <string>
#include <vector>
#include <sentry_serial/sentry_serial_driver.hpp>
#include <sentry_serial/sentry_packect.hpp>
#include "serial/serial.h"
#include "geometry_msgs/msg/twist.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "nav_msgs/msg/odometry.hpp"

using namespace std::chrono_literals;


namespace sentry_serial
{
    // 构造函数
    sentry_serial_node::sentry_serial_node() : Node("sentry_serial_node")
    {
        RCLCPP_INFO(this->get_logger(), "串口节点创建");

        if(Serial_COM==0)
        {
            sp.setPort("/dev/ttyCH341USB0");//选择要开启的串口号
        }
        if(Serial_COM==1)
        {
            sp.setPort("/dev/ttyACM0");//选择要开启的串口号
        }

        sp.setBaudrate(115200);//设置波特率
        serial::Timeout _time =serial::Timeout::simpleTimeout(2000);//超时等待
        sp.setTimeout(_time);

        // TF广播
        tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

        // 创建定时器
        timer_=this->create_wall_timer(1ms,std::bind(&sentry_serial_node::on_timer,this));

        // 串口接收线程
        std::thread(std::bind(&sentry_serial_node::serial_receive_thread,this)).detach();

        // 云台状态发布者
        gimbal_pub_ = this->create_publisher<sentry_interfaces::msg::Gimbal>("/serial/gimbal", rclcpp::SensorDataQoS());

        // IMU发布者
        imu_pub_ = this->create_publisher<sensor_msgs::msg::Imu>("/imu0", rclcpp::SensorDataQoS());

        // odometer发布者
        odometer_pub_ = this->create_publisher<nav_msgs::msg::Odometry>("/odom0", rclcpp::SensorDataQoS());

        cmd_vel_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
            "/cmd_vel", rclcpp::SensorDataQoS(),
            std::bind(&sentry_serial_node::cmd_vel_callback, this, std::placeholders::_1));

        // 默认出弹速度
        packect.speed = default_speed ;

        odometer_init();
    }

    // tf广播线程(jieshoushuju)
    void sentry_serial_node::serial_receive_thread(void)
    {
        rclcpp::Rate rate(2.0);//一秒钟执行两次
        while (!sp.isOpen())
        {
            try
            {
                //打开串口
                sp.open();
                RCLCPP_INFO(this->get_logger(),"打开串口成功, sp.open();");
            }
            catch (serial::IOException& e)
            {
                RCLCPP_FATAL(this->get_logger(),"Unable to open port");
                rate.sleep();
            }
        }

        while(1)
        {
            // 接收的包头
            uint8_t original_receivedata_header[1];
            // 接收的原始数据(还要接受校验数据)
            uint8_t original_receivedata[sizeof(receivedata) + 2];
            // RCLCPP_INFO(this->get_logger(),"while");

            if(sp.available()!=0)
            {
                // RCLCPP_INFO(this->get_logger(),"receive_data");
                sp.read(original_receivedata_header,sizeof(original_receivedata_header));

                if(original_receivedata_header[0]==receive_header)
                {
                    // 接收数据
                    sp.read(original_receivedata,sizeof(original_receivedata));

                    // 合校验
                    if (original_receivedata[sizeof(receivedata)+1] == (original_receivedata[0]>>4) + (original_receivedata[2]>>4))
                    {
                        receivedata[0] = ((int16_t)original_receivedata[0]<<8) + (int16_t)original_receivedata[1];
                        receivedata[1] = ((int16_t)original_receivedata[2]<<8) + (int16_t)original_receivedata[3];
                        receivedata[2] = ((int16_t)original_receivedata[4]<<8) + (int16_t)original_receivedata[5];
                        receivedata[3] = ((int16_t)original_receivedata[6]<<8) + (int16_t)original_receivedata[7];
                        receivedata[4] = ((int16_t)original_receivedata[8]<<8) + (int16_t)original_receivedata[9];
                        receivedata[5] = ((int16_t)original_receivedata[10]<<8) + (int16_t)original_receivedata[11];
                        receivedata[6] = ((int16_t)original_receivedata[12]<<8) + (int16_t)original_receivedata[13];

                        packect.roll = (double)receivedata[0]/10000;
                        packect.pitch = (double)receivedata[1]/10000;
                        packect.yaw = (double)receivedata[2]/10000;
                        packect.wheel_1 = (double)receivedata[3];
                        packect.wheel_2 = (double)receivedata[4];
                        packect.wheel_3 = (double)receivedata[5];
                        packect.wheel_4 = (double)receivedata[6];
                        packect.color = original_receivedata[14];

                        // RCLCPP_INFO(this->get_logger(),"接收数据roll:%.2f;  yaw:%.2f,   pitch:%.2f;  speed:%.2f;  color:%d",packect.roll,packect.yaw,packect.pitch,packect.speed,packect.color);
                        // RCLCPP_INFO(this->get_logger(),"接收数据wheel1:%4.5f;  wheel2:%4.5f,   wheel3:%4.5f;  wheel4:%4.5f;  color:%d",
                        // packect.wheel_1,packect.wheel_2,packect.wheel_3,packect.wheel_4,packect.color);
                        serial_timed_out_count = 0;
                    }
                }
            }
        }
    }

    // 定时器
    void sentry_serial_node::on_timer(void)
    {
        geometry_msgs::msg::TransformStamped t;
        t.header.stamp = this->now() + rclcpp::Duration::from_seconds(timestamp_offset_);
        t.header.frame_id = "odom";
        t.child_frame_id = "gimbal_link";
        tf2::Quaternion q;
        // 定轴欧拉角旋转
        q.setRPY(packect.roll, packect.pitch, packect.yaw);
        t.transform.rotation = tf2::toMsg(q);
        tf_broadcaster_->sendTransform(t);

        // 发布云台状态信息
        gimbal_msg.roll = packect.roll;
        gimbal_msg.yaw = packect.yaw ;
        gimbal_msg.pitch = packect.pitch;
        gimbal_msg.speed = packect.speed;
        gimbal_msg.color = packect.color;
        gimbal_pub_->publish(gimbal_msg);

        // 发布IMU信息
        imu_msg.header.stamp = this->now();
        imu_msg.header.frame_id = "imu";
        imu_msg.orientation = tf2::toMsg(q);
        imu_pub_->publish(imu_msg);

        odometer_calulation();

        serial_timed_out_count++;
    }

    void sentry_serial_node::odometer_init(void)
    {
        odometer.k = 2.05e-4;
        odometer.theta = 0;
        odometer.dt = 0.01;
    }

    void sentry_serial_node::odometer_calulation(void)
    {
        // RCLCPP_INFO(this->get_logger(),"odometer_calulation");
        auto now = clock.now();//获得当前时间戳
        odometer.dt = (double)(now.nanoseconds()-last_time)/1000000000;

        // 输出日志 
        // RCLCPP_INFO(this->get_logger(), "本次耗时: %lf s", odometer.dt);
        // 保存本次时间戳
        last_time = now.nanoseconds();

        odometer.v_1 = packect.wheel_1 * odometer.k;
        odometer.v_2 = packect.wheel_2 * odometer.k;
        odometer.v_3 = packect.wheel_3 * odometer.k;
        odometer.v_4 = packect.wheel_4 * odometer.k;

        odometer.w_1 = odometer.v_1 / odometer.radius;
        odometer.w_1 = odometer.v_2 / odometer.radius;
        odometer.w_1 = odometer.v_3 / odometer.radius;
        odometer.w_1 = odometer.v_4 / odometer.radius;

        odometer.v_z_odom = odometer.w_1 + odometer.w_2 + odometer.w_3 + odometer.w_4;
        odometer.theta = packect.yaw;

        odometer.v_x_odom = (- odometer.v_1 - odometer.v_2 + odometer.v_3 + odometer.v_4) * 0.707f;
        odometer.v_y_odom = (- odometer.v_1 + odometer.v_2 + odometer.v_3 - odometer.v_4) * 0.707f;

        odometer.v_x_map = odometer.v_x_odom * cos(odometer.theta) 
                            - odometer.v_y_odom * sin(odometer.theta);
        odometer.v_y_map = odometer.v_x_odom * sin(odometer.theta)
                            + odometer.v_y_odom * cos(odometer.theta);

        odometer.X_Position += odometer.v_x_map * odometer.dt;
        odometer.Y_Position += odometer.v_y_map * odometer.dt;

        // RCLCPP_INFO(this->get_logger(),"X: %f--------Y: %f", 
        //             odometer.X_Position, odometer.Y_Position);
        // RCLCPP_INFO(this->get_logger(),"theta: %f", odometer.theta);
    }

    void sentry_serial_node::cmd_vel_callback(const geometry_msgs::msg::Twist::SharedPtr cmd_vel_msg)
    {
        // RCLCPP_INFO(this->get_logger(),"发送/cmd_vel");
        // cmd_vel_msg->linear.x;

        // 发送的数据
        senddata[0] = (int16_t)(cmd_vel_msg->linear.x * 1000);
        senddata[1] = (int16_t)(cmd_vel_msg->linear.y * 1000);
        senddata[2] = (int16_t)(cmd_vel_msg->angular.z * 1000);


        // 发送的原始数据
        uint8_t original_senddata[sizeof(senddata) + sizeof(shoot_flag) + 2];
        original_senddata[0] = send_header;
        original_senddata[1] = senddata[0] >> 8;
        original_senddata[2] = senddata[0];
        original_senddata[3] = senddata[1] >> 8;
        original_senddata[4] = senddata[1];
        original_senddata[5] = senddata[2] >> 8;
        original_senddata[6] = senddata[2];


        if (sp.isOpen())// && serial_timed_out_count<serial_timed_out_liimt
        {
            sp.write(original_senddata,sizeof(original_senddata));//两个参数，第一个参数是要发送的数据地址，第二个数据是要发送数据的长度
            RCLCPP_INFO(this->get_logger(),"x:%lf, y:%lf, z:%lf", cmd_vel_msg->linear.x, cmd_vel_msg->linear.y, cmd_vel_msg->angular.z);
            RCLCPP_INFO(this->get_logger(),"%ld----%ld----%ld", senddata[0], senddata[1], senddata[2]);
        }
    };
}


int main(int argc, char const *argv[])
{
    // 初始化ros2客户端
    rclcpp::init(argc, argv);
    // 调用spin函数
    rclcpp::spin(std::make_shared<sentry_serial::sentry_serial_node>());
    // 释放资源
    rclcpp::shutdown();
    return 0;
}
