#ifndef sentry_SERIAL__sentry_PACKECT_HPP_
#define sentry_SERIAL__sentry_PACKECT_HPP_


namespace sentry_serial
{
    // 串口接收数据包
    struct Serial_ReceivePacket
    {
        float roll;
        float pitch;
        float yaw;
        float speed;
        float wheel_1;
        float wheel_2;
        float wheel_3;
        float wheel_4;
        uint8_t color;

    } __attribute__((packed));

    struct Odometer_Calculation
    {
        float v_1, v_2, v_3, v_4;                       //四个轮子的线速度
        float w_1, w_2, w_3, w_4;                       //四个轮子的角速度
        float radius;                                   //底盘半径
        float theta;                                    //odom坐标系与map坐标系的夹角，imu_yaw数据
        float k, dt;                                    //补偿系数和解算间隔时间
        float v_x_odom, v_y_odom, v_z_odom;             //odom坐标系下xyz方向速度
        float v_x_map, v_y_map, v_z_map;                //map坐标系下xyz方向速度
        float X_Position, Y_Position;                   //map坐标系下XY方向位置

    } __attribute__((packed));
}

#endif 
