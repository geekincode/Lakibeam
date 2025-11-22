#!/bin/bash

# 启动串口通信----------------------------------------------
echo "启动串口通信"

if [ -d "log_run" ]; then
    echo "输出日志到:log_run" > /dev/null
else
    echo "创建日志目录并输出:log_run" > /dev/null
    mkdir "log_run" || { echo "创建目录失败"; exit 1; }
fi

cd log_run
if [ -d "serial" ]; then
    echo "输出日志到:log_run/seiral"
else
    echo "创建日志目录并输出:log_run/serial"
    mkdir "serial" || { echo "创建目录失败"; exit 1; }
fi
cd ..
serial_logfile="log_run/serial/serial_$(date +%Y-%m-%d_%H-%M-%S).log"

password="123456"
echo "$password" | sudo -S chmod 777 /dev/ttyCH341USB0
source install/setup.bash 
ros2 run sentry_serial sentry_serial_driver > "$serial_logfile" 2>&1
# ---------------------------------------------------------
