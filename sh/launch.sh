#!/bin/bash

# 启动导航-------------------------------------------------
echo "启动导航"

if [ -d "log_run" ]; then
    echo "输出日志到:log_run" > /dev/null
else
    echo "创建日志目录并输出:log_run" > /dev/null
    mkdir "log_run" || { echo "创建目录失败"; exit 1; }
fi

cd log_run
if [ -d "nav" ]; then
    echo "输出日志到:log_run/nav"
else
    echo "创建日志目录并输出:log_run/nav"
    mkdir "nav" || { echo "创建目录失败"; exit 1; }
fi
cd ..

date_run="$(date +"%Y-%m-%d %H:%M:%S")"
nav_logfile="log_run/nav/nav_$(date +%Y-%m-%d_%H-%M-%S).log"

source install/setup.bash
ros2 launch sentry_main sentry_main.launch.py > "$nav_logfile" 2>&1
#---------------------------------------------------------
