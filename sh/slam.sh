#!/bin/bash


source install/setup.bash
password="123456"
# echo "$password" | sudo -S chmod 777 /dev/ttyUSB0
# echo "$password" | sudo -S chmod 777 /dev/ttyCH341USB0

ros2 launch sentry_slam cartographer.launch.py