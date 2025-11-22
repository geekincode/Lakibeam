#!/bin/bash

# colcon build

source install/setup.bash
password="123456"
echo "$password" | sudo -S chmod 777 /dev/ttyCH341USB0

ros2 run sentry_serial sentry_serial_driver