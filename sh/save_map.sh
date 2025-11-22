#!/bin/bash

ros2 service call /finish_trajectory cartographer_ros_msgs/srv/FinishTrajectory "{trajectory_id: 0}"

ros2 service call /write_state cartographer_ros_msgs/srv/WriteState "{filename: '/home/rm/map/mymap0327.pbstream'}"

ros2 run nav2_map_server map_saver_cli -t map -f /home/rm/map/map_20250327
