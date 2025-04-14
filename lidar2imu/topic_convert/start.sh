#!/bin/sh
rm -rf build/ install/ log/
colcon build 
cd build/onlinecalib
./onlinecalib --ros-args -p lidar_topic:=/my_rslidar_points -p imu_topic:=/my_odometry
