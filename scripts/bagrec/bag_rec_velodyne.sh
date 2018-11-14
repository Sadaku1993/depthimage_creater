#!/bin/bash

hander()
{
    sleep 1
}

trap hander SIGINT

TIME=$(date +%Y-%m-%d-%H-%M-%S)
ODOM="/odom"
IMU="/imu/data"
VELODYNE="/velodyne_packets"

echo $TIME &
echo $ODOM &
echo $IMU &
echo $VELODYNE &

/opt/ros/kinetic/bin/rosbag record $ODOM $IMU $VELODYNE -O /home/amsl/$TIME.bag
