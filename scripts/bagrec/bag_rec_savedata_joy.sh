#!/bin/bash

hander()
{
    sleep 1
}

trap hander SIGINT

TIME=$(date +%Y-%m-%d-%H-%M-%S)
CLOUD="/cloud/tf"
ODOM="/odom"
IMU="/imu/data"
LCL="/cloud/lcl"
REALSENSE0="/camera0/color/image_raw/compressed /camera0/color/camera_info"
REALSENSE1="/camera1/color/image_raw/compressed /camera1/color/camera_info"
REALSENSE2="/camera2/color/image_raw/compressed /camera2/color/camera_info"
REALSENSE3="/camera3/color/image_raw/compressed /camera3/color/camera_info"
REALSENSE4="/camera4/color/image_raw/compressed /camera4/color/camera_info"
OBJECT_INFO="/object_info/0 /object_info/1 /object_info/2 /object_info/3 /object_info/4"

echo $TIME &
echo $CLOUD &
echo $ODOM &
echo $IMU &
echo $LCL &
echo $REALSENSE0 &
echo $REALSENSE1 &
echo $REALSENSE2 &
echo $REALSENSE3 &
echo $REALSENSE4 &
echo $OBJECT_INFO &

/opt/ros/kinetic/bin/rosbag record $CLOUD $ODOM $IMU $LCL $REALSENSE0 $REALSENSE1 $REALSENSE2 $REALSENSE3 $REALSENSE4 $OBJECT_INFO -O /home/amsl/$TIME.bag
