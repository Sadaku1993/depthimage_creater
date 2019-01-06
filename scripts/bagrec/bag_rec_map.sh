
#!/bin/bash

hander()
{
    sleep 1
}

trap hander SIGINT

TIME=$(date +%Y-%m-%d-%H-%M-%S)
# CLOUD="/cloud/tf"
ODOM="/odom"
IMU="/imu/data"
LCL="/cloud/lcl"
RM_HUMAN="/cluster/human/removed"
NORMAL="/cloud_normal"

echo $TIME &
# echo $CLOUD &
echo $ODOM &
echo $IMU &
echo $LCL &
echo $RM_HUMAN &
echo $NORMAL &

# /opt/ros/kinetic/bin/rosbag record $ODOM $IMU $RM_HUMAN -O /home/amsl/$TIME.bag
/opt/ros/kinetic/bin/rosbag record $ODOM $IMU $NORMAL -O /home/amsl/$TIME.bag
