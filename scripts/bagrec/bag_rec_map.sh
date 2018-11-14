
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

echo $TIME &
# echo $CLOUD &
echo $ODOM &
echo $IMU &
echo $LCL &
echo $RM_HUMAN &

/opt/ros/kinetic/bin/rosbag record $ODOM $IMU $LCL $RM_HUMAN -O /home/amsl/$TIME.bag
