#!/bin/bash

ssh -t -t tx2-0 << EOF
export ROS_MASTER_URI="http://192.168.0.142:11311"

sleep 1s

/opt/ros/kinetic/bin/roslaunch tensorflow_object_detection_tx2 realsense_0.launch

sleep 1s
