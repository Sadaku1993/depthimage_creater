#!/bin/bash
source /opt/ros/kinetic/setup.bash
source /home/amsl/ros_catkin_ws/devel/setup.bash

source /home/amsl/.bashrc

gnome-terminal -e "/opt/ros/kinetic/bin/roslaunch sq1_extra sq1_vehicle.launch" --geometry=50x12+0+0 &

gnome-terminal -e "/opt/ros/kinetic/bin/roslaunch xsens_driver xsens_driver.launch" --geometry=50x12+0+250 &

gnome-terminal -e "/opt/ros/kinetic/bin/roslaunch velodyne_pointcloud 32e_points.launch" --geometry=50x12+0+500 &

# rviz
gnome-terminal -e '/opt/ros/kinetic/bin/rosrun rviz rviz -d /home/amsl/.rviz/depthimage_creater_velodyne.rviz' --geometry=50x12+0+750 &
sleep 1s

# bagrec
# gnome-terminal -e '/home/amsl/ros_catkin_ws/src/calibration/scripts/bag_rec_calibration.sh' --geometry=50x12+1200+250 &
