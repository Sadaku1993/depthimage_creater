#!/bin/bash
source /opt/ros/kinetic/setup.bash
source /home/amsl/ros_catkin_ws/devel/setup.bash

source /home/amsl/.bashrc

# roscore
gnome-terminal -e '/opt/ros/kinetic/bin/roscore' --geometry=50x12+0+0 &
sleep 1s

# Launch Sensor TF
gnome-terminal -e '/opt/ros/kinetic/bin/roslaunch depthimage_creater sq2_zed.launch' --geometry=50x12+0+250 &
sleep 1s
gnome-terminal -e '/opt/ros/kinetic/bin/roslaunch depthimage_creater zed_dumy_tf.launch' --geometry=50x12+0+500 &
sleep 1s
gnome-terminal -e '/opt/ros/kinetic/bin/roslaunch sensor_fusion transform_broadcaster.launch' --geometry=50x12+0+750 &
sleep 1s

# split pointcloud
gnome-terminal -e '/opt/ros/kinetic/bin/roslaunch depthimage_creater split.launch' --geometry=50x12+600+0 &
sleep 1s

# divide pointcloud
gnome-terminal -e '/opt/ros/kinetic/bin/roslaunch sensor_fusion division.launch' --geometry=50x12+600+250 &
sleep 1s

# republish Image
gnome-terminal -e '/opt/ros/kinetic/bin/roslaunch sensor_fusion republish.launch' --geometry=50x12+600+500 &
sleep 1s

# lcl
gnome-terminal -e '/opt/ros/kinetic/bin/roslaunch sensor_fusion lcl_for_bag.launch' --geometry=50x12+1200+0 & 
sleep 1s

# height_map
gnome-terminal -e '/opt/ros/kinetic/bin/roslaunch velodyne_height_map sq_height_map.launch' --geometry=50x12+1200+250 &
sleep 1s

# clustering
gnome-terminal -e '/opt/ros/kinetic/bin/roslaunch depthimage_creater clustering.launch' --geometry=50x12+1200+500 &
sleep 1s

# object detection
gnome-terminal -e '/opt/ros/kinetic/bin/roslaunch tensorflow_object_detection multi_camera.launch' --geometry=50x12+1200+750 &
sleep 1s

# bagfile
gnome-terminal -e '/opt/ros/kinetic/bin/rosbag play /home/amsl/bagfiles/2018/SII/sq2_zed_dkan_outdoor.bag' --geometry=50x12+1800+0 &
sleep 1s

# rviz
gnome-terminal -e '/opt/ros/kinetic/bin/rosrun rviz rviz -d /home/amsl/.rviz/depthimage_creater.rviz' --geometry=50x12+1800+250 &
sleep 1s
