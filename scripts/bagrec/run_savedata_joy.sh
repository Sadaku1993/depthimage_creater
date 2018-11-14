#!/bin/bash
source /opt/ros/kinetic/setup.bash
source /home/amsl/ros_catkin_ws/devel/setup.bash

source /home/amsl/.bashrc

# Launch SQ Lidar
gnome-terminal -e '/opt/ros/kinetic/bin/roslaunch sq1_extra run_sq2_for_joy.launch' --geometry=50x12+0+0 &
sleep 1s

# Launch Sensor TF
gnome-terminal -e '/opt/ros/kinetic/bin/roslaunch sensor_fusion laser_tf.launch' --geometry=50x12+0+250 &
sleep 1s

# Transform PointCloud from /centerlaser_ to /centerlaser
gnome-terminal -e '/opt/ros/kinetic/bin/roslaunch sensor_fusion laser_transform_pointcloud.launch' --geometry=50x12+0+500 &
sleep 1s

# Launch realsense
gnome-terminal -e '/home/amsl/ros_catkin_ws/src/depthimage_creater/scripts/realsense/realsense0.sh' --geometry=50x12+0+750 &
gnome-terminal -e '/home/amsl/ros_catkin_ws/src/depthimage_creater/scripts/realsense/realsense1.sh' --geometry=50x12+500+0 &
gnome-terminal -e '/home/amsl/ros_catkin_ws/src/depthimage_creater/scripts/realsense/realsense2.sh' --geometry=50x12+500+250 &
gnome-terminal -e '/home/amsl/ros_catkin_ws/src/depthimage_creater/scripts/realsense/realsense3.sh' --geometry=50x12+500+500 &
gnome-terminal -e '/home/amsl/ros_catkin_ws/src/depthimage_creater/scripts/realsense/realsense4.sh' --geometry=50x12+500+750 &
sleep 1s

# LCL
gnome-terminal -e '/opt/ros/kinetic/bin/roslaunch sensor_fusion lcl.launch' --geometry=50x12+1000+0 &
sleep 1s

# rviz
gnome-terminal -e '/opt/ros/kinetic/bin/rosrun rviz rviz -d /home/amsl/.rviz/depthimage_creater_savedata.rviz' --geometry=50x12+1000+250 &
sleep 1s

# bagrec
# gnome-terminal -e '/home/amsl/ros_catkin_ws/src/calibration/scripts/bag_rec_calibration.sh' --geometry=50x12+1200+250 &
