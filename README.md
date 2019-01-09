# DepthImage Creater
By integration the data of LiDAR and Depth Camera, create Depth Image for Depth Estimation Dataset.

## Reauirement
- ROS (kinetic)
- tensorflow
- [tensorflow_object_detection](https://github.com/Sadaku1993/tensorflow_object_detection_ros)
- [amsl_recog_msgs](https://github.com/Sadaku1993/amsl_recog_msgs)

## How to Use

### Rosbag record
```
roscd depthimage_creater/scripts/bagrec
./run_savedata_joy.sh
./bag_rec_savedata_joy.sh
```
bag data is saved at here.
depthimage_creater_20181013.bag
depthimage_creater_20181024.bag
depthimage_creater_indoor_20190104.bag

### Object Detection
```
roslaunch depthimage_creater object_detection.launch
roscd depthimage_creater/scripts/bagrec
./bag_rec_map.sh
```
bag data name is .
20181018-sadakuni.bag
20181024-rm_human.bag


### Create Map
Require data
- nav_msgs/Odometry /odom
- sensor_msgs/Imu /imu/data
- sensor_msgs/PointCloud2 /cluster/human/removed
- sensor_msgs/PointCloud2 /cloud/lcl

Normal Estimation for Map data
```
roslaunch depthimage_creater normal_estimation_for_map.launch
```

### Data Saver
```
roscore
rosparam set use_sim_time true
roslaunch kari_complement add_height.launch
roslaunch depthimage_creater data_saver.launch
roslaunch depthimage_creater bag.launch
```

### Upsampling
```
roslaunch depthimage_creater polygon_mesh.launch
pcl_vtk2ply map.vtk map.ply
pcl_mesh_sampleing map.ply map_meshsample.pcd -n_sample 10000000 -leaf_size 0.0001
```

### DepthImage Creater
```
roslaunch depthimage_creater depthimage_creater.launch
```
