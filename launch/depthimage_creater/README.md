# depthimage_creater

## How to Use

``` save_data
roscore
rosparam set use_sim_time true
roslaunch kari_complement add_height.launch
roslaunch depthimage_creater data_saver.launch
roslaunch depthimage_creater bag.launch
```

``` depthimage creater
roslaunch depthimage_creater depthimage_creater.launch
```
