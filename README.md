### swarm_transformation
## 使用说明

# 1. 打开gazebo仿真界面（gazebo文件需自己配置， 以pp_px4为例） 
```
cd ~/PX4-Autopilot 
source Tools/setup_gazebo.bash $(pwd) $(pwd)/build/px4_sitl_default 
export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:$(pwd) 
export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:$(pwd)/Tools/sitl_gazebo 
roslaunch px4 multi_uav_mavros_sitl_sdf.launch
```

# 2. 进入到affine_transformation文件 
```
catkin_make 
```
在编译过程中，如果有报错，则根据报错提示补充所需依赖文件。

# 3. roslaunch运行代码 
```
source ~/affine_tranformation/devel/setup.bash roslaunch camera_detect_offb affine_formation.launch
```

# 4.打开键盘控制输入代码 
```
rosrun camera_detect_offb commond_pub_node
```
