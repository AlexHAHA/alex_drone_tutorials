



### ~/.bashrc

set environment 

```bash
source ~/Desktop/catkin_offboard/devel/setup.bash(optional, your ros project)
source ~/PX4_Firmware/Tools/setup_gazebo.bash ~/PX4_Firmware/ ~/PX4_Firmware/build/px4_sitl_default
export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:~/PX4_Firmware
export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:~/PX4_Firmware/Tools/sitl_gazebo
```

make sure all the **export** is behind **source**!

```bash
alex@alex:~$ source ~/.bashrc
GAZEBO_PLUGIN_PATH :/home/alex/PX4_Firmware/build/px4_sitl_default/build_gazebo:/home/alex/PX4_Firmware/build/px4_sitl_default/build_gazebo
GAZEBO_MODEL_PATH :/home/alex/PX4_Firmware//Tools/sitl_gazebo/models:/home/alex/PX4_Firmware//Tools/sitl_gazebo/models
LD_LIBRARY_PATH /home/alex/Desktop/cetcs_onboard_application/devel/lib:/home/alex/Desktop/catkin_offboard/devel/lib:/opt/ros/melodic/lib:/home/alex/PX4_Firmware/build/px4_sitl_default/build_gazebo:/home/alex/PX4_Firmware/build/px4_sitl_default/build_gazebo
```



## Problems

### cannot find px4 package

when you run **roslaunch px4 mavros_posix_sitl.launch** you may get error with:

*RLException: [indoor1.launch] is neither a launch file nor is [px4] a launch file*

Resolve: you must put all **source** before **export**!