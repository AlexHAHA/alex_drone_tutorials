# offboard control



## mavros实现offboard

 mavros的本质是使用ROS实现对底层的mavlink通信的封装，也就是说你不需要关系mavlink协议如何实现，也不需要关心串口编程或者UDP/TCP编程如何实现，只需要会ROS即可。运行ROS的计算机会自动解析mavlink数据并按类型分后发布到相应的话题，用户只需要订阅话题即可获取飞控状态信息，同样的，用户需要实现的飞行控制时，只需要向相应话题发布msg或者调用相应service即可实现对飞控的控制指令发送。

在实际运行过程中，用户需要首先运行mavros主节点（通过调用mavros提供的launch文件即可实现），另外用户还需要运行自己根据飞行任务创建的节点。

<img src="C:\Users\admin\AppData\Roaming\Typora\typora-user-images\image-20201014133755544.png" alt="image-20201014133755544" style="zoom:35%;" />



mavros一般通过其自带的launch文件来启动，mavros提供了几个基本的launch文件，在mavros安装路径下：

```
/opt/ros/melodic/share/mavros/launch
---
apm.launch
px4.launch
mavlink_bridge.launch
```



px4.launch文件内容如下：

```html
<launch>
	<!-- vim: set ft=xml noet : -->
	<!-- example launch script for PX4 based FCU's -->

	<arg name="fcu_url" default="/dev/ttyACM0:57600" />
	<arg name="gcs_url" default="" />
	<arg name="tgt_system" default="1" />
	<arg name="tgt_component" default="1" />
	<arg name="log_output" default="screen" />
	<arg name="fcu_protocol" default="v2.0" />
	<arg name="respawn_mavros" default="false" />

	<include file="$(find mavros)/launch/node.launch">
		<arg name="pluginlists_yaml" value="$(find mavros)/launch/px4_pluginlists.yaml" />
		<arg name="config_yaml" value="$(find mavros)/launch/px4_config.yaml" />

		<arg name="fcu_url" value="$(arg fcu_url)" />
		<arg name="gcs_url" value="$(arg gcs_url)" />
		<arg name="tgt_system" value="$(arg tgt_system)" />
		<arg name="tgt_component" value="$(arg tgt_component)" />
		<arg name="log_output" value="$(arg log_output)" />
		<arg name="fcu_protocol" value="$(arg fcu_protocol)" />
		<arg name="respawn_mavros" default="$(arg respawn_mavros)" />
	</include>
</launch>
```

**px4.launch解析**

px4.launch set arg:config_yaml = px4_config.yaml and pluginlists_yaml = px4.pluginlists.yaml

px4.launch文件会调用node.launch文件，其路径为/opt/ros/melodic/share/mavros/launch。



## 仿真

### 仿真环境搭建

#### PX4_Firmware

```
PX4_Firmware/              -- WORKSPACE
|--Tools/                    
  |--sitl_gazebo/             -- used as package name: mavlink_sitl_gazebo($rospack list)
    |--models/                -- model definations
      |--iris/
        |--iris.sdf
        |--model.config
      |--M100/
      |--rotors_description/
        |--urdf/                           // 
          |--iris.xacro multirotor_base.xacro plane.xacro standard_vtol.xacro
        |--meshes/
        |--model.config                    //
```



### 运行

#### 启动PX4 gazebo仿真环境

在*PX4_Firmware/launch*中有很多launch用于启动基于gazebo的仿真环境，可以通过如下命令启动：

```
$ roslaunch px4 mavros_posix_sitl.launch
```

以上命令会打开一个gazebo窗口并且显示一个四旋翼无人机，同时会启动mavros，接下来就可以通过GCS或者编写程序来控制无人机了。

**启动流程的相关解释：**

mavros_posix_sitl.launch 调用`px4.launch(/opt/ros/melodic/share/mavros/launch) `

px4.launch进行参数配置，加载文件`px4_config.yaml` and `px4.pluginlists.yaml`

px4.launch 调用`node.launch(/opt/ros/melodic/share/mavros/launch)`



#### 设置无人机类型

~/PX4_Firmware/Tools/sitl_gazebo/models
~/PX4_Firmware/Tools/sitl_gazebo/models/rotors_description/urdf

availiable vehicle type: iris, standard_vtol, plane

```xml
<launch>
    <!-- vehicle type -->
    <arg name="vehicle" default="standard_vtol"/>
    ...
    <group ns="uav0">
    	<include file="$(find px4)/launch/single_vehicle_spawn.launch">
            <arg name="vehicle" value="$(arg vehicle)"/>
        </include>
    </group>
</launch>
```



## 多机

如果进行多机控制，那么需要区分每个无人机，获取其信息并发送控制指令，我们可以在`launch`文件中使用<group>标签来定义一个或多个命名空间，这样在每个命名空间下运行的mavros节点就可以相互独立。 

### 多机仿真环境启动

我们使用*PX4_Firmware/launch*中的*multi_uav_mavros_sitl.launch*文件启动多机的仿真环境，运行后可以生成名字为*uav0, uav1, uav2*三个mavros节点。

```
$ roslaunch px4 multi_uav_mavros_sitl.launch
... logging to /home/alex/.ros/log/9d888692-11e0-11eb-a921-e86a64796890/roslaunch-alex-31117.log
Checking log directory for disk usage. This may take a while.
Press Ctrl-C to interrupt
Done checking log file disk usage. Usage is <1GB.

xacro: in-order processing became default in ROS Melodic. You can drop the option.
xacro: in-order processing became default in ROS Melodic. You can drop the option.
xacro: in-order processing became default in ROS Melodic. You can drop the option.
started roslaunch server http://alex:38969/

SUMMARY
========

CLEAR PARAMETERS
 * /uav0/mavros/
 * /uav1/mavros/
 * /uav2/mavros/

PARAMETERS
 * /gazebo/enable_ros_network: True
 * /rosdistro: melodic
 * /rosversion: 1.14.7
 * /uav0/mavros/cmd/use_comp_id_system_control: False
 * /uav0/mavros/conn/heartbeat_rate: 1.0
 * /uav0/mavros/conn/system_time_rate: 1.0
...
...
 * /uav0/mavros/wheel_odometry/wheel1/x: 0.0
 * /uav0/mavros/wheel_odometry/wheel1/y: 0.15
 * /uav0/rotors_description: <?xml version="1....
 ...
 ...
 * /uav1/mavros/cmd/use_comp_id_system_control: False
 * /uav1/mavros/conn/heartbeat_rate: 1.0
 * /uav1/mavros/conn/system_time_rate: 1.0
 * /uav1/mavros/conn/timeout: 10.0
 * /uav1/mavros/conn/timesync_rate: 10.0
 * /uav1/mavros/distance_sensor/hrlv_ez4_pub/field_of_view: 0.0
 * /uav1/mavros/distance_sensor/hrlv_ez4_pub/frame_id: hrlv_ez4_sonar
 ...
 ...
 * /uav2/mavros/cmd/use_comp_id_system_control: False
 * /uav2/mavros/conn/heartbeat_rate: 1.0
 * /uav2/mavros/conn/system_time_rate: 1.0
 * /uav2/mavros/conn/timeout: 10.0
 * /uav2/mavros/conn/timesync_rate: 10.0
 ...
 ...
NODES
  /
    gazebo (gazebo_ros/gzserver)
    gazebo_gui (gazebo_ros/gzclient)
  /uav0/
    iris_0_spawn (gazebo_ros/spawn_model)
    mavros (mavros/mavros_node)
    sitl_0 (px4/px4)
  /uav1/
    iris_1_spawn (gazebo_ros/spawn_model)
    mavros (mavros/mavros_node)
    sitl_1 (px4/px4)
  /uav2/
    iris_2_spawn (gazebo_ros/spawn_model)
    mavros (mavros/mavros_node)
    sitl_2 (px4/px4)
process[uav0/iris_0_spawn-5]: started with pid [31173]
INFO  [px4] Creating symlink /home/alex/PX4_Firmware/ROMFS/px4fmu_common -> /home/alex/.ros/etc

______  __   __    ___ 
| ___ \ \ \ / /   /   |
| |_/ /  \ V /   / /| |
|  __/   /   \  / /_| |
| |     / /^\ \ \___  |
\_|     \/   \/     |_/

px4 starting.
...
...
INFO  [px4] Calling startup script: /bin/sh etc/init.d-posix/rcS 1
process[uav1/iris_1_spawn-8]: started with pid [31200]
process[uav1/mavros-9]: started with pid [31201]
process[uav2/sitl_2-10]: started with pid [31204]
INFO  [px4] Creating symlink /home/alex/PX4_Firmware/ROMFS/px4fmu_common -> /home/alex/.ros/etc

______  __   __    ___ 
| ___ \ \ \ / /   /   |
| |_/ /  \ V /   / /| |
|  __/   /   \  / /_| |
| |     / /^\ \ \___  |
\_|     \/   \/     |_/

px4 starting.

```



启动的gazebo窗口会出现3架无人机，如下图：

<img src="source\multidrone_sim_gazebo.PNG" style="zoom:30%;" />



launch文件定义了三个无人机对应的节点，依次为*/uav0/mavros，/uav1/mavros，/uav2/mavros*，例如通过查看节点的结果如下：

```
alex@alex:~$ rosnode list
/gazebo
/gazebo_gui
/rosout
/rqt_gui_py_node_32167
/uav0/mavros
/uav1/mavros
/uav2/mavros
alex@alex:~$ 

```

通过运行*rqt_graph*获取的结果如下：

<img src="source\multidrone_sim_node_graph.PNG" style="zoom:40%;" />

### 多机示例



```
<?xml version="1.0"?>
<launch>
    <!-- UAV0 -->
    <group ns="uav0">
        <!-- MAVROS and vehicle configs -->
        <arg name="fcu_url" default="/dev/ttyACM0:57600"/>

        <!-- MAVROS -->
        <include file="$(find mavros)/launch/px4.launch">
            <arg name="fcu_url" value="$(arg fcu_url)"/>
        </include>
    </group>
</launch>
```



## 实例演示

### Demo C++

本示例讲解如何使用ROS建立一个样例工程，用来控制无人机起飞，代码示例来自于[官网](https://dev.px4.io/master/en/ros/mavros_offboard.html)。

**建立ros工作空间**

创建工作空间，并且创建*offboard_takeoff*的package：

```bash
$cd ~/Desktop
$mkdir catkin_offboard
$cd catkin_offboard
$mkdir src
$cd src
$catkin_init_workspace
$catkin_create_pkg offboard_takeoff std_msgs mavros_msgs ros_py ros_cpp geometry_msgs
```

**在/src下添加程序**

在/src下新建文件offb_node.cpp，添加如下代码：

```c++
/**
 * @file offb_node.cpp
 * @brief Offboard control example node, written with MAVROS version 0.19.x, PX4 Pro Flight
 * Stack and tested in Gazebo SITL
 */

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>

mavros_msgs::State current_state;
void state_cb(const mavros_msgs::State::ConstPtr& msg){
    current_state = *msg;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "offb_node");
    ros::NodeHandle nh;

    ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>
            ("mavros/state", 10, state_cb);
    ros::Publisher local_pos_pub = nh.advertise<geometry_msgs::PoseStamped>
            ("mavros/setpoint_position/local", 10);
    ros::ServiceClient arming_client = nh.serviceClient<mavros_msgs::CommandBool>
            ("mavros/cmd/arming");
    ros::ServiceClient set_mode_client = nh.serviceClient<mavros_msgs::SetMode>
            ("mavros/set_mode");

    //the setpoint publishing rate MUST be faster than 2Hz
    ros::Rate rate(20.0);

    // wait for FCU connection
    while(ros::ok() && !current_state.connected){
        ros::spinOnce();
        rate.sleep();
    }

    geometry_msgs::PoseStamped pose;
    pose.pose.position.x = 0;
    pose.pose.position.y = 0;
    pose.pose.position.z = 2;

    //send a few setpoints before starting
    for(int i = 100; ros::ok() && i > 0; --i){
        local_pos_pub.publish(pose);
        ros::spinOnce();
        rate.sleep();
    }

    mavros_msgs::SetMode offb_set_mode;
    offb_set_mode.request.custom_mode = "OFFBOARD";

    mavros_msgs::CommandBool arm_cmd;
    arm_cmd.request.value = true;

    ros::Time last_request = ros::Time::now();

    while(ros::ok()){
        if( current_state.mode != "OFFBOARD" &&
            (ros::Time::now() - last_request > ros::Duration(5.0))){
            if( set_mode_client.call(offb_set_mode) &&
                offb_set_mode.response.mode_sent){
                ROS_INFO("Offboard enabled");
            }
            last_request = ros::Time::now();
        } else {
            if( !current_state.armed &&
                (ros::Time::now() - last_request > ros::Duration(5.0))){
                if( arming_client.call(arm_cmd) &&
                    arm_cmd.response.success){
                    ROS_INFO("Vehicle armed");
                }
                last_request = ros::Time::now();
            }
        }

        local_pos_pub.publish(pose);

        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}
```



**编译**

在CMakeLists.txt的*build*部分增加对文件*offb_node.cpp*文件的编译

```cmake
add_executable(offboard src/offb_node.cpp)
target_link_libraries(offboard ${catkin_LIBRARIES})
```

在工作空间下，运行*catkin_make*命令，编译并生成可执行程序。

**运行**

首先打开一个终端运行*mavros*：

```bash
$roslaunch mavros px4.launch fcu_url:=/dev/ttyUSB0:57600
```

再打开一个终端运行编译生成的可执行程序*offboard*：

```bash
$rosrun offboard_takeoff offboard
```



### Example-sim

In this example we will control drone fly in a circle while keep the drone heading the center of circle.

```python
import time
import sys
import rospy
from mavros_msgs.msg import State, PositionTarget
from mavros_msgs.srv import CommandBool, SetMode
from std_msgs.msg import Header

target_velocity = Twist()
target_raw = PositionTarget()

rospy.init_node("set_velocity", anonymous=True)
rospy.loginfo("Test setpoint_velocity")

service_timeout = 30
try:
    rospy.wait_for_service('mavros/cmd/arming', service_timeout)
    rospy.wait_for_service('mavros/set_mode', service_timeout)
except ROSException:
    rospy.loginfo("ROS exception")
srv_arm  = rospy.ServiceProxy("mavros/cmd/arming", CommandBool)
srv_setmode = rospy.ServiceProxy("mavros/set_mode", SetMode)
pub_local_raw = rospy.Publisher('mavros/setpoint_raw/local', 
                                PositionTarget, queue_size=1)
def ctrl_arm_disarm(arm_flag):
    res = srv_arm(arm_flag)
    rospy.loginfo("Arm ctrl result {}".format(res))

def ctrl_set_mode(mode, timeout=10):
    base_mode   = 0
    custom_mode = mode
    res = srv_setmode(base_mode, custom_mode)
    rospy.loginfo("Set mode {} res: {}".format(mode, res))

def velocity_ctrl_raw():
    """
    Topic: ~setpoint_raw/local
    Function: set velocity and yaw rate in local frame, the drone will fly in a circle!
    """
    # Header
    target_raw.header = Header()
    # set coordinate frame
    target_raw.coordinate_frame = target_raw.FRAME_BODY_NED
    # ignore position and acc/force ctrl
    target_raw.type_mask = target_raw.IGNORE_PX | target_raw.IGNORE_PY | target_raw.IGNORE_PZ | target_raw.IGNORE_YAW
    target_raw.type_mask += target_raw.IGNORE_AFX | target_raw.IGNORE_AFY | target_raw.IGNORE_AFZ
    # just ctrl velocity and yaw rate
    target_raw.velocity.x = 2.0
    target_raw.yaw_rate = 0.5
    rate = rospy.Rate(10)

    ctrl_arm_disarm(True)
    ctrl_set_mode('AUTO.TAKEOFF')
    time.sleep(15)
    ctrl_set_mode('OFFBOARD')
    while not rospy.is_shutdown():
        target_raw.header.stamp = rospy.Time.now()
        pub_local_raw.publish(target_raw)
        #ctrl_set_mode('OFFBOARD')
        rate.sleep()

if __name__ == '__main__':
    velocity_ctrl_raw()



```





## mavros package explaination

### parameter

```
res = rospy.get_param('mavros/setpoint_attitude/use_quaternion')
rospy.loginfo("Use quaternion flag:{}".format(res))
rospy.set_param('mavros/setpoint_attitude/use_quaternion', True)
```



### velocity control

#### setpoint_velocity

control the velocity in global frame

topic:

/mavros/setpoint_velocity/cmd_vel
/mavros/setpoint_velocity/cmd_vel_unstamped

| topic                                       | msg                        |                                                              |
| ------------------------------------------- | -------------------------- | ------------------------------------------------------------ |
| /mavros/setpoint_velocity/cmd_vel           | geometry_msgs/TwistStamped | std_msgs/Header header<br/>geometry_msgs/Twist twist<br/>   geometry_msgs/Vector3 linear<br/>   geometry_msgs/Vector3 angular<br/> |
| /mavros/setpoint_velocity/cmd_vel_unstamped | geometry_msgs/Twist        | geometry_msgs/Vector3 linear<br/>geometry_msgs/Vector3 angular<br/> |



#### setpoint_raw/local

Local position, velocity and acceleration setpoint. 

**Topic:**

`~setpoint_raw/local` ([mavros_msgs/PositionTarget](http://docs.ros.org/en/api/mavros_msgs/html/msg/PositionTarget.html)) 

**PositionTarget.msg Definition**

```
# Message for SET_POSITION_TARGET_LOCAL_NED
#
# Some complex system requires all feautures that mavlink
# message provide. See issue #402.

std_msgs/Header header

uint8 coordinate_frame
uint8 FRAME_LOCAL_NED = 1
uint8 FRAME_LOCAL_OFFSET_NED = 7
uint8 FRAME_BODY_NED = 8
uint8 FRAME_BODY_OFFSET_NED = 9

uint16 type_mask
uint16 IGNORE_PX = 1 # Position ignore flags
uint16 IGNORE_PY = 2
uint16 IGNORE_PZ = 4
uint16 IGNORE_VX = 8 # Velocity vector ignore flags
uint16 IGNORE_VY = 16
uint16 IGNORE_VZ = 32
uint16 IGNORE_AFX = 64 # Acceleration/Force vector ignore flags
uint16 IGNORE_AFY = 128
uint16 IGNORE_AFZ = 256
uint16 FORCE = 512 # Force in af vector flag
uint16 IGNORE_YAW = 1024
uint16 IGNORE_YAW_RATE = 2048

geometry_msgs/Point position
geometry_msgs/Vector3 velocity
geometry_msgs/Vector3 acceleration_or_force
float32 yaw
float32 yaw_rate
```

### attitude control





**topic:**

`~setpoint_raw/attitude` ([mavros_msgs/AttitudeTarget](http://docs.ros.org/en/api/mavros_msgs/html/msg/AttitudeTarget.html)) 

**msg definition:**

mavros_msgs/AttitudeTarget.msg



三个欧拉角，或者 4 元数

in px4_config.yaml:

```yaml
# setpoint_attitude
setpoint_attitude:
  reverse_thrust: false     # allow reversed thrust
  #before:
  #use_quaternion: false     # enable PoseStamped topic subscriber
  #after:
  use_quaternion: true     # enable PoseStamped topic subscriber
```





**topic:**

`~setpoint_raw/attitude` ([mavros_msgs/AttitudeTarget](http://docs.ros.org/en/api/mavros_msgs/html/msg/AttitudeTarget.html)) 

**msg definition:**

mavros_msgs/AttitudeTarget.msg

```
# Some complex system requires all feautures that mavlink
# message provide. See issue #402, #418.

std_msgs/Header header

uint8 type_mask
uint8 IGNORE_ROLL_RATE = 1 # body_rate.x
uint8 IGNORE_PITCH_RATE = 2 # body_rate.y
uint8 IGNORE_YAW_RATE = 4 # body_rate.z
uint8 IGNORE_THRUST = 64
uint8 IGNORE_ATTITUDE = 128 # orientation field

geometry_msgs/Quaternion orientation
geometry_msgs/Vector3 body_rate
float32 thrust
```







```
quaternion = tf.transformations.quaternion_from_euler(roll, pitch, yaw)
#type(pose) = geometry_msgs.msg.Pose
pose.orientation.x = quaternion[0]
pose.orientation.y = quaternion[1]
pose.orientation.z = quaternion[2]
pose.orientation.w = quaternion[3]

#type(pose) = geometry_msgs.msg.Pose
quaternion = (
    pose.orientation.x,
    pose.orientation.y,
    pose.orientation.z,
    pose.orientation.w)
euler = tf.transformations.euler_from_quaternion(quaternion)
roll = euler[0]
pitch = euler[1]
yaw = euler[2]
```





```
//! Old: A simple low pass filter implementation
// Taken from http://en.wikipedia.org/wiki/Low-pass_filter
double lowPassFilter(double x, double y0, double dt, double T) 
{
    double res = y0 + (x - y0) * (dt_/(dt_+T_));
    return res;
}
```

