# MAVROS

- API: http://wiki.ros.org/mavros#Nodes

- Github: https://github.com/mavlink/mavros

## installation

### 安装mavros方式一：apt-get

```shell
# For ubuntu16.04 kinetic, use:
sudo apt-get install ros-kinetic-mavros ros-kinetic-mavros-extras
# For ubuntu18.04 melodic, use:
sudo apt-get install ros-melodic-mavros ros-melodic-mavros-extras
```

### 安装mavros方式二：源码

如果你需要进行二次开发，必然要修改源码，那么只能使用这种安装方式。

参考教程：

Ref:https://www.ncnynl.com/archives/201709/2077.html

Ref:https://blog.csdn.net/qq_38649880/article/details/88082360?utm_medium=distribute.pc_relevant_t0.none-task-blog-BlogCommendFromMachineLearnPai2-1.channel_param&depth_1-utm_source=distribute.pc_relevant_t0.none-task-blog-BlogCommendFromMachineLearnPai2-1.channel_param

### 安装GeographicLib

方式一：通过运行install_geographiclib_datasets.sh进行安装，注意这种方式需要在能连接其服务器时才行，很多时候因为网络问题并不能安装成功，所以不建议采用这种安装方式。
sudo /opt/ros/kinetic/lib/mavros/install_geographiclib_datasets.sh
方式二：下载缺少的GeographicLib文件，在没有安装GeographicLib之前你可以先运行一下mavros，可以看到弹出错误：即无法在路径`/usr/share/GeographicLib/geoids`下找到egm96-5.pgm文件，你可以从下面链接下载：
https://sourceforge.net/projects/geographiclib/files/geoids-distrib/
选择`egm96-5.tar.bz2`下载解压后拷贝至`/usr/share/GeographicLib/geoids`。

### install path

```
$rospack list
....
mavlink /opt/ros/melodic/share/mavlink
mavros /opt/ros/melodic/share/mavros
mavros_extras /opt/ros/melodic/share/mavros_extras
mavros_msgs /opt/ros/melodic/share/mavros_msgs
....
```



## 测试

### 启动mavros

在mavros/launch中给出了基本的launch示例，如果你希望操作PX4飞控，那么你可以启动px4.launch文件，并且同时需要制定连接飞控的方式，mavros提供了很多URL地址连接方式，你可以用来连接FCU以及GCS，具体命令格式如下：

- Serial: `/path/to/serial/device[:baudrate]`
- Serial: `serial:///path/to/serial/device[:baudrate][/?ids=sysid,compid]`
- Serial with hardware flow control: `serial-hwfc:///path/to/serial/device[:baudrate][?ids=sysid,compid]`
- UDP: `udp://[bind_host][:port]@[remote_host][:port][/?ids=sysid,compid]`
- UDP Broadcast: `udp-b://[bind_host][:port]@[:port][/?ids=sysid,compid]`
- TCP client: `tcp://[server_host][:port][/?ids=sysid,compid]`
- TCP server: `tcp-l://[bind_host][:port][/?ids=sysid,compid]`

使用示例如下：

```bash
roslaunch mavros px4.launch fcu_url:=/dev/ttyACM0:57600
#
roslaunch mavros px4.launch fcu_url:=/dev/ttyUSB0:921600
#
roslaunch mavros px4.launch fcu_url:="udp://:14540@127.0.0.1:14557"
```

- Note: if `FCU: DeviceError:serial:open: Permission denied` happened, use: **sudo chmod 777 /dev/ttyACM0

启动service，查看Imu相关数据

```
 $ rosservice call /mavros/set_stream_rate 0 10 1
 $ rostopic echo /mavros/imu/data
```



## 飞行平台配置

### PX4飞控设置

#### 串口设置

由于机载计算机进行飞行控制时需要很高的实时性，mavros会检测通信实时性，如果串口波特率设置太低，则会提示`TM:RTT too high for timesync:17.85ms`，故应该配置高速波特率。

#### 使用QGC设置

QGC连接飞控后，在参数中搜索**SER_TEL1_BAUD**，该参数用来设置TELEM1串口的波特率，选择**921600 8N1**，重启飞控生效。



飞控的`SYS_COMPANION` 参数设置为`Companion Link (921600 baud, 8N1)`



### 树莓派设置

树莓派3/4有两个串口：/dev/ttyAMA0和/dev/ttyS0，其中默认AMA0用于板载蓝牙，S0用于外设，其引脚对应关系为：

| 物理引脚BOARD | BCM编码 | 串口引脚 |
| ------------- | ------- | -------- |
| 8             | 14      | TX       |
| 10            | 15      | RX       |
|               |         |          |



## MAVSDK

The [MAVSDK](https://mavsdk.mavlink.io/develop/en/) is a [MAVLink](https://mavlink.io/en/) Library with APIs for [C++](https://mavsdk.mavlink.io/develop/en/cpp/), [iOS](http://dronecode-sdk-swift.s3.eu-central-1.amazonaws.com/docs/master/index.html), Python and Android.



### Imu

**File: sensor_msgs/Imu.msg**

```
std_msgs/Header header
geometry_msgs/Quaternion orientation
float64[9] orientation_covariance
geometry_msgs/Vector3 angular_velocity
float64[9] angular_velocity_covariance
geometry_msgs/Vector3 linear_acceleration
float64[9] linear_acceleration_covariance
```

**geometry_msgs/Vector3.msg**

```
float64 x
float64 y
float64 z
```

