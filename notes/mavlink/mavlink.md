# MAVLink

## MAVLink Basic

### Packet Format

Packet format定义了mavlink通信协议的数据包格式，打包与解析都需要按照该协议格式进行。

#### mavlink 1

<img src="source\packet_mavlink_v1.jpg" style="zoom:60%;" />

| Byte Index     | C version                  | Content                                                      | Value   | Explanation                                                  |
| -------------- | -------------------------- | ------------------------------------------------------------ | ------- | ------------------------------------------------------------ |
| 0              | `uint8_t magic`            | Packet start marker                                          | 0xFE    | Protocol-specific start-of-text (STX) marker used to indicate the beginning of a new packet. Any system that does not understand protocol version will skip the packet. |
| 1              | `uint8_t len`              | Payload length                                               | 0 - 255 | Indicates length of the following `payload` section (fixed for a particular message). |
| 2              | `uint8_t seq`              | Packet sequence number                                       | 0 - 255 | Used to detect packet loss. Components increment value for each message sent. |
| 3              | `uint8_t sysid`            | System ID                                                    | 1 - 255 | ID of *system* (vehicle) sending the message. Used to differentiate systems on network. Note that the broadcast address 0 may not be used in this field as it is an invalid *source* address. |
| 4              | `uint8_t compid`           | Component ID                                                 | 1 - 255 | ID of *component* sending the message. Used to differentiate components in a *system* (e.g. autopilot and a camera). Use appropriate values in [MAV_COMPONENT](https://mavlink.io/en/messages/common.html#MAV_COMPONENT). Note that the broadcast address `MAV_COMP_ID_ALL` may not be used in this field as it is an invalid *source* address. |
| 5              | `uint8_t msgid`            | Message ID                                                   | 0 - 255 | ID of *message type* in payload. Used to decode data back into message object. |
| 6 to (n+6)     | `uint8_t payload[max 255]` | Payload data                                                 |         | Message data. Content depends on message type (i.e. Message ID). |
| (n+7) to (n+8) | `uint16_t checksum`        | [Checksum](https://mavlink.io/en/guide/serialization.html#checksum) (low byte, high byte) |         | X.25 CRC for message (excluding `magic` byte). Includes [CRC_EXTRA](https://mavlink.io/en/guide/serialization.html#crc_extra) byte. |

#### mavlink 2

<img src="source\packet_mavlink_v2.jpg" style="zoom:80%;" />

| Byte Index       | C version                  | Content                                                      | Value        | Explanation                                                  |
| ---------------- | -------------------------- | ------------------------------------------------------------ | ------------ | ------------------------------------------------------------ |
| 0                | `uint8_t magic`            | Packet start marker                                          | 0xFD         | Protocol-specific start-of-text (STX) marker used to indicate the beginning of a new packet. Any system that does not understand protocol version will skip the packet. |
| 1                | `uint8_t len`              | Payload length                                               | 0 - 255      | Indicates length of the following `payload` section. This may be affected by [payload truncation](https://mavlink.io/en/guide/serialization.html#payload_truncation). |
| 2                | `uint8_t incompat_flags`   | [Incompatibility Flags](https://mavlink.io/en/guide/serialization.html#incompat_flags) |              | Flags that must be understood for MAVLink compatibility (implementation discards packet if it does not understand flag). |
| 3                | `uint8_t compat_flags`     | [Compatibility Flags](https://mavlink.io/en/guide/serialization.html#compat_flags) |              | Flags that can be ignored if not understood (implementation can still handle packet even if it does not understand flag). |
| 4                | `uint8_t seq`              | Packet sequence number                                       | 0 - 255      | Used to detect packet loss. Components increment value for each message sent. |
| 5                | `uint8_t sysid`            | System ID (sender)                                           | 1 - 255      | ID of *system* (vehicle) sending the message. Used to differentiate systems on network. Note that the broadcast address 0 may not be used in this field as it is an invalid *source* address. |
| 6                | `uint8_t compid`           | Component ID (sender)                                        | 1 - 255      | ID of *component* sending the message. Used to differentiate *components* in a *system* (e.g. autopilot and a camera). Use appropriate values in [MAV_COMPONENT](https://mavlink.io/en/messages/common.html#MAV_COMPONENT). Note that the broadcast address `MAV_COMP_ID_ALL` may not be used in this field as it is an invalid *source* address. |
| 7 to 9           | `uint32_t msgid:24`        | Message ID (low, middle, high bytes)                         | 0 - 16777215 | ID of *message type* in payload. Used to decode data back into message object. |
| 10 to (n+10)     | `uint8_t payload[max 255]` | [Payload](https://mavlink.io/en/guide/serialization.html#payload) |              | Message data. Depends on message type (i.e. Message ID) and contents. |
| (n+11) to (n+12) | `uint16_t checksum`        | [Checksum](https://mavlink.io/en/guide/serialization.html#checksum) (low byte, high byte) |              | X.25 CRC for message (excluding `magic` byte). Includes [CRC_EXTRA](https://mavlink.io/en/guide/serialization.html#crc_extra) byte. |
| (n+12) to (n+26) | `uint8_t signature[13]`    | [Signature](https://mavlink.io/en/guide/message_signing.html) |              | (Optional) Signature to ensure the link is tamper-proof.     |

- The minimum packet length is 11 bytes for acknowledgment packets without payload.
- The maximum packet length is 279 bytes for a signed message that uses the whole payload.

### Payload Format

获取了mavlink数据包后，我们可以通过packet format定义的协议格式提取一个完整的mavlink包，也就是可以获取到mavlink包的len、sysid、compid以及msgid和payload等等。那如何理解这个包发送的内容到底是代表什么意思呢？这个时候就需要根据msgid对应的payload format去解析mavlink包“装载”的信息是什么了。

简单来说，payload就是信息，我们需要根据msgid对应的数据结构去解析这个信息，msgid对应的数据结构定义是由.xml文件定义的，根据不同的.xml文件可以由mavgen工具生成对应编程语言的库文件，编程时可以调用库函数自动解析不同msgid对应的数据结构。

MAVLink does not include information about the message structure in the payload itself (in order to reduce overhead)! Instead the sender and receiver must share a common understanding of the meaning, order and size of message fields in the over-the-wire format.

Messages are encoded within the MAVLink packet:

1) The `msgid` (message id) field identifies the specific message encoded in the packet.

2) The payload field contains the message data.

- MAVLink [reorders the message fields](https://mavlink.io/en/guide/serialization.html#field_reordering) in the payload for over-the-wire transmission (from the order in the original [XML Message Definitions](https://mavlink.io/en/messages/)).
- MAVLink 2 [truncates](https://mavlink.io/en/guide/mavlink_2.html#packet_truncation) any zero-filled bytes at the end of the payload before the message is sent and sets the packet `len` field appropriately (MAVLink 1 always sends all bytes).

3) The `len` field contains the length of the payload data.

4) A [CRC_EXTRA](https://mavlink.io/en/guide/serialization.html#crc_extra) byte is added to the message [checksum](https://mavlink.io/en/guide/serialization.html#checksum). A receiver can use this to confirm that it is compatible with the payload message format/definition.



### Common Message Set

一个完整的mavlink消息机制可包括如下三个部分的概念：

- Mavlink type enumerations

包含了所有需要用到的枚举类型。

- Mavlink commands(MAV_CMD)

包含了关于MAV_CMD相关的枚举类型定义，由于command内容较多，所以单独拿出来说明。当mavlink包的msgid=76(COMMAND_LONG)时，会用到MAV_CMD的枚举类型。

- Mavlink message

定义了所有msgid对应的数据结构。

#### XML

一个.xml文件的根字段为<mavlink>，包含了两个主要字段：<enums>和<messages>，整个文件的的具体格式内容如下：

```xml
<?xml version="1.0"?>
<mavlink>
<enums>
  <enum name="MAV_COMPONENT">
    <entry value="0" name="MAV_COMP_ID_ALL">
      <description>Target id (target_component)...</description>
    </entry>
    <entry value="1" name="MAV_COMP_ID_AUTOPILOT1">
      <description>System flight controller component...</description>
    </entry>
    // other entrys
    ....
    ....
  </enum>
    
  // MAV_CMD  
  <enum name="MAV_CMD">
    <entry value="16" name="MAV_CMD_NAV_WAYPOINT" hasLocation="true" isDestination="true">
      <description>Navigate to waypoint.</description>
      <param index="1" label="Hold" units="s" minValue="0">Hold time.</param>
      <param index="2" label="Accept Radius" units="m" minValue="0"></param>
      <param index="3" label="Pass Radius" units="m"></param>
      <param index="4" label="Yaw" units="deg"></param>
      <param index="5" label="Latitude">Latitude</param>
      <param index="6" label="Longitude">Longitude</param>
      <param index="7" label="Altitude" units="m">Altitude</param>
    </entry>
    // other entrys
    ....
    ....
  </enum>
  //
  <enum name="MAV_CMD_ACK">
    <entry name="MAV_CMD_ACK_OK">
        <description>Command / mission item is ok.</description>
    </entry>
    <entry name="MAV_CMD_ACK_ERR_FAIL">
      <description>Generic error message .....</description>
    </entry>
    // other entrys
    ....
    ....
  </enum>
  
</enums>

<messages>
  //msgid=0
  <message id="0" name="HEARTBEAT">
    <description>The heartbeat message shows that...</description>
    <field type="uint8_t" name="type" enum="MAV_TYPE">Vehicle </field>
    <field type="uint8_t" name="autopilot" enum="MAV_AUTOPILOT">Auto...  </field>
    <field type="uint8_t" name="base_mode" enum="MAV_MODE_FLAG">Sys...   </field>
    <field type="uint32_t" name="custom_mode">A bitfield for use...      </field>
    <field type="uint8_t" name="system_status" enum="MAV_STATE">System...</field>
    <field type="uint8_t_mavlink_version" name="mavlink_version">MAVLink...</field>
  </message>
    
  <message id="76" name="COMMAND_LONG">
    <description>Send a command with up to seven parameters...</description>
    <field type="uint8_t" name="target_system">System which should... </field>
    <field type="uint8_t" name="target_component">Component which ... </field>
    <field type="uint16_t" name="command" enum="MAV_CMD">Command...   </field>
    <field type="uint8_t" name="confirmation">0: First transmission...</field>
    <field type="float" name="param1">Parameter 1</field>
    <field type="float" name="param2">Parameter 2</field>
    <field type="float" name="param3">Parameter 3</field>
    <field type="float" name="param4">Parameter 4</field>
    <field type="float" name="param5">Parameter 5</field>
    <field type="float" name="param6">Parameter 6</field>
    <field type="float" name="param7">Parameter 7</field>
  </message>
  // other message ids
  ....
  ....
</messages>
</mavlink>
```



#### Message example

##### HEARTBEAT

```XML
<message id="0" name="HEARTBEAT">
    <description>The heartbeat message shows that a system or component is present and responding. The type and autopilot fields (along with the message component id), allow the receiving system to treat further messages from this system appropriately (e.g. by laying out the user interface based on the autopilot). This microservice is documented at https://mavlink.io/en/services/heartbeat.html</description>
    <field type="uint8_t" name="type" enum="MAV_TYPE">Vehicle or component type. For a flight controller component the vehicle type (quadrotor, helicopter, etc.). For other components the component type (e.g. camera, gimbal, etc.). This should be used in preference to component id for identifying the component type.</field>
    <field type="uint8_t" name="autopilot" enum="MAV_AUTOPILOT">Autopilot type / class. Use MAV_AUTOPILOT_INVALID for components that are not flight controllers.</field>
    <field type="uint8_t" name="base_mode" enum="MAV_MODE_FLAG" display="bitmask">System mode bitmap.</field>
    <field type="uint32_t" name="custom_mode">A bitfield for use for autopilot-specific flags</field>
    <field type="uint8_t" name="system_status" enum="MAV_STATE">System status flag.</field>
    <field type="uint8_t_mavlink_version" name="mavlink_version">MAVLink version, not writable by user, gets added by protocol because of magic data type: uint8_t_mavlink_version</field>
</message>
```

生成的**mavlink_msg_heartbeat.h**主要包括了如下几个部分：

- 数据结构体

  **mavlink_heartbeat_t**

- 打包函数

  **mavlink_msg_heartbeat_pack**或者**mavlink_msg_heartbeat_encode**

- 解析函数

  **mavlink_msg_heartbeat_decode**

```c
#define MAVLINK_MSG_ID_HEARTBEAT 0

typedef struct __mavlink_heartbeat_t {
     uint32_t custom_mode; /*<  A bitfield for use for autopilot-specific flags*/
     uint8_t type; 
     uint8_t autopilot; 
     uint8_t base_mode; /*<  System mode bitmap.*/
     uint8_t system_status; /*<  System status flag.*/
     uint8_t mavlink_version; /*<  MAVLink version, not writable by user, gets added by protocol because of magic data type: uint8_t_mavlink_version*/
} mavlink_heartbeat_t;

static inline uint16_t mavlink_msg_heartbeat_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,uint8_t type, uint8_t autopilot, uint8_t base_mode, uint32_t custom_mode, uint8_t system_status)
{
}
static inline uint16_t mavlink_msg_heartbeat_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_heartbeat_t* heartbeat)
{
    return mavlink_msg_heartbeat_pack(system_id, component_id, msg, heartbeat->type, heartbeat->autopilot, heartbeat->base_mode, heartbeat->custom_mode, heartbeat->system_status);
}

static inline void mavlink_msg_heartbeat_decode(const mavlink_message_t* msg, mavlink_heartbeat_t* heartbeat)
{
}
```



##### COMMAND_LONG

```xml
<message id="76" name="COMMAND_LONG">
    <description>Send a command with up to seven parameters to the MAV. The command microservice is documented at https://mavlink.io/en/services/command.html</description>
    <field type="uint8_t" name="target_system">System which should execute the command</field>
    <field type="uint8_t" name="target_component">Component which should execute the command, 0 for all components</field>
    <field type="uint16_t" name="command" enum="MAV_CMD">Command ID (of command to send).</field>
    <field type="uint8_t" name="confirmation">0: First transmission of this command. 1-255: Confirmation transmissions (e.g. for kill command)</field>
    <field type="float" name="param1">Parameter 1 (for the specific command).</field>
    <field type="float" name="param2">Parameter 2 (for the specific command).</field>
    <field type="float" name="param3">Parameter 3 (for the specific command).</field>
    <field type="float" name="param4">Parameter 4 (for the specific command).</field>
    <field type="float" name="param5">Parameter 5 (for the specific command).</field>
    <field type="float" name="param6">Parameter 6 (for the specific command).</field>
    <field type="float" name="param7">Parameter 7 (for the specific command).</field>
</message>
```



### How To Use

#### Generate headers

首先下载[mavlink](https://github.com/mavlink/mavlink/)仓库代码，MAVLink仓库代码提供了GUI或者命令行两种方式生成不同语言的”Header"，最方便的就是运行GUI，可以通过如下命令行：

```
$python mavgenerate.py
```

接下来根据GUI界面提示即可。

#### Code in python

```python
import time
import socket

from mavlink import *
"""
    向GCS发送Heartbeat消息并接收GCS发送的Heartbeat消息
"""
class Device(object): 
    """
    创建设备文件，作为file参数传入MAVLink类，由于MAVLink发送数据使用file.write()，故Device需要将设备发送函数映射至write()。
    """
    def __init__(self, device_ip, device_port = 14550):
        #udp协议
        self.server      = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)  
        self.device_ip   = device_ip     # gcs(目标) IP
        self.device_port = device_port # gcs(目标) 端口
        self.device_address = (self.device_ip, self.device_port)
    
    def write(self, data_send): 
        # mavlink对象调用此函数发送数据
        sent_callback = self.server.sendto(data_send, self.device_address)

def main():
    local_addr = ("127.0.0.1", 14551)
    gcs_addr   = ("127.0.0.1", 14550)
    sock       = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    # 绑定本地端口，方便接收外部数据
    sock.bind(local_addr)
    dev = Device(gcs_addr[0], gcs_addr[1])
    drone = MAVLink(dev, 1, 200)

    for i in range(10):
        print(f">>> Communite loop {i}th")
        print(f"send heartbeat msg")
        drone.heartbeat_send(MAV_TYPE_HELICOPTER, MAV_AUTOPILOT_GENERIC, MAV_MODE_GUIDED_ARMED, 0, MAV_STATE_ACTIVE)
        # 接收消息
        data, addr = sock.recvfrom(1024)
        if len(data) > 0:
            result = drone.parse_char(data)
            # 判断msg的类型
            if isinstance(result, MAVLink_heartbeat_message):
                print(f"received heartbeat: sys={result.get_srcSystem()}, cmp={result.get_srcComponent()}")
        time.sleep(1)
if __name__ == '__main__':
    main()
```



### Display In HTML

在mavlink\doc文件夹下给出了两种生成HTML的方法。

1. php: 可以使用PHP study创建一个网页，将doc下的内容放到该网页对应的本地文件夹中。

   修改一下mavlink.php文件内容：

```
# 将common.xml文件拷贝到网页对应的本地文件夹，并指定下文件路径
$xml_file_name = "./common.xml";
# mavlink_to_html_table.xsl已经存在，重新指定下路径即可
$xsl_file_name= "./mavlink_to_html_table.xsl";
```

​      在php study的设置菜单，选择“配置文件”->php.ini，双击下面的php7.3.4.nts即可打开php.ini文件，然后找  到**;extension=xsl**并去掉**;**（即去掉注释）

2. python: 运行doc/mavlink_gitbook.py文件即可，默认在doc文件夹下生成messages文件夹，存放生成后的html文件。



## MAVROS

- API: http://wiki.ros.org/mavros#Nodes

- Github: https://github.com/mavlink/mavros

### installation

#### 安装mavros方式一：apt-get

```shell
# For ubuntu16.04 kinetic, use:
sudo apt-get install ros-kinetic-mavros ros-kinetic-mavros-extras
# For ubuntu18.04 melodic, use:
sudo apt-get install ros-melodic-mavros ros-melodic-mavros-extras
```

#### 安装mavros方式二：源码

如果你需要进行二次开发，必然要修改源码，那么只能使用这种安装方式。

参考教程：

Ref:https://www.ncnynl.com/archives/201709/2077.html

Ref:https://blog.csdn.net/qq_38649880/article/details/88082360?utm_medium=distribute.pc_relevant_t0.none-task-blog-BlogCommendFromMachineLearnPai2-1.channel_param&depth_1-utm_source=distribute.pc_relevant_t0.none-task-blog-BlogCommendFromMachineLearnPai2-1.channel_param

#### 安装GeographicLib

方式一：通过运行install_geographiclib_datasets.sh进行安装，注意这种方式需要在能连接其服务器时才行，很多时候因为网络问题并不能安装成功，所以不建议采用这种安装方式。
sudo /opt/ros/kinetic/lib/mavros/install_geographiclib_datasets.sh
方式二：下载缺少的GeographicLib文件，在没有安装GeographicLib之前你可以先运行一下mavros，可以看到弹出错误：即无法在路径`/usr/share/GeographicLib/geoids`下找到egm96-5.pgm文件，你可以从下面链接下载：
https://sourceforge.net/projects/geographiclib/files/geoids-distrib/
选择`egm96-5.tar.bz2`下载解压后拷贝至`/usr/share/GeographicLib/geoids`。

### 测试

启动mavros

```
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

