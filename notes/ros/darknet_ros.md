# darknet ros

## 概述

本教程详细讲解如何将在ROS下部署darknet-yolo。

## darknet

darknet官方网站：https://pjreddie.com/darknet/yolo/

### 编译

编译前需要对`MakeFile`进行适应性修改：

### 运行

1、检测图片

```
./darknet detect cfg/yolov3-tiny.cfg yolov3-tiny.weights data/dog.jpg
```

2、使用USB摄像头

```
sudo ./darknet detector demo cfg/coco.data cfg/yolov3.cfg yolov3.weights
```

**Note:** 

- 在TX2上一定要用sudo才能正常获取检测结果。
- TX2上运行yolov3可能有点吃力（显存小了些），建议运行yolov3-tiny。

## darknet_ros[ETH Zurich]

在GitHub上关于如何在ROS下部署darknet最好的开源项目是ETH Zurich **[Robotic Systems Lab](http://www.rsl.ethz.ch/)**，该项目的开发环境是Ubuntu18.04，ROS版本是Melodic，不过我用的是Ubuntu16.04，ROS版本Kinetic。

### Project explanation

```python
darknet_ros/
  |-darknet/                        # pjreddie darknet source files
  |-darknet_ros/
    |-config/
      |-ros.yaml                    # 'main' yaml file, define topic and action
      |-yolov2.yaml                 # 'user' yaml file, config by user
    |-include/
    |-launch/
      |-darknet_ros.launch                # launch file
    |-src/
      |-yolo_object_detector_node.cpp     # 'main' node to run
      |-YoloObjectDetector.cpp            # YoloObjectDetector
    |-yolo_network_config/                # cfg file and weights file
  |-darknet_ros_msgs/
    |-action/
    |-msg/
      |-BoundingBox.msg       # bbox data structure
      |-BoundingBoxes.msg     # detection results, array<BoundingBox>
      |-ObjectCount.msg       # number of detected object
```

#### ros.yaml

这个文件主要定义了订阅的图像topic的名字以及检测结果发布的topic名字。

在**darknet_ros**节点运行后，生成的topic有：

```
/darknet_ros/bounding_boxes
/darknet_ros/check_for_objects/cancel
/darknet_ros/check_for_objects/feedback
/darknet_ros/check_for_objects/goal
/darknet_ros/check_for_objects/result
/darknet_ros/check_for_objects/status
/darknet_ros/detection_image
/darknet_ros/found_object
```

生成的msg有：

```
darknet_ros_msgs/CheckForObjectsAction
darknet_ros_msgs/CheckForObjectsActionFeedback
darknet_ros_msgs/CheckForObjectsActionGoal
darknet_ros_msgs/CheckForObjectsActionResult
darknet_ros_msgs/CheckForObjectsFeedback
darknet_ros_msgs/CheckForObjectsGoal
darknet_ros_msgs/CheckForObjectsResult
darknet_ros_msgs/BoundingBox
darknet_ros_msgs/BoundingBoxes
darknet_ros_msgs/ObjectCount
```



#### darknet_ros.launch

1、图像topic

darknet_ros节点订阅的图像msg的topic名字为`/camera/rgb/image_raw`。

```
  <!-- Console launch prefix -->
  <arg name="launch_prefix" default=""/>
  <arg name="image" default="/camera/rgb/image_raw" />
```

### 使用

#### 示例1：图片检测

本示例将创建一个发布节点用于发布一张本地图片送至`darknet_ros`，并创建一个订阅节点获取检测结果

1、新建一个publisher节点

在与darknet_ros同样的workspace下建立一个新的package(alex_ros)，并建立一个`image_publisher.cpp`读取本地图片并发布

```
$catkin_create_pkg alex_ros roscpp std_msgs cv_bridge image_transport sensor_msgs
```

`image_publisher.cpp`内容如下：

```c++
#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "image_publisher");
    ros::NodeHandle nh;
    image_transport::ImageTransport img_trans(nh);
    // 一定注意，这里发布的topic名为/camera/rgb/image_raw
    image_transport::Publisher pub = img_trans.advertise("/camera/rgb/image_raw", 1);
    
    cv::Mat image = cv::imread("/home/dji/Desktop/onboard_test/catkin_ws_opencv/src/image_topic/src/dog.jpg", CV_LOAD_IMAGE_COLOR);
    ROS_INFO("load image success, width=%d, height=%d", image.cols, image.rows);
    cv::imshow("dog", image);
    cv::waitKey(10);
    cv::destroyWindow("dog");

    sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", image).toImageMsg();
    
    ros::Rate loop_rate(1);
    while(nh.ok())
    {
        pub.publish(msg);
        ros::spinOnce();
        loop_rate.sleep();
    }
}

```

在CMakeLists.txt中增加

```
find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})

add_executable(image_publisher src/image_publisher.cpp)
target_link_libraries(image_publisher
  ${catkin_LIBRARIES}
  ${OpenCV_LIBS}
```

2、创建一个订阅节点

在`alex_ros`中增加一个`show_detectresult.cpp`用于订阅检测结果。

```c++
#include <ros/ros.h>
#include "darknet_ros_msgs/ObjectCount.h"

void detectCallback(const darknet_ros_msgs::ObjectCount::ConstPtr& msg)
{
    ROS_INFO("DetectionResult: object count is %d", msg->count);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "detect_subscriber");
    ros::NodeHandle nh;

    ros::Subscriber det_oc_pub = nh.subscribe("/darknet_ros/found_object", 1, detectCallback);

    ros::spin();
    return 0;
}
```

在CMakeLists.txt中增加

```cmake
find_package(catkin REQUIRED COMPONENTS
  ...
  #必须要包含darknet_ros package才能获取相关message的头文件
  darknet_ros
)
add_executable(show_detectresult src/show_detectresult.cpp)
target_link_libraries(show_detectresult ${catkin_LIBRARIES})
```

3、运行

在一个终端下运行darknet_ros

```
$sudo -i
# cd /home/dji/catkin_ws
# sudo gedit ~/.bashrc
# 在~/.bashrc文件中最后一行添加source /opt/ros/kinetic/setup.bash
# source ~/.bashrc
# source devel/setup.bash
# roslaunch darknet_ros darknet_ros.launch
```

在一个终端下运行image_publisher

```
$sudo -i
# cd /home/dji/catkin_ws
# sudo gedit ~/.bashrc
# 在~/.bashrc文件中最后一行添加source /opt/ros/kinetic/setup.bash
# source ~/.bashrc
# source devel/setup.bash
# roslaunch alex_ros image_publisher
```

#### 使用USB摄像头测试

1、运行`usb_cam`

```
rosrun usb_cam usb_cam_node
```

可能需要提前安装一下`usb_cam`包，运行命令`sudo apt-get install ros-kinetic-usb-cam`

2、运行darknet_ros

```
$sudo -i
# cd /home/dji/catkin_ws
# sudo gedit ~/.bashrc
# 在~/.bashrc文件中最后一行添加source /opt/ros/kinetic/setup.bash
# source ~/.bashrc
# source devel/setup.bash
# roslaunch darknet_ros darknet_ros.launch
```

## 问题及解决

在ubuntu下使用darknet时，可能会遇到很多问题，下面将基本遇到的问题进行描述。

### cfg文件格式不对

- 描述

错误信息如下：

`First section must be [net] or [network]: No such file or directory`

- 解决方法

使用vi打开cfg文件，然后修改其格式：

```
$ vi yolov2.cfg
--------------------------
:set ff=unix   #修改格式
按回车键
:wq            #保存退出
```

### 图片存在却无法读取问题

- 描述

 Cannot load image "image_path/image.jpg"，类似这样的，明明图片存在却无法读取问题

- 解决方法

可能是train.txt之类的文件格式不对，参考前面，将格式修改为`unix`。

### CUDA Error

- 描述

CUDA Error: __global__ function call is not configured

- 解决

在运行`darknet_ros`之前，使用`sudo -i`命令获取管理员权限，然后再运行darknet_ros。