# Running ROS across multiple machines

ROS作为分布式操作系统，基于网络通信，只要在同一个局域网内，各节点可以运行在不同计算机平台上，本文档将详细描述如何在多个计算机上运行ROS。

在多机通信过程中，请记住如下几点：

- 只能有一个计算机作为master
- 所有节点计算机必须在同一个可以互相通信的局域网络

我们以一个示例讲解如何在同一个局域网的两台计算机上运行ROS，我们默认计算机的hostname与IP配置如下：

- 树莓派计算机：hostname=raspberrypi， IP=192.168.1.10
- 笔记本：hostname=alex，IP=192.168.1.20

## 配置

### 时间同步

使用*chrony*同步多个计算机的时间

```
sudo apt-get install chrony
```



### 配置/etc/hosts

查看主机名（hostname）

```
$hostname
```

/etc/network

查看本机IP

```
$ifconfig
```

通过修改*/etc/hosts*文件，将网络内的主机名与IP进行绑定，这样计算机就能够根据hostname进行IP地址的解析了，进行如下配置：

```
127.0.0.1     localhost
127.0.0.1     raspberrypi
192.168.1.10  raspberrypi
192.168.1.20  alex
```

配置完成后，即可进行相互通信测试：

1、笔记本与树莓派通信测试

在笔记本打开*cmd*窗口，运行*ping*命令：

```
$ ping raspberrypi
```

2、树莓派与笔记本通信测试

在树莓派打开*cmd*窗口，运行*ping*命令：

```
$ ping alex
```



### 配置~/.bashrc

我们将笔记本计算机作为master，

1、在笔记本上修改*~/.bashrc*文件，增加如下两行：

```
export ROS_HOSTNAME=alex
export ROS_MASTER_URI=http://alex:11311
```

2、在树莓派上修改*~/.bashrc*文件，增加如下两行：

```
export ROS_HOSTNAME=raspberrypi
export ROS_MASTER_URI=http://alex:11311
```



## 测试

### 小海龟测试

我们经使用小海龟测试示例进行两个计算机的ROS通信测试。

1、笔记本启动roscore

在笔记本打开*cmd*窗口，然后运行*roscore*：

```
$roscore &
```

2、笔记本启动小海龟节点

在笔记本打开新的*cmd*窗口，运行小海龟：

```
$rosrun turtlesim turtlesim_node
```

3、树莓派启动小海龟控制节点

```
$rosrun turtlesim turtle_teleop_key
```



## 远程控制

在实际应用场景中，不可能所有计算机都配置显示器，很可能只有笔记本有其他计算机都没有，那么如何在没有显示器的计算机上进行测试、运行程序呢？——答案是ssh。

ubuntu默认安装了ssh-client，如果没有安装ssh-server，则运行如下指令安装：

```
sudo apt-get install openssh-server
```

这样便可以使用笔记本ssh其他计算机了，在笔记本运行：

```
$ssh raspberrypi
```



