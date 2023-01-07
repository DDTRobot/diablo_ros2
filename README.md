<p align="center"><strong>DIABLO ROS2</strong></p>
<p align="center"><a href="https://github.com/DDTRobot/diablo_sdk_v2/blob/main/LICENSE"><img alt="License" src="https://img.shields.io/badge/License-Apache%202.0-orange"/></a>
<img alt="language" src="https://img.shields.io/badge/language-c++-red"/>
<img alt="platform" src="https://img.shields.io/badge/platform-linux-l"/>
</p>


<p align="center">
    语言：<a href="./docs/docs_en/README_EN.md"><strong>English</strong></a> / <strong>中文</strong>
</p>



​	基于串口通信的 `DIABLO` 机器人二次开发控制接口，您可以通过 `ROS2` 快速上手。如果您想要不通 `ROS` 进行开发，也可以在 [ROS](https://github.com/DDTRobot/diablo-sdk-v1) 中修改 `CMakeLists` 的方式只对源码进行编译。我们将不断的更新 `ROS2` 的功能节点 , 希望能对您的机器人开发有所帮助。

---

![diablo_robot_render](./docs/img/diablo_robot_render.jpg)

## Basic Information 基本信息

- `X3pi` 默认用户是 `root` , 密码为 `root`

  > 树莓派中默认用户是 `diablo` ,密码为 `diablo123`

- `X3pi` 中IO默认串口号是 `/dev/ttyS3`

  > 树莓派中IO默认串口号是 `/dev/ttyAMA0`,如果您使用自定义的镜像，请[重新配置串口映射](https://diablo-sdk-docs.readthedocs.io/en/latest/pages/Installation/installing-sdk-on-pi.html)
  >
  > 您可以通过修改 [Hal.init("/dev/ttyS3")](./diablo_interaction/diablo_ctrl/src/diablo_ctrl.cpp) 并重新编译达到切换硬件的目的

- `ROS_DOMAIN_ID=5` , 可通过 `export ROS_DOMAIN_ID=5` 连接并控制局域网中 `DIABLO` 的功能节点。



## Installation 安装

| Installation method | Supported platform[s] | Development Docs    | Official website                         |
| ------------------- | --------------------- | ------------------- | ---------------------------------------- |
| Source              | Linux , ros-foxy      | [DIABLO 开发手册](https://diablo-sdk-docs.readthedocs.io/en/latest/index.html) | [Direct drive](https://directdrive.com/) |

您可以在大多数 `Linux` 设备中编译我们的 SDK 源码。或者在支持 ros-foxy 的设备中直接编译我们提供的 ros 包。


## Quick Start 快速开始

1. 创建ros工程文件夹

```bash
#make sure you have build all dependence.

sudo apt-get install python3-colcon-common-extensions
mkdir -p ~/diablo_ws/src
cd ~/diablo_ws/src

#clone API source code
git clone -b basic https://github.com/DDTRobot/diablo_ros2.git

cd ~/diablo_ws
colcon build
source install/setup.bash

#before starting the node , please check of serial port in diablo_ctrl.cpp is correct.
ros2 run diablo_ctrl diablo_ctrl_node

#run controller python script
ros2 run diablo_teleop teleop_node 
```

2. 完整版编译

```bash
#make sure you have build all dependence.

sudo apt-get install python3-colcon-common-extensions python3-pip
sudo pip3 install rosdep
sudo rosdep init
rosdep update
mkdir -p ~/diablo_ws/src
cd ~/diablo_ws/src

#clone API source code
git clone https://github.com/DDTRobot/diablo_ros2.git
cd ~/diablo_ws
rosdep install -i --from-path src --rosdistro foxy -y

colcon build
source install/setup.bash

#before starting the node , please check of serial port in diablo_ctrl.cpp is correct.
ros2 run diablo_ctrl diablo_ctrl_node

#run controller python script
ros2 run diablo_teleop teleop_node 
```

## Contents 目录

以下为Ros2 节点目录 :

* [机器人传感器感知模块](./diablo_ception)

  > [机器人内置传感器](./diablo_ception/diablo_body)

* [机器人SDK与通用方法模块](./diablo_common)

* [机器人控制交互模块](./diablo_interaction)

  > [获取机器人SDK控制权限](./diablo_interaction/diablo_ctrl)
  >
  > [捕获键盘输入信息](./diablo_interaction/diablo_teleop)

* [ROS自定义消息模块](./diablo_interfaces)

  > [机器人基础控制信息](./diablo_interfaces/motion_msgs)

* [Ros可视化仿真模块](./diablo_visualise)

  > [Ros rviz2 gazebo simulation](./diablo_visualise/diablo_simulation)
  >
  > [Rviz2 自定义遥控器界面](./diablo_visualise/diablo_rviz2_plugin)
  >
  > [电机角度转Rviz2显示角度](./diablo_visualise/diablo_simpose_trans)

  

