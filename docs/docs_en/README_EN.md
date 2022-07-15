<p align="center"><strong>DIABLO SDK</strong></p>
<p align="center"><a href="https://github.com/Direcrt-Drive-Technology/diablo-sdk-v1/blob/master/LICENSE"><img alt="License" src="https://img.shields.io/badge/License-Apache%202.0-orange"/></a>
<img alt="language" src="https://img.shields.io/badge/language-c++-red"/>
<img alt="platform" src="https://img.shields.io/badge/platform-linux-l"/>
</p>


<p align="center">
    语言：<a href="README.en.md"><strong>English</strong></a> / <strong>中文</strong>
</p>



​	基于串口通信的 `DIABLO` 机器人二次开发控制接口，您可以通过 `ROS2` 快速上手。如果您想要不通 `ROS` 进行开发，也可以在 [ROS](https://github.com/DDTRobot/diablo-sdk-v1) 中修改 `CMakeLists` 的方式只对源码进行编译。我们将不断的更新 `ROS2` 的功能节点 , 希望能对您的机器人开发有所帮助。

---

![diabloRender](../img/diablo_robot_render.png)

## Basic Information 

- `X3pi` The default user is `root`  and the password `root`

  > `Raspberry pi` The default user is `diablo`  and the password `diablo123`

- `X3pi` The default serial port number of IO is `/dev/ttyS3`

  > `Raspberry pi` The default serial port number of IO is `/dev/ttyAMA0`
  >
  > You can switch hardware by modifying [Hal.init("/dev/ttyS3")](./diablo_interaction/diablo_ctrl/src/diablo_ctrl.cpp) and recompiling

- `ROS_DOMAIN_ID=5` , You can use `export ROS_ DOMAIN_ Id=5` connect and control the `Diablo` function node in the LAN.



## Installation 

| Installation method | Supported platform[s] | Development Docs  | Official website                         |
| ------------------- | --------------------- | ----------------- | ---------------------------------------- |
| Source              | Linux , ros-foxy      | [DIABLO Manual]() | [Direct drive](https://directdrive.com/) |

You can compile our SDK source code in most `Linux` devices. Or directly compile the ROS package provided by us in the device that supports ROS foxy.



## Contents 

The following is the ros2 node Directory:

* [Robot ception node]()

  > [Robot carrier sensor]()

* [SDK source code and common fuction]()

* [Robot firing sdk and ctrl]()

  > [Start sdk ctrl]()
  >
  > [Capture keyboard event]()

* [Ros custom msgs]()

  > [Robot basic movement ctrl msg]()

* [Ros rviz2 and gazebo example]()

  > [rviz2 gazebo simulation]()
  >
  > [rviz2 Qt teleop UI]()
  >
  > [motor angle 2 urdf angle]()

  

## Quick Start 

1. Create ros project workspace

```bash
#make sure you have build all dependence.

sudo apt-get install python3-colcon-common-extensions
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws/src

#clone API source code
git clone -b basic https://github.com/DDTRobot/diablo_sdk_v2.git

cd ~/catkin_ws
colcon build
source install/setup.bash

#before starting the node , please check of serial port in diablo_ctrl.cpp is correct.
ros2 run diablo_ctrl diablo_ctrl_node

#run controller python script
ros2 run diablo_teleop teleop_node 
```

