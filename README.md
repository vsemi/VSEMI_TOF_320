# VSemi ToF Camera

[TOC]

## 1 Quick Start

### 1.1 Install
> Install Ubuntu 20.04 Desktop 64bit.

### 1.2 Settings
#### 1.2.1 Disable auto update
#### 1.2.2 Set auto login
#### 1.2.3 Disable auto sleep
#### 1.2.4 Set Network
#### 1.2.5 Set auto connect
#### 1.2.6 Set ToF camera connect to Ethnet
> 10.10.31.191/24

### 1.3 ROS Install

#### 1.3.1 Set sources.list
```bash
sudo sh -c '. /etc/lsb-release && echo "deb http://mirrors.ustc.edu.cn/ros/ubuntu/ `lsb_release -cs` main" > /etc/apt/sources.list.d/ros-latest.list'
```

#### 1.3.2 Set key
```bash
sudo apt install curl
```
```bash
curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -
```

#### 1.3.3 Install
```bash
sudo apt update
```
```bash
sudo apt install ros-noetic-desktop-full
```

#### 1.3.4 Set environment
```bash
echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc
```
```bash
source ~/.bashrc
```

#### 1.3.5 Dependancies
```bash
sudo apt install python3-rosdep python3-rosinstall python3-rosinstall-generator python3-wstool build-essential
```

#### 1.3.6 Init ROS
```bash
sudo rosdep init
```
```bash
rosdep update
```

### 1.4 Set port

#### 1.4.1 IMU Port
文件：/etc/udev/rules.d/90-sj-vision.ttyimu.rules
```bash
KERNEL=="ttyUSB*", ATTRS{idVendor}=="1a86", ATTRS{idProduct}=="7523", MODE:="0777", SYMLINK+="ttyIMU"

```

### 1.5 Source

#### 1.5.1 Source path
```bash
$ cd ~/VSEMI_TOF_320
```

### 1.6 Set auto-start
#### 1.6.1 Set CAN

File：/etc/init.d/initCAN

```bash
#!/bin/sh -e

# initialization socket can
# ip link set can0 down
ip link set can0 type can bitrate 250000
ip link set can0 up
ip link set can0 txqueuelen 1000
```

Create auto Start

```bash
$ sudo ln -s /etc/init.d/initCAN /etc/rc4.d/S60initCAN
```
#### 1.6.2 Start
```bash
$ ~/VSEMI_TOF_HARV_SF/run.sh &
```

