# 自动跟车系统

[TOC]

## 1 系统部署

### 1.1 系统安装
> 正常安装Ubuntu 20.04 Desktop 64bit 版本，过程不详细描述。

### 1.2 系统设置
#### 1.2.1 关闭系统自动更新
#### 1.2.2 设置自动登录
#### 1.2.3 设置永不休眠
#### 1.2.4 设置移动网络自动激活
#### 1.2.5 设置VPN自动连接
#### 1.2.6 设置660转接板网卡IP
> 10.10.31.191/24

### 1.3 ROS安装

#### 1.3.1 设置sources.list
```bash
sudo sh -c '. /etc/lsb-release && echo "deb http://mirrors.ustc.edu.cn/ros/ubuntu/ `lsb_release -cs` main" > /etc/apt/sources.list.d/ros-latest.list'
```

#### 1.3.2 设置密钥
```bash
sudo apt install curl
```
```bash
curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -
```

#### 1.3.3 安装
```bash
sudo apt update
```
```bash
sudo apt install ros-noetic-desktop-full
```

#### 1.3.4 环境变量
```bash
echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc
```
```bash
source ~/.bashrc
```

#### 1.3.5 依赖库
```bash
sudo apt install python3-rosdep python3-rosinstall python3-rosinstall-generator python3-wstool build-essential
```

#### 1.3.6 初始化ROS
```bash
sudo rosdep init
```
```bash
rosdep update
```

### 1.4 驱动部署

#### 1.4.1 IMU驱动
文件：/etc/udev/rules.d/90-sj-vision.ttyimu.rules
```bash
KERNEL=="ttyUSB*", ATTRS{idVendor}=="1a86", ATTRS{idProduct}=="7523", MODE:="0777", SYMLINK+="ttyIMU"

```

### 1.5 源码部署

#### 1.5.1 目录路径
```bash
$ cd /home/sj-vision/Applications/VSEMI_TOF_HARV_SF
```

### 1.6 配置自动启动
#### 1.6.1 自动初始化CAN卡

文件：/etc/init.d/initCAN

```bash
#!/bin/sh -e

# initialization socket can
# ip link set can0 down
ip link set can0 type can bitrate 250000
ip link set can0 up
ip link set can0 txqueuelen 1000
```

加入开机启动

```bash
$ sudo ln -s /etc/init.d/initCAN /etc/rc4.d/S60initCAN
```
#### 1.6.2 桌面环境配置开机自启
```bash
$ /bin/bash /home/sj-vision/Applications/VSEMI_TOF_HARV_SF/run.sh &
```

