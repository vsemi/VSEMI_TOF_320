Prerequisites:
=============

  Ubuntu
  ROS Noetic

Install ROS Noetic
==================

  >> sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'

  >> sudo apt install curl # if you haven't already installed curl
     curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -

  >> sudo apt update
  
  >> sudo apt install ros-noetic-desktop-full

  >> echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc
  
  >> source ~/.bashrc

  >> sudo apt install python3-rosdep python3-rosinstall python3-rosinstall-generator python3-wstool build-essential
    
  >> sudo rosdep init
  
  >> rosdep update
  
  ## set env for root so that application can be ran using sudo command

  >> sudo -s
  
  >> echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc
  
  >> exit

Find RGB camera ID
==================

   # plug in RGB/IMU, and run command to find each camera ID:
   
   ls /dev/v4l/by-id
   
   # the command returns 2 records for each camera, for example:
   
   usb-Chicony_Electronics_Co._Ltd._Integrated_Camera_0001-video-index0
   usb-Chicony_Electronics_Co._Ltd._Integrated_Camera_0001-video-index1
   
   # the one with "0" is the ID we are going to use:
   
   usb-Chicony_Electronics_Co._Ltd._Integrated_Camera_0001-video-index0

Config RGB camera ID
====================

   # Using text editor, open file all_nodes.launch, under src/vsemi_c320_camera/launch
   # edit line 31 as below, enter camera ID retrived from previous step:
   
   <arg name="video_camera" default="/dev/v4l/by-id/usb-046d_Logitech_Webcam_C930e_986A01BE-video-index0"/>

Find IMU
==================

   # plug in RGB/IMU, and run command to find each usb device:
   
   lsusb
   
   # from the list, find the ID of the IMU:
   
   Bus 001 Device 003: ID 10c4:ea60 Silicon Labs CP210x UART Bridge
   
   # read the detailed info of the IMU:
   
   lsusb -v -d 10c4:ea60
   
   # from the output, find:
   
   idVendor           0x10c4 Silicon Labs
   idProduct          0xea60 CP210x UART Bridge
   
Re-mapping IMU for convenience
====================

   # create a file:
   
   sudo vi /etc/udev/rules.d/99-vsemi.ttyimu.rules
   
   # which contains:
   
   KERNEL=="ttyUSB*", ATTRS{idVendor}=="10c4", ATTRS{idProduct}=="ea60", MODE:="0777", SYMLINK+="ttyIMU"
   
   # the idVendor and idProduct founs from previous step
   
   # reboot ubuntu, then, check the mapping:
   
   ls /dev/ttyIMU
   
   # you should able to find the IMU entry

Config IMU
====================

   # Using text editor, open file all_nodes.launch, under src/vsemi_c320_camera/launch
   # edit line 31 as below, enter camera ID retrived from previous step:
   
   <arg name="imu_port" default="/dev/ttyIMU"/>
   
Build and run application:

  ## make sure the file src/vsemi_c320_camera/cfg/vsemi_c320_camera.cfg has execute permission:

  >> sudo chmod a+rxw src/vsemi_c320_camera/cfg/vsemi_c320_camera.cfg

  >> catkin_make
  
Grant permission if you did not re-mapping IMU under root (skip the step if IMU been re-mapped):

  # in case your IMU at ttyUSB0:

  >> sudo chmod a+rw /dev/ttyUSB0

To start the ROS application
============================

  #Add a static IP for PC to access TOF camera. TOF camera has an static IP 10.10.31.180. So the address for PC can be set to:
  
  10.10.31.190

  # run command:

  # ./run.sh

To stop ROS
===========

  # please press Ctr + C in terminal 
  

