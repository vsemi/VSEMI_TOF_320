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

Read RGB camera ID
==================

   # plug in one RGB camera, and run command to find each camera ID:
   
   ls /dev/v4l/by-id
   
   # the command returns 2 records for each camera, for example:
   
   usb-Chicony_Electronics_Co._Ltd._Integrated_Camera_0001-video-index0
   usb-Chicony_Electronics_Co._Ltd._Integrated_Camera_0001-video-index1
   
   # the one with "0" is the ID we are going to use:
   
   usb-Chicony_Electronics_Co._Ltd._Integrated_Camera_0001-video-index0

Config RGB camera ID
====================

   # Using text editor, open file camera.launch, under src/vsemi_c320_camera/launch
   # edit line 31 as below, enter camera ID retrived from previous step:
   
   <arg name="video_camera" default="usb-Chicony_Electronics_Co._Ltd._Integrated_Camera_0001-video-index0"/>
   
Build and run application:

  ## make sure the file src/vsemi_c320_camera/cfg/vsemi_c320_camera.cfg has execute permission:

  >> sudo chmod a+rxw src/vsemi_c320_camera/cfg/vsemi_c320_camera.cfg

  >> catkin_make

To start the ROS application
============================

  #Add a static IP for PC:
  
  10.10.31.191

  #Edit run.sh and sensor.sh
  
  export ROS_IP=10.10.31.191
  export ROS_MASTER_URI=http://10.10.31.191:11311

  # run command:

  >> sudo -s
  
  # enter password

  # ./run.sh

Record ROS bag
==============

  # start another command line console:

  >> rosbag record /vsemi_c320_camera/camera/cloud /vsemi_c320_camera/camera/cloud_raw /vsemi_c320_camera/camera/depth_bgr /vsemi_c320_camera/camera/camera /vsemi_c320_camera/camera/amplitude /vsemi_c320_camera/camera/video
   
To stop ROS
===========

  # please press Ctr + C in terminal 


Run Application in Distributed Environment
==========================================

1. On Jetson Nano:

  #Add a static IP for Nano:
  
  10.10.31.191

  #Edit sensor.sh, change to the Nano's static IP address:
  
  export ROS_IP=10.10.31.191
  export ROS_MASTER_URI=http://10.10.31.191:11311

  # then run command to start sensor node only:

  >> sudo -s
  
  # enter password

  # ./sensor.sh

2. On remote PC:

  #Add a static IP for PC:
  
  10.10.31.191

  #Edit rviz.sh, change to the Nano's static IP address:
  
  export ROS_MASTER_URI=http://10.10.31.191:11311

  # then run command to start rviz node:

  >> ./rviz.sh

3. To record data:

  # in a terminal, run command below before recording:
  
  >> export ROS_MASTER_URI=http://10.10.31.191:11311
  
  >> rosbag record /vsemi_c320_camera/camera/cloud /vsemi_c320_camera/camera/camera /vsemi_c320_camera/camera/video


