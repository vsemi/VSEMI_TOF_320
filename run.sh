#xport ROS_IP=10.10.31.191
#export ROS_MASTER_URI=http://10.10.31.191:11311

ip addr |grep can0|grep DOWN|wc -l > /dev/null

if [ $? -eq 1 ];then
        echo "Initialize CAN..."
        ip link set can0 type can bitrate 250000
        ip link set can0 up
        ip link set can0 txqueuelen 1000
fi

source /opt/ros/noetic/setup.bash
source /home/sj/Applications/VSEMI_TOF_HARV_SF/devel/setup.bash

roslaunch vsemi_tof_harv default.launch

