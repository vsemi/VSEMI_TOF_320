#!/bin/bash
sudo ip link set can0 type can bitrate 250000
sudo ip link set can0 up
sudo ip link set can0 txqueuelen 1000

