#!/bin/bash

# 脚本说明
# 国讯芯微can0/can1默认关闭，运行此脚本开启can0/can1功能
# 直接将此脚本内容，拷贝到/etc/init.d/nplc.sh后面，国讯芯微开机自动开启CAN端口

# 打开can0和can1 @Jordi 20260109
sudo busybox devmem 0x0c303018 w 0xc458
sudo busybox devmem 0x0c303010 w 0xc400
sudo busybox devmem 0x0c303008 w 0xc458
sudo busybox devmem 0x0c303000 w 0xc400

sudo modprobe can
sudo modprobe can_raw
sudo modprobe mttcan

sudo ip link set down can0
sudo ip link set can0 type can bitrate 1000000
sudo ip link set up can0

sudo ip link set down can1
sudo ip link set can1 type can bitrate 1000000
sudo ip link set up can1
