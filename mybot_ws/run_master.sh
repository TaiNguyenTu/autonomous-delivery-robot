#!/bin/bash

# Lấy IP của Jetson
JETSON_IP=$(hostname -I | awk '{print $1}')

# Xuất biến môi trường ROS
export ROS_IP=$JETSON_IP
export ROS_MASTER_URI=http://$JETSON_IP:11311

# # In thông tin
# echo "[Jetson] ROS_IP=$ROS_IP"
# echo "[Jetson] ROS_MASTER_URI=$ROS_MASTER_URI"

# Chạy file launch chính
roslaunch navi navi.launch