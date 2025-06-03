sudo route del default gw 192.168.1.1 dev eth0
sudo route add -host 20.41.104.186 gw 192.168.100.1 dev wlan0
sudo route add -host 20.41.104.186 gw 172.20.10.1 dev wlan0


# thời gian cuối cùng file binary được build lại.
ls -l devel/lib/guider/guider_node
# mở quạt
sudo sh -c 'echo 255 > /sys/devices/pwm-fan/target_pwm'



# Set IP ra ngoài
export ROS_IP=172.20.10.4     # (địa chỉ IP của Jetson)
export ROS_MASTER_URI=http://172.20.10.4:11311
roslaunch navi navi.launch


# trên máy tính
export ROS_MASTER_URI=http://172.20.10.4:11311
export ROS_IP=172.20.10.3
rosrun rqt_reconfigure rqt_reconfigure



rosclean check
rosclean purge