#!/usr/bin/env python
import rospy
from utils.msg import DiffVel
from utils.msg import Odometry
import paho.mqtt.client as mqtt

from utils.srv import CommandLong

def diff_vel_callback(data, client): 
    vel_l = data.left_vel
    vel_r = data.right_vel
    client.publish("vel", "{}, {}".format(vel_l, vel_r))
    rospy.loginfo("vel: %s","{}, {}".format(vel_l, vel_r))

def location_callback(data, client):
    latitude = data.latitude
    longitude = data.longitude
    client.publish("gps", "{}, {}".format(latitude,longitude))
    rospy.loginfo("gps: %s", "{}, {}".format(latitude,longitude))

if __name__ == "__main__":
    rospy.init_node('start_mq_node')

    client = mqtt.Client()
    
    client.connect("10.42.0.206", 4040, 60)

    rospy.Subscriber('diff_motor/vel', DiffVel, diff_vel_callback, client)

    # rospy.Subscriber('odom_1', Odometry, location_callback, client)

    rospy.spin()
    
    a = "85fdda129a"

    client.disconnect()

    