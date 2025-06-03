#!/usr/bin/env python

import rospy
import paho.mqtt.client as mqtt
from utils.msg import Waypoint, WaypointList

def on_mqtt_message(client, userdata, msg):
    waypoint_values = msg.payload.split("|")
    waypoints_list = []
    for waypoint_str in waypoint_values[1:]:
        lat, lon = waypoint_str.split(",")
        wp = Waypoint()
        wp.x_lat = float(lat)
        wp.y_long = float(lon)
        wp.command = int(waypoint_values[0])
        waypoints_list.append(wp)
    rospy.loginfo("Parsed waypoints: %s", waypoints_list)    
    points_msg = WaypointList()
    points_msg.waypoints = waypoints_list
    points_msg.current_seq = len(waypoints_list)
    points_pub.publish(points_msg)

def points_callback(msg):
    rospy.loginfo("Received points data:")
    for wp in msg.waypoints:
        rospy.loginfo("x_lat: %f, y_long: %f", wp.x_lat, wp.y_long)

if __name__ == "__main__":
    rospy.init_node('test_node', anonymous=True)
    global points_pub
    points_pub = rospy.Publisher("/mission/item_list", WaypointList, queue_size=10)
    rospy.Subscriber("points", WaypointList, points_callback)
    client = mqtt.Client()

    client.on_message = on_mqtt_message

    client.connect("192.168.100.8", 4444, 60)

    client.subscribe("/mission/item_list")

    client.loop_start()

    rospy.spin()

    client.loop_stop()

    client.disconnect()

