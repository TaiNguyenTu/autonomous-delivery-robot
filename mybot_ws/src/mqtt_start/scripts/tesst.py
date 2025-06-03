#!/usr/bin/env python

import rospy
import random
import paho.mqtt.client as mqtt

broker_address = "broker.emqx.io"
port = 8083
topic = "random/data"

client = mqtt.Client(transport='websockets')
client.on_connect = lambda client, userdata, flags, rc: rospy.loginfo("Connected to MQTT broker with result code " + str(rc))
client.connect(broker_address, port)
client.loop_start()

rospy.init_node('mqtt_random_publisher', anonymous=True)
rate = rospy.Rate(1)

while not rospy.is_shutdown():
    random_data = random.randint(1, 10)
    client.publish(topic, random_data)
    rospy.loginfo("Published: {}".format(random_data))
    rate.sleep()

client.loop_stop()
client.disconnect()
