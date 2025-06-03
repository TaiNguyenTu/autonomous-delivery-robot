#!/usr/bin/env python
import rospy
from utils.msg import DiffVel
from utils.msg import Odometry, Setpoint, Error, Encoder
import paho.mqtt.client as mqtt
import math
import json

pub_kp_vel = rospy.Publisher('diff_motor/kp', DiffVel, queue_size=10)
pub_ki_vel = rospy.Publisher('diff_motor/ki', DiffVel, queue_size=10)
pub_kd_vel = rospy.Publisher('diff_motor/kd', DiffVel, queue_size=10)


def diff_vel_callback(data, client):  
    vel_l = data.left_vel
    vel_r = data.right_vel
    client.publish("AGV/setvel", "{},{}".format(vel_l, vel_r))
    # rospy.loginfo("setvel: %s","{},{}".format(vel_l, vel_r))

def real_vel_callback(data, client): 
    vel_l = data.left_twist
    vel_r = data.right_twist
    client.publish("AGV/realvel", "{},{}".format(vel_l, vel_r))
    # rospy.loginfo("realvel: %s","{},{}".format(vel_l, vel_r))

def location_callback(data, client):
    latitude = data.latitude
    longitude = data.longitude
    heading = data.heading*180/math.pi
    client.publish("AGV/gps", "{}, {}, {}".format(latitude,longitude,heading))
    # rospy.loginfo("gps: %s", "{}, {}, {}".format(latitude,longitude,heading))


def desire_point_callback(data, client):
    speed = data.linear_velocity.x
    heading = data.orientation.z*180/(math.pi)
    client.publish("AGV/desire_point", "{}, {}".format(speed, heading))
    # rospy.loginfo("desire_point: %s","{}, {}".format(speed, heading))

def error_callback(data, client):
    along = data.along_track
    cross = data.cross_track
    client.publish("AGV/err", "{}, {}".format(along, cross))
    # rospy.loginfo("error: %s","{}, {}".format(along, cross))

def heading_callback(data, client):
    heading = data.heading*180/math.pi
    client.publish("AGV/heading", "{}".format(heading))
    # rospy.loginfo("heading: %s",  "{}".format(heading))

    
def on_pid_vel_setting(client, userdata, msg):
    pid_values = msg.payload.decode("utf-8")
    pid_list = json.loads(pid_values)
    diff_vel_msg = DiffVel()
    diff_vel_msg.header.stamp = rospy.Time.now()
    diff_vel_msg.left_vel = pid_list[0]["kp"]
    diff_vel_msg.right_vel = pid_list[1]["kp"]
    pub_kp_vel.publish(diff_vel_msg)
    diff_vel_msg.left_vel = pid_list[0]["kd"]
    diff_vel_msg.right_vel = pid_list[1]["kd"]
    pub_ki_vel.publish(diff_vel_msg)
    diff_vel_msg.left_vel = pid_list[0]["ki"]
    diff_vel_msg.right_vel = pid_list[1]["ki"]
    pub_kd_vel.publish(diff_vel_msg)
    # rospy.loginfo("PID done")
    
if __name__ == "__main__":
    rospy.init_node('start_mq_node', anonymous=True)

    client = mqtt.Client()
    client.connect("20.41.104.186", 1883, 60)

    client.subscribe("pid_setting")
    client.on_message = on_pid_vel_setting
    client.loop_start() 

    rospy.Subscriber('diff_motor/vel', DiffVel, diff_vel_callback, client) 

    rospy.Subscriber('diff_motor/real_vel', Encoder, real_vel_callback, client)

    rospy.Subscriber('odom_1', Odometry, location_callback, client)

    rospy.Subscriber('odom_2', Odometry, heading_callback, client)
    rospy.Subscriber('setpoint', Setpoint, desire_point_callback, client)

    rospy.Subscriber('error', Error, error_callback, client)

    rospy.spin()

    client.disconnect()













# #!/usr/bin/env python
# import rospy
# from utils.msg import DiffVel
# from utils.msg import Odometry, Setpoint, Error, Encoder
# import paho.mqtt.client as mqtt
# import math
# import json

# pub_kp_vel = rospy.Publisher('diff_motor/kp', DiffVel, queue_size=10)
# pub_ki_vel = rospy.Publisher('diff_motor/ki', DiffVel, queue_size=10)
# pub_kd_vel = rospy.Publisher('diff_motor/kd', DiffVel, queue_size=10)


# latest_heading = None
# latest_gps = None

# latest_setvel=None
# latest_realvel=None
# latest_desire_point=None
# latest_error=None



# def diff_vel_callback(data, client): 
#     global latest_setvel
#     vel_l = data.left_vel
#     vel_r = data.right_vel
#     latest_setvel = (vel_l, vel_r)

# def publish_diff_vel(event, client):
#     global latest_setvel
#     if latest_setvel is not None:
#         vel_l, vel_r = latest_setvel
#         client.publish("AGV/setvel", "{},{}".format(vel_l, vel_r))
#         # rospy.loginfo("setvel: %s","{},{}".format(vel_l, vel_r))






# def real_vel_callback(data, client): 
#     global latest_realvel
#     vel_l = data.left_twist
#     vel_r = data.right_twist
#     latest_realvel = (vel_l, vel_r)


# def publish_real_vel(event, client):
#     global latest_realvel
#     if latest_realvel is not None:
#         vel_l, vel_r = latest_realvel
#         client.publish("AGV/realvel", "{},{}".format(vel_l, vel_r))
#         # rospy.loginfo("realvel: %s","{},{}".format(vel_l, vel_r))
    



# def location_callback(data, client):
#     global latest_gps
#     latitude = data.latitude
#     longitude = data.longitude
#     heading = data.heading*180/math.pi
#     latest_gps = (latitude, longitude, heading)

# def publish_gps(event, client):
#     global latest_gps
#     if latest_gps is not None:
#         latitude, longitude, heading = latest_gps
#         client.publish("AGV/gps", "{}, {}, {}".format(latitude, longitude, heading))
#         # rospy.loginfo("Published GPS: %s", "{}, {}, {}".format(latitude, longitude, heading))






# def desire_point_callback(data, client):
#     global latest_desire_point
#     speed = data.linear_velocity.x
#     heading = data.orientation.z*180/(math.pi)
#     latest_desire_point = (speed, heading)


# def publish_desire_point(event, client):
#     global latest_desire_point
#     if latest_desire_point is not None:
#         speed, heading = latest_desire_point
#         client.publish("AGV/desire_point", "{}, {}".format(speed, heading))
#         # rospy.loginfo("desire_point: %s","{}, {}".format(speed, heading))





# def error_callback(data, client):
#     global latest_error
#     along = data.along_track
#     cross = data.cross_track
#     latest_error = (along, cross)

   
# def publish_error(event, client):
#     global latest_error
#     if latest_error is not None:
#         along, cross = latest_error
#         client.publish("AGV/err", "{}, {}".format(along, cross))
#         # rospy.loginfo("error: %s","{}, {}".format(along, cross))







# def heading_callback(data, client):
#     global latest_heading
#     latest_heading = data.heading * 180 / math.pi


# def publish_heading(event, client):
#     global latest_heading
#     if latest_heading is not None:
#         client.publish("AGV/heading", "{}".format(latest_heading))
#         # rospy.loginfo("Published heading: %s", "{}".format(latest_heading))

    



# def on_pid_vel_setting(client, userdata, msg):
#     pid_values = msg.payload.decode("utf-8")
#     pid_list = json.loads(pid_values)
#     diff_vel_msg = DiffVel()
#     diff_vel_msg.header.stamp = rospy.Time.now()
#     diff_vel_msg.left_vel = pid_list[0]["kp"]
#     diff_vel_msg.right_vel = pid_list[1]["kp"]
#     pub_kp_vel.publish(diff_vel_msg)
#     diff_vel_msg.left_vel = pid_list[0]["kd"]
#     diff_vel_msg.right_vel = pid_list[1]["kd"]
#     pub_ki_vel.publish(diff_vel_msg)
#     diff_vel_msg.left_vel = pid_list[0]["ki"]
#     diff_vel_msg.right_vel = pid_list[1]["ki"]
#     pub_kd_vel.publish(diff_vel_msg)
#     rospy.loginfo("PID done")
    
# if __name__ == "__main__":
#     rospy.init_node('start_mq_node', anonymous=True)

#     client = mqtt.Client()
#     client.connect("20.41.104.186", 1883, 60)

#     client.subscribe("pid_setting")
#     client.on_message = on_pid_vel_setting
#     client.loop_start() 

#     rospy.Subscriber('diff_motor/vel', DiffVel, diff_vel_callback, client) 

#     rospy.Subscriber('diff_motor/real_vel', Encoder, real_vel_callback, client)

#     rospy.Subscriber('odom_1', Odometry, location_callback, client)

#     rospy.Subscriber('odom_2', Odometry, heading_callback, client)

#     rospy.Subscriber('setpoint', Setpoint, desire_point_callback, client)

#     rospy.Subscriber('error', Error, error_callback, client)

#     rospy.Timer(rospy.Duration(0.252), lambda event: publish_heading(event, client))
#     rospy.Timer(rospy.Duration(0.252), lambda event: publish_gps(event, client))
#     rospy.Timer(rospy.Duration(0.252), lambda event: publish_diff_vel(event, client))
#     rospy.Timer(rospy.Duration(0.252), lambda event: publish_real_vel(event, client))
#     rospy.Timer(rospy.Duration(0.252), lambda event: publish_desire_point(event, client))
#     rospy.Timer(rospy.Duration(0.252), lambda event: publish_error(event, client))

#     rospy.spin()

#     client.disconnect()
