#!/usr/bin/env python
import rospy
from utils.msg import Encoder, DiffVel, Odometry
import sys, select, termios, tty

def getKey():
    tty.setraw(sys.stdin.fileno())
    rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
    if rlist:
        key = sys.stdin.read(1)
    else:
        key = ''

    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key

if __name__=="__main__":
    settings = termios.tcgetattr(sys.stdin)
    rospy.init_node('test_actuator')
    pub = rospy.Publisher('diff_motor/vel', DiffVel, queue_size=5)
    pubgps = rospy.Publisher('odom_1', Odometry, queue_size=5)
    pubodom = rospy.Publisher('odom_2', Odometry, queue_size=5)

    rate = rospy.Rate(10)
# 10.77462
# 106.65962
    while not rospy.is_shutdown():
        msg = DiffVel()
        msg.left_vel = 40
        msg.right_vel = 40
        pub.publish(msg)
        # msg.latitude = 10.8817
        # msg.longitude = 108.4655
        # msg.position.x = 2
        # msg.position.y = 2
        # pubgps.publish(msg)
        msg = Odometry()
        msg.heading = 2.17
        pubodom.publish(msg)
        rate.sleep()
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)