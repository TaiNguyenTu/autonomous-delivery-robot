#!/usr/bin/env python

import rospy
import math
from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Quaternion, Twist, Pose, Point

class IMUToOdom:
    def __init__(self):
        rospy.init_node('imu_to_odom')
        
        self.odom_pub = rospy.Publisher('odom', Odometry, queue_size=10)
        rospy.Subscriber('imu/data', Imu, self.imu_callback)
        
        self.odom = Odometry()
        self.odom.header.frame_id = 'odom'
        self.odom.child_frame_id = 'base_link'
        self.odom.pose.pose = Pose()
        self.odom.pose.pose.position = Point(0, 0, 0)
        self.odom.pose.pose.orientation = Quaternion(0, 0, 0, 1)
        
        self.current_time = rospy.Time.now()
        self.last_time = rospy.Time.now()
        
        self.angular_velocity_z = 0.0
        self.linear_acceleration_x = 0.0
        
        self.main_loop()

    def imu_callback(self, imu_msg):
        self.angular_velocity_z = imu_msg.angular_velocity.z
        self.linear_acceleration_x = imu_msg.linear_acceleration.x
        
        self.current_time = imu_msg.header.stamp
        dt = (self.current_time - self.last_time).to_sec()
        
        delta_theta = self.angular_velocity_z * dt
        
        quaternion = self.odom.pose.pose.orientation
        q = [quaternion.x, quaternion.y, quaternion.z, quaternion.w]
        dq = [0.0, 0.0, math.sin(delta_theta / 2), math.cos(delta_theta / 2)]
        q = self.quaternion_multiply(q, dq)
        
        self.odom.pose.pose.orientation.x = q[0]
        self.odom.pose.pose.orientation.y = q[1]
        self.odom.pose.pose.orientation.z = q[2]
        self.odom.pose.pose.orientation.w = q[3]
        
        self.odom.twist.twist.linear.x = self.linear_acceleration_x * dt
        
        self.odom.pose.pose.position.x += self.odom.twist.twist.linear.x * dt * math.cos(delta_theta)
        self.odom.pose.pose.position.y += self.odom.twist.twist.linear.x * dt * math.sin(delta_theta)
        
        self.last_time = self.current_time
    
    def quaternion_multiply(self, q1, q2):
        return [
            q1[3] * q2[0] + q1[0] * q2[3] + q1[1] * q2[2] - q1[2] * q2[1],
            q1[3] * q2[1] - q1[0] * q2[2] + q1[1] * q2[3] + q1[2] * q2[0],
            q1[3] * q2[2] + q1[0] * q2[1] - q1[1] * q2[0] + q1[2] * q2[3],
            q1[3] * q2[3] - q1[0] * q2[0] - q1[1] * q2[1] - q1[2] * q2[2]
        ]
    
    def main_loop(self):
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            self.odom.header.stamp = rospy.Time.now()
            
            self.odom_pub.publish(self.odom)
            
            rate.sleep()

if __name__ == '__main__':
    try:
        imu_to_odom = IMUToOdom()
    except rospy.ROSInterruptException:
        pass
