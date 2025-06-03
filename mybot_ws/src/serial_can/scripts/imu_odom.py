#!/usr/bin/env python

import rospy
from sensor_msgs.msg import Imu
from geometry_msgs.msg import PoseStamped
import numpy as np
from utils.msg import Odometry
class IMUOdometry:
    def __init__(self):
        rospy.init_node('imu_odometry_node', anonymous=True)
        
        self.position = np.zeros(2) 
        self.velocity = np.zeros(2)
        self.prev_time = None
        
        self.bias = np.zeros(2)
        self.bias_computed = False
        
        rospy.Subscriber('/odom_2', Odometry, self.imu_callback)
        self.odom_pub = rospy.Publisher('/imu_odometry', PoseStamped, queue_size=10)
        
        rospy.spin()

    def compute_bias(self, msg):
        samples = 500
        sum_accel = np.zeros(2)
        
        for _ in range(samples):
            sum_accel[0] += msg.linear_acceleration.x
            sum_accel[1] += msg.linear_acceleration.y
            rospy.sleep(0.01)  # 10ms delay
        
        self.bias = sum_accel / samples
        self.bias_computed = True
        rospy.loginfo("Bias computed: x=%.5f, y=%.5f" % (self.bias[0], self.bias[1]))

    def imu_callback(self, msg):
        current_time = msg.header.stamp.to_sec()
        if self.prev_time is None:
            self.prev_time = current_time
            if not self.bias_computed:
                self.compute_bias(msg)
            return
        
        dt = current_time - self.prev_time
        self.prev_time = current_time
        
        linear_accel = np.array([msg.linear_acceleration.x - self.bias[0], 
                                 msg.linear_acceleration.y - self.bias[1]])
        linear_accel = np.round(linear_accel, 2)
        self.velocity = np.round(self.velocity, 3)
        alpha = 0.5  
        self.velocity = alpha * (self.velocity + linear_accel * dt) + (1 - alpha) * self.velocity    
        th = 0.1
        if (linear_accel[0] > th and linear_accel[0] < -th and 
            linear_accel[1] > th and linear_accel[1] < -th):
            self.position += 0 * dt
            rospy.loginfo("xa1=%.5f, ya=%.5f", linear_accel[0], linear_accel[1])
        else:
            rospy.loginfo("xa=%.5f, ya=%.5f", linear_accel[0], linear_accel[1])
            self.position += self.velocity * dt

        pose = PoseStamped()
        pose.header.stamp = msg.header.stamp
        pose.header.frame_id = "odom"
        
        pose.pose.position.x = self.position[0]
        pose.pose.position.y = self.position[1]
        pose.pose.position.z = 0.0 

        self.odom_pub.publish(pose)

if __name__ == '__main__':
    try:
        IMUOdometry()
    except rospy.ROSInterruptException:
        pass
