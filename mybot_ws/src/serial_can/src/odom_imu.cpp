#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
#include "std_msgs/Float32.h"
#include "utils/Odometry.h"


using namespace std;

  ros::Subscriber msg;
  ros::Subscriber msg1;
    float pose_x;
    float pose_y;
    float pose_z;

    float linear_x;
    float linear_y;
    float linear_z;

    float angular_x;
    float angular_y;
    float angular_z;

    float orientation_x;
    float orientation_y;
    float orientation_z;
    float orientation_w;

void CallbackOdometry(const utils::Odometry &od)
{

    //    pose_x = od.position.x;
     //   pose_y = od.position.y;
     //   pose_z = od.position.z;

        linear_x = od.linear_acceleration.x;
        linear_y = od.linear_acceleration.y;
        linear_z = od.linear_acceleration.z;

        angular_x = od.angular_velocity.x;
        angular_y = od.angular_velocity.y;
        angular_z = od.angular_velocity.z;

        orientation_x = od.orientation.x;
        orientation_y = od.orientation.y;
        orientation_z = od.orientation.z;
        orientation_w = 0.999999;

        // ROS_INFO("%f", pose);
}

//..................................................................................
int main(int argc, char** argv){
  
  //    system("python3 /home/ndthuan/agv_ws/src/AdvancedQRBar.py");

  ros::init(argc, argv, "odometry_publisher");

  ros::NodeHandle n;
  ros::Publisher odom_pub = n.advertise<nav_msgs::Odometry>("odom", 50);

  // ros::Subscriber Imu = n.subscribe("/odom_1", 1, CallbackTwist );
	ros::Subscriber model = n.subscribe("/odom_2", 1, CallbackOdometry);
//   ros::Subscriber Imu = n.subscribe("/ins/imu", 1, CallbackImu );

  ros::Rate r(20.0);
  while(n.ok()){

    ros::spinOnce();            
    //first, we'll publish the transform over tf
    geometry_msgs::TransformStamped odom_trans;
    odom_trans.header.stamp = ros::Time::now();
    odom_trans.header.frame_id = "odom";
    odom_trans.child_frame_id = "base_link";

    //next, we'll publish the odometry message over ROS
    nav_msgs::Odometry odom;
    odom.header.stamp = ros::Time::now();
    odom.header.frame_id = "odom";
    odom.child_frame_id = "base_link";

    //set the position & orientation
   // odom.pose.pose.position.x = pose_x;
   // odom.pose.pose.position.y = pose_y;
   // odom.pose.pose.position.z = pose_z;
 
    odom.pose.pose.orientation.x = orientation_x;
    odom.pose.pose.orientation.y = orientation_y;
    odom.pose.pose.orientation.z = orientation_z;
    odom.pose.pose.orientation.w = orientation_w;

    odom.twist.twist.angular.x = angular_x;
    odom.twist.twist.angular.y = angular_y;
    odom.twist.twist.angular.z = angular_z;

    odom.twist.twist.linear.x = linear_x;
    odom.twist.twist.linear.y = linear_y;
    odom.twist.twist.linear.z = linear_z;
   // odom.pose.pose.orientation = orientation;


    // set twist
    // odom.twist.twist = twist;

    //publish the message
    odom_pub.publish(odom);

    r.sleep();
  }
}
