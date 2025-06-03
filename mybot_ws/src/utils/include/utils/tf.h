#ifndef TF_H
#define TF_H

#include <ros/ros.h>

#include <nav_msgs/Odometry.h>

#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Scalar.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2/LinearMath/Vector3.h>

#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>

#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

const tf2::Transform body_to_gps_point =
    tf2::Transform(tf2::Quaternion(0, 0, 0, 1), tf2::Vector3(0.4495, -0.03, -0.4105));

const tf2::Transform body_to_imu_point =
    tf2::Transform(tf2::Quaternion(0, 0, 0, 1), tf2::Vector3(0.4495, 0.03, -0.4105));

const tf2::Transform body_to_lidar_point =
    tf2::Transform(tf2::Quaternion(1, 0, 0, 1.03412e-13), tf2::Vector3(0.3945, -9.46216e-14, -0.4575));

const tf2::Transform ned_to_map = tf2::Transform(tf2::Quaternion(1, 0, 0, 1.03412e-13), tf2::Vector3(0, 0, 0));

inline void get_RPY_from_quaternion(const geometry_msgs::Quaternion& quatMsg, double& roll, double& pitch, double& yaw)
{
  tf2::Quaternion quat;
  tf2::fromMsg(quatMsg, quat);
  tf2::Matrix3x3(quat).getRPY(roll, pitch, yaw);
}

inline void get_RPY_from_quaternion(const tf2::Quaternion& quat, double& roll, double& pitch, double& yaw)
{
  tf2::Matrix3x3(quat).getRPY(roll, pitch, yaw);
}

inline double get_heading_from_quaternion(const geometry_msgs::Quaternion& quatMsg)
{
  double roll, pitch, heading;
  get_RPY_from_quaternion(quatMsg, roll, pitch, heading);
  return heading;
}

inline double get_heading_from_quaternion(const tf2::Quaternion& quat)
{
  double roll, pitch, heading;
  get_RPY_from_quaternion(quat, roll, pitch, heading);
  return heading;
}

inline geometry_msgs::Quaternion get_quaternion_from_RPY(const double& roll, const double& pitch, const double& yaw)
{
  tf2::Quaternion quat;
  quat.setRPY(roll, pitch, yaw);
  return tf2::toMsg(quat);
}

inline geometry_msgs::Quaternion get_quaternion_from_heading(const double& heading)
{
  tf2::Quaternion quat;
  quat.setRPY(0, 0, heading);
  return tf2::toMsg(quat);
}

inline tf2::Quaternion convert_to_tf_quaternion(const geometry_msgs::Quaternion& quat)
{
  return tf2::Quaternion(quat.x, quat.y, quat.z, quat.w);
}

inline tf2::Transform get_transform_from_odometry(const nav_msgs::Odometry& msg)
{
  tf2::Vector3 pos;
  tf2::Quaternion quat;
  tf2::fromMsg(msg.pose.pose.position, pos);
  tf2::fromMsg(msg.pose.pose.orientation, quat);
  return tf2::Transform(quat, pos);
}

inline tf2::Transform listen_to_tf(const std::string& src_frame, const std::string& dest_frame, ros::NodeHandle& nh)
{
  tf2_ros::Buffer tfBuffer;
  tf2_ros::TransformListener tfListener(tfBuffer);
  geometry_msgs::TransformStamped transformStamped;

  ros::Rate rate(10.0);
  while (nh.ok())
  {
    try
    {
      transformStamped = tfBuffer.lookupTransform(src_frame, dest_frame, ros::Time(0));
      break;
    }
    catch (tf2::TransformException& ex)
    {
      ROS_WARN("%s", ex.what());
      ros::Duration(1.0).sleep();
      continue;
    }
  }

  tf2::Transform result;
  tf2::fromMsg(transformStamped.transform, result);
  return result;
}

#endif // TF_H
