#ifndef CAN_H
#define CAN_H

#include <ros/ros.h>
#include <serial/serial.h>
#include <sstream>
#include <vector>
#include <string>
#include <iomanip>
#include <sensor_msgs/NavSatFix.h>
#include <utils/DiffVel.h>
#include <utils/Odometry.h>
#include <utils/Odometry_gps.h>
#include <utils/geo.h>
#include <cmath>
#include <boost/bind.hpp>
#include "std_msgs/String.h"
#include <sensor_msgs/NavSatFix.h>
#include <sensor_msgs/Imu.h>
#include <utils/Encoder.h>
#include <boost/algorithm/string.hpp>
#include <utils/tf.h>
#include "std_msgs/String.h"

using namespace std;
using namespace utils;
using namespace serial;

struct vel_pid
{
    float Kp;
    float Ki;
    float Kd;
    vel_pid() : Kp(0.0), Ki(0.0), Kd(0.0) {}
};
struct imudata {
    float yaw;
    float ax;
    float ay;
    float az;
    float gx;
    float gy;
    float gz;
    imudata() : yaw(0.0), ax(0.0), ay(0.0), az(0.0), gx(0.0), gy(0.0), gz(0.0) {}
};

bool can_enabled;
std::string can_port;
int can_baudrate;
ros::Publisher pubOdom;
ros::Publisher pubdata;


ros::Publisher pubEncode;
ros::Publisher pubImu;
ros::Timer loopPub;
Odometry odom;
Encoder encode;
sensor_msgs::Imu imu;
#endif // GPSAHRS_RECEIVER_H


