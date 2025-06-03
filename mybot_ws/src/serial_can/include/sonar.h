#ifndef SONAR_H
#define SONAR_H

#include <ros/ros.h>
#include <serial/serial.h>
#include <sstream>
#include <string>
#include <iomanip>
#include <sensor_msgs/NavSatFix.h>
#include <utils/obtacles.h>
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
#include <boost/numeric/ublas/matrix.hpp>
#include <boost/numeric/ublas/vector.hpp>
#include <boost/numeric/ublas/io.hpp> 

using namespace boost::numeric::ublas;

matrix<float> A(2, 2); // State transition matrix
vector<float> x(2);    // State vector
matrix<float> P(2, 2); // Estimate error covariance
matrix<float> Q(2, 2); // Process noise covariance
vector<float> z(1);    // Measurement vector
matrix<float> H(1, 2); // Measurement matrix
matrix<float> K(2, 1); // Kalman gain
matrix<float> R(1, 1); // Measurement noise covariance

matrix<float> A1(2, 2); // State transition matrix
vector<float> x1(2);    // State vector
matrix<float> P1(2, 2); // Estimate error covariance
matrix<float> Q1(2, 2); // Process noise covariance
vector<float> z1(1);    // Measurement vector
matrix<float> H1(1, 2); // Measurement matrix
matrix<float> K1(2, 1); // Kalman gain
matrix<float> R1(1, 1); // Measurement noise covariance

using namespace std;
using namespace utils;
using namespace serial;

bool can_enabled;
std::string can_port;
int can_baudrate;
ros::Publisher pubSonar;
ros::Publisher pubdata;
ros::Timer loopPub;
obtacles sonar;
#endif

