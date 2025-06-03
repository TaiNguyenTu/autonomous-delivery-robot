#include <ros/ros.h>
#include <serial/serial.h>
#include <sstream>
#include <string>
#include <iomanip>
#include <sensor_msgs/NavSatFix.h>
#include <sensor_msgs/Imu.h>
#include <utils/DiffVel.h>
#include <utils/Odometry.h>
#include <utils/obtacles.h>
#include <cmath>
#include <utils/Encoder.h>
#include <boost/bind.hpp>
#include <boost/algorithm/string.hpp>
#include <utils/tf.h>
#include "std_msgs/String.h"
#include "sonar.h"

using namespace std;
boost::numeric::ublas::matrix<float> inverse(boost::numeric::ublas::matrix<float> mat) {
    boost::numeric::ublas::matrix<float> inv(1, 1);
    inv(0, 0) = 1 / mat(0, 0);
    return inv;
}

float applyKalmanFilter_l(float measurement) {
    x = prod(A, x);
    P = prod(A, matrix<float>(prod(P, trans(A)))) + Q;
    z(0) = measurement;
    boost::numeric::ublas::vector<float> y = z - prod(H, x); // Measurement residual
    boost::numeric::ublas::matrix<float> S = prod(H, matrix<float>(prod(P, trans(H)))) + R; // Residual covariance
    K = prod(P, matrix<float>(prod(trans(H), matrix<float>(inverse(S))))); // Kalman gain
    x = x + prod(K, y); // Updated state estimate
    P = prod((identity_matrix<float>(2) - prod(K, H)), P); // Updated estimate covariance
    
    return x(0); // Return the filtered left_dis
}

float applyKalmanFilter_r(float measurement1) {
    x1 = prod(A1, x1);
    P1 = prod(A1, matrix<float>(prod(P1, trans(A1)))) + Q1;
    z1(0) = measurement1;
    boost::numeric::ublas::vector<float> y1 = z1 - prod(H1, x1); // Measurement residual
    boost::numeric::ublas::matrix<float> S1 = prod(H1, matrix<float>(prod(P1, trans(H1)))) + R1; // Residual covariance
    K1 = prod(P1, matrix<float>(prod(trans(H1), matrix<float>(inverse(S1))))); // Kalman gain
    x1 = x1 + prod(K1, y1); // Updated state estimate
    P1 = prod((identity_matrix<float>(2) - prod(K1, H1)), P1); // Updated estimate covariance
    return x1(0); // Return the filtered left_dis
}

void serialCallback(const std::string& data, const std::string& data1, ros::Publisher pubdata, ros::Publisher pubSonar) {
    std::vector<std::string> dataList, dataList1;
    boost::split(dataList,data,boost::is_any_of("R"));
    boost::split(dataList1,data1,boost::is_any_of("R"));
    float left_dis = 0;
    float right_dis = 0;
    if (data.size() == 6) {
    if (dataList.size() == 2 && dataList1.size() == 2) {
    left_dis = std::stof(dataList[1]);
    right_dis = std::stof(dataList1[1]);
    }


    // ROS_INFO_STREAM("Left: " << dataList[1]);  
    // ROS_INFO_STREAM("Right:" << dataList1[1]); 
    left_dis = applyKalmanFilter_l(left_dis);
    right_dis = applyKalmanFilter_r(right_dis); 
    ROS_INFO("Left: %.2f", left_dis);  
    ROS_INFO("Right: %.2f", right_dis);  

    // ROS_INFO_STREAM("Received "  << dataList.size()  << " bytes " << "so phan tu: " << dataList[1]);

    sonar.left_dis_sonar = left_dis;
    sonar.right_dis_sonar = right_dis;
    if (left_dis < 50) {
        sonar.sonar_left = true;
    }
    else {
        sonar.sonar_left = false;
    }
        if (right_dis < 50) {
        sonar.sonar_right = true;
    }
    else {
        sonar.sonar_right = false;
    }
    pubSonar.publish(sonar);
    }  
}

void updatePub(const ros::TimerEvent&) {

}

int main(int argc, char** argv) {
    ros::init(argc, argv, "sonar_node");
    ros::NodeHandle nh;
    ros::NodeHandle private_nh("~");
    pubdata = nh.advertise<std_msgs::String>("so", 10);
    pubSonar =  nh.advertise<utils::obtacles>("sonar", 10);
    loopPub = nh.createTimer(ros::Duration(0.01), updatePub);
    serial::Serial serialPort;
    serial::Serial serialPort1;  

    A(0, 0) = 1; A(0, 1) = 0;
    A(1, 0) = 0; A(1, 1) = 1;
    x(0) = 0; 
    x(1) = 0;
    P(0, 0) = 1; P(0, 1) = 0;
    P(1, 0) = 0; P(1, 1) = 1;
    Q(0, 0) = 0.001; Q(0, 1) = 0;
    Q(1, 0) = 0; Q(1, 1) = 0.001;
    H(0, 0) = 1; H(0, 1) = 0;
    R(0, 0) = 0.03;

    A1(0, 0) = 1; A1(0, 1) = 0;
    A1(1, 0) = 0; A1(1, 1) = 1;
    x1(0) = 0; 
    x1(1) = 0;
    P1(0, 0) = 1; P1(0, 1) = 0;
    P1(1, 0) = 0; P1(1, 1) = 1;
    Q1(0, 0) = 0.001; Q1(0, 1) = 0;
    Q1(1, 0) = 0; Q1(1, 1) = 0.001;
    H1(0, 0) = 1; H1(0, 1) = 0;
    R1(0, 0) = 0.03;

    try {
        serialPort.setPort("/dev/ttyUSB0");
        serialPort.setBaudrate(9600);
        serial::Timeout timeout = serial::Timeout::simpleTimeout(1000);
        serialPort.setTimeout(timeout);
        serialPort.open();

        serialPort1.setPort("/dev/ttyUSB1");
        serialPort1.setBaudrate(9600);
        serial::Timeout timeout1 = serial::Timeout::simpleTimeout(1000);
        serialPort1.setTimeout(timeout);
        serialPort1.open();
    } catch (serial::IOException& e) {
        ROS_ERROR_STREAM("Unable to open serial port: " << e.what());
        return -1;
    }
    if (!serialPort.isOpen() || !serialPort1.isOpen()) {
        ROS_ERROR_STREAM("Failed to open serial port.");
        return -1;
    }

    try {
        while (ros::ok()) {
            if (serialPort.available() && serialPort1.available()) {
                std::string data = serialPort.readline(65536, "\r");
                std::string data1 = serialPort1.readline(65536, "\r");
                serialCallback(data, data1, pubdata, pubSonar);
            }
            ros::spinOnce();
        }
    } catch (serial::IOException& e) {
        ROS_ERROR_STREAM("Error reading from serial port: " << e.what());
    }
    serialPort.close();
    return 0;
}
