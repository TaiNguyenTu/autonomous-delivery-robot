#ifndef GUIDER_H
#define GUIDER_H

#include <ros/ros.h>

#include <utils/WaypointList.h>
#include <sensor_msgs/LaserScan.h>
#include <utils/Odometry.h>
#include <utils/Setpoint.h>
#include <utils/Error.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <utils/CommandLong.h>
#include <utils/ParamGet.h>
#include <utils/ParamSet.h>
#include <utils/SetMode.h>
#include <utils/mode_indoor.h>
#include <utils/Complete.h>
#include <utils/Position_avoid_obstacles.h>
#include <utils/pointID.h>
#include <utils/goal_indoor.h>
#include <utils/heading_plot.h>
#include <utils/CommandInt.h>
#include <utils/CommandLong.h>
#include <utils/ParamGet.h>
#include <utils/ParamSet.h>
#include <utils/Odometry_gps.h> 
#include <utils/DiffVel.h>
#include <utils/obtacles.h>
#include <stdio.h>
#include <iostream>
#include <string>
#include <fstream>

#include "utils/geo.h"
#include "straight_LOS.h"
#include <tf/tf.h>
#include <pid.h>

#include <dynamic_reconfigure/server.h>
#include <guider/GuiderConfig.h>

using namespace utils;


template <typename T>
T clamp(T val, T min_val, T max_val) {
    return std::max(min_val, std::min(val, max_val));
}



class Guider
{
public:
    Guider();
    ~Guider();

  double ned_lat;
  double ned_lon;
  double guiding_period;
  double controlling_period = 0.02;  // chinh 
  double update_period = 0.01;
  double diff_rpm = 45;
  double wheel_radius = 0.125;

    double currX;
    double currY;
    uint8_t mode_in = 2;
bool indoor;
    double currX_indoor;
    double currY_indoor;
    double currX_outdoor;
    double currY_outdoor;
    double currheading;
    double x_avoid;
    double y_avoid;


    uint8_t pointId;

    bool obstacles = false;

    float lat_end_waypoint;
    float lat_end_waypoint_set = 107746320.00;
    float waypoint_indoor_x[3];
    float waypoint_indoor_y[3];
    float waypoint_a_x[3];
    float waypoint_a_y[3];
  

  bool set_com_avoid = true;
  bool com_avoid ;

  float waypoint_x_0, waypoint_x_1, waypoint_x_2 ;
  float waypoint_y_0, waypoint_y_1, waypoint_y_2 ;
    double currHeading_indoor_x;
  double currHeading_indoor_y;
  double currHeading_indoor_z;
  double currHeading_indoor_w;
    float goal_indoor_set ;
  float complete_indoor ;
  float complete_indoor_set = 0.01 ;
    ros::Subscriber subPose;
    ros::Subscriber sub_mode;
    ros::Subscriber sub_goal_indoor_set;
    ros::Publisher pubSP;
    ros::Publisher pubError;
    ros::Publisher pubheadingplot;
    ros::Subscriber subsonar;
    ros::Publisher pubDiffVel;
    ros::Subscriber subSetpoint;
    ros::Subscriber sub_avoid;
    ros::Subscriber subOdom;
    ros::Subscriber sub_ID;
    ros::Subscriber subItemList;
    ros::Subscriber subGps;
    ros::Publisher pub_complete;

    ros::Timer loopGuidance;
    ros::Timer loopControl;
    ros::Timer loopUpdate;
    ros::Timer loopAddwaypointAvoid;
    ros::ServiceServer resStartMission;
    ros::ServiceServer resSetSetpointParams;
    ros::ServiceServer resGetSetpointParams;
    ros::ServiceServer resSetLOSParams;
    ros::ServiceServer resGetLOSParams;

    ros::Time lastSetpointTime;
    ros::Time lastControlUpdateTime;

    bool isMissionStarted = false;
    std::string customMode  = "2";
    bool sonar_left;
    bool sonar_right;
    StraightLOS straightLOSGuider;
    PID headingPID;

    double desiredSpeed ;
    double desiredHeading;
    double currHeading ;
    private:
    bool has_heading = false;
    double speed;
    double heading;
    
    double heading_scale;
    void logToFile(double gpsX, double gpsY, double leftVel, double rightVel);

    dynamic_reconfigure::Server<guider::GuiderConfig> *dr_srv;
void onReconfig(guider::GuiderConfig &config, uint32_t level);


    void onCallbackpointID(const pointID::ConstPtr& id);
    void onOdomCallBack(const Odometry::ConstPtr &msg);
    void onPosCallBack(const Odometry::ConstPtr &msg);
    void onSonarCallBack(const obtacles::ConstPtr& msg);
    void onSetpointCallBack(const Setpoint::ConstPtr& msg);
    void onItemListCallBack(const WaypointList::ConstPtr &msg);    
    void onGuidanceLoop(const ros::TimerEvent& event);
    void onControlLoop(const ros::TimerEvent& event);
    bool onStartMissionCallBack(CommandLong::Request &req, CommandLong::Response &res);
    bool onSetLOSParamsCallBack(ParamSet::Request &req, ParamSet::Response &res);
    bool onGetLOSParamsCallBack(ParamGet::Request &req, ParamGet::Response &res);
    bool onGetSetpointParamsCallBack(ParamGet::Request &req, ParamGet::Response &res);
    bool onSetSetpointParamsCallBack(ParamSetRequest& req, ParamSetResponse& res);
    void onCallbackPosAvoid(const Position_avoid_obstacles::ConstPtr& pos_avoid);
    void addwaypoint(const ros::TimerEvent& );
    void publishSetpoint(const double &speed, const double &heading);
    void publishError(const double &xe, const double &ye);
    void onModeCallback(const mode_indoor::ConstPtr& mode);
    void onCallbackPose(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& pos);
    void onCallbackGoalindoor(const goal_indoor::ConstPtr& goal_in);

    
    inline bool compare_string(const char *str1, const char *str2)
    {
        return !strncmp(str1, str2, strlen(str2));
    }
};
#endif // GUIDER_H
