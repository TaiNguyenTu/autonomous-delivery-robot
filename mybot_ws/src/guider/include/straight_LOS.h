#ifndef  STRAIGHT_LOS_H
#define  STRAIGHT_LOS_H

#include <base_LOS.h>
#include <utils/pointID.h>
#include <ros/ros.h>
#include <utils/WaypointList.h>
using namespace utils;


class StraightLOS : public BaseLOS
{
public:
    StraightLOS();
    ~StraightLOS();
    ros::Publisher pubID;
    ros::Subscriber subWaypoint;
    std::vector<double> alpha_P;
    std::vector<double> s;
    unsigned pointId;
    unsigned long numPoints;
    std::vector<float> waypoint_outdoor_x_;
    std::vector<float> waypoint_outdoor_y_;

    // lấy tọa độ x,y
    
    // float waypoint_outdoor_x[2] = {31.6668960, 44.825916}; 
    // float waypoint_outdoor_y[2] = {-18.9031224, -2.212247};

    // float waypoint_outdoor_x[3] = {-55.3068158, -68.9406922  , -8.8952712}; 
    // float waypoint_outdoor_y[3] = {84.1853912, 103.7365434,  146.3228544};// từ A tới C

    // float waypoint_outdoor_x[2] = {-68.9406922  , -8.8952712}; 
    // float waypoint_outdoor_y[2] = {103.7365434,  146.3228544};// từ A tới C

    // float waypoint_outdoor_x[3] = {-8.8952712, -68.9406922, -55.3068158} ; 
    // float waypoint_outdoor_y[3] = {146.3228544, 103.7365434, 84.1853912};// từ C về A

    // float waypoint_outdoor_x[2] = {-8.8952712, -68.9406922 }; 
    // float waypoint_outdoor_y[2] = {146.3228544, 103.7365434};// từ C về A

    void setupLOS(); //phải gán chuỗi điểm thực vào waypoints rồi tính alpha_p và along_track_error
    void setLOS();   //tính alpha_p và along_track_error dựa vào waypoints có sẵn
    void resetLOS(); // đưa waypoints về 0
    bool runLOS(const double& odomX,const double& odomY); // chạy thuật toán để x,y bám theo waypoint
    private:
    double cte_scale;
};
#endif // STRAIGHT_LOS_H
