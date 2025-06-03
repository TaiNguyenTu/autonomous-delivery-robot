#include "straight_LOS.h"


// dựa vào base_LOS
// Tính alpha_P và along-track err qua hàm void StraightLOS::setLOS() và void StraightLOS::setupLOS()
// thực hiện thuật toán LOS: đầu ra là desiredHeading và crossTrackError qua hàm bool StraightLOS::runLOS

StraightLOS::StraightLOS() {
  ros::NodeHandle nh;
  pubID = nh.advertise<pointID>("/pointID", 10);  //publish
  // subWaypoint = nh.subscribe<WaypointList>("/mission/item_list", 10);
}
StraightLOS::~StraightLOS(){
  if(ros::isStarted()){
    ros::shutdown();
    ros::waitForShutdown();
  }
}


// Tính alpha_P và along-track err
void StraightLOS::setLOS()
{
  numPoints = waypoints.size();
  for (int i = 0 ; i < numPoints ; i++){
  }
  if (numPoints > 1)
  {
    alpha_P.resize(numPoints - 1);
    s.resize(numPoints - 1);
    for (unsigned i = 0; i < numPoints - 1; i++)
    { 
      // alpha_P và along-track err
      double diff_x = waypoints[i + 1].x - waypoints[i].x;   
      double diff_y = waypoints[i + 1].y - waypoints[i].y;
      alpha_P[i] = atan2(diff_y, diff_x);
      s[i] = diff_x * cos(alpha_P[i]) + diff_y * sin(alpha_P[i]);
    }
  }
}

void StraightLOS::setupLOS()
{
  ros::NodeHandle nh_private("~");

   // Đọc tham số từ launch file
  nh_private.getParam("waypoint_outdoor_x", waypoint_outdoor_x_);
  nh_private.getParam("waypoint_outdoor_y", waypoint_outdoor_y_);
  nh_private.getParam("LOS_cte_scale", cte_scale);  
  // ROS_INFO_STREAM("waypoint_outdoor_x_ size: " << waypoint_outdoor_x_.size());
  // ROS_INFO_STREAM("waypoint_outdoor_y_ size: " << waypoint_outdoor_y_.size());
  // for (size_t i = 0; i < waypoint_outdoor_x_.size(); i++) {
  //   ROS_INFO_STREAM("x[" << i << "] = " << waypoint_outdoor_x_[i] << ", y[" << i << "] = " << waypoint_outdoor_y_[i]);
  // }
  // ROS_INFO_STREAM("LOS_cte_scale: " << cte_scale);
   // Kiểm tra
  if (waypoint_outdoor_x_.size() != waypoint_outdoor_y_.size() || waypoint_outdoor_x_.empty()) {
      ROS_ERROR("Invalid waypoint list!");
      return;
  }
   // Đưa vào waypoints
  for (size_t i = 0; i < waypoint_outdoor_x_.size(); i++) {
      waypoints.push_back(BaseLOS::Point(waypoint_outdoor_x_[i], waypoint_outdoor_y_[i]));
  }

  numPoints = waypoints.size();
  if (numPoints > 1)
  {
    alpha_P.resize(numPoints - 1);
    s.resize(numPoints - 1);
    for (unsigned i = 0; i < numPoints - 1; i++)
    {
      double diff_x = waypoints[i + 1].x - waypoints[i].x;
      double diff_y = waypoints[i + 1].y - waypoints[i].y;
      alpha_P[i] = atan2(diff_y, diff_x);
      s[i] = diff_x * cos(alpha_P[i]) + diff_y * sin(alpha_P[i]);
    }
  }
}

void StraightLOS::resetLOS()
{
  waypoints.clear();
  pointId = 0;
}


// hàm runLOS:
// odomX là x và waypoints[pointId].x là xk
// odomY là y và waypoints[pointId].y là yk
bool StraightLOS::runLOS(const double &odomX, const double &odomY)
{
    double c_alpha = cos(alpha_P[pointId]);
    double s_alpha = sin(alpha_P[pointId]);
    double diff_x = odomX - waypoints[pointId].x;
    double diff_y = odomY - waypoints[pointId].y;

  
  // Calculate the along-track and cross-track errors
    alongTrackError = diff_x * c_alpha + diff_y * s_alpha;
    crossTrackError = -diff_x * s_alpha + diff_y * c_alpha;

  // Find desired heading
  // ROS_INFO_STREAM("hihi: " << cte_scale);
    double delta = (maxDelta - minDelta) * exp(-cte_scale * crossTrackError * crossTrackError) + minDelta;

    // double delta = (maxDelta - minDelta) * exp(-cte_scale * fabs(crossTrackError)) + minDelta;
  
    desiredHeading = alpha_P[pointId] + atan2(-crossTrackError, delta);
    // desiredHeading = 101.12*M_PI/180;
    // desiredHeading = -81.44*M_PI/180;
    // desiredHeading = 0.2 * sin(2 * M_PI * ros::Time::now().toSec() / 10.0);

      // ROS_INFO("alpha_P %f",alpha_P[pointId]);
    desiredHeading = atan2(sin(desiredHeading), cos(desiredHeading)); // Lưu ý coi chính xác chưa
      // ROS_INFO("desired_heading %f",desiredHeading);

  // Bán kính chuyển tiếp 
    if (pointId < numPoints - 2)
    {
      if (fabs(s[pointId] - alongTrackError) < radius)
        pointId++;
    }
    else
    {
      if (fabs(s[pointId] - alongTrackError) < 1.0)
        pointId++;
    }

    pointID id;
    id.pointID = pointId;
    pubID.publish(id);

  // Terminate LOS if the final point has been reached
    return pointId < numPoints - 1;
}


