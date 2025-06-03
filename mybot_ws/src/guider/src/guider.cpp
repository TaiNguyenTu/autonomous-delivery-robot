#include <guider.h>
#include <algorithm>
#include <dynamic_reconfigure/server.h>
#include <guider/GuiderConfig.h>
#include <ctime>
#include <iomanip>
#include <sstream>
#include <sys/stat.h>
#include <chrono>
#include <dirent.h>  

std::ofstream guiderLog;

std::string generateLogFilename() {
    time_t now = time(0);
    struct tm *ltm = localtime(&now);

    int year = 1900 + ltm->tm_year;
    int month = 1 + ltm->tm_mon;
    int day = ltm->tm_mday;

    std::ostringstream prefix;
    prefix << std::setw(2) << std::setfill('0') << day << "_"
           << std::setw(2) << std::setfill('0') << month << "_"
           << year;

    std::string logDir = "/home/devbot/mybot_ws/log";
    std::string prefixStr = prefix.str();
    int runCount = 1;

    DIR *dir = opendir(logDir.c_str());
    if (dir != nullptr) {
        struct dirent *ent;
        while ((ent = readdir(dir)) != nullptr) {
            std::string fname(ent->d_name);
            if (fname.find(prefixStr + "_run") == 0 && fname.substr(fname.size() - 4) == ".csv") {
                ++runCount;
            }
        }
        closedir(dir);
    }

    std::ostringstream filename;
    filename << logDir << "/" << prefixStr << "_run" << runCount << ".csv";
    return filename.str();
}



Guider::Guider()
{
    ros::NodeHandle private_nh("~");
    //desiredSpeed=5; desiredHeading=1.33866753628
    private_nh.getParam("set_speed", desiredSpeed);
    private_nh.getParam("set_heading", desiredHeading);
    
    // chu kì dẫn đường và controlling_period là chu kỳ điều khiển
    private_nh.getParam("guiding_period", guiding_period);
    private_nh.param("controlling_period", controlling_period, controlling_period);
    //
    private_nh.getParam("LOS_radius", BaseLOS::radius);
    private_nh.getParam("LOS_min_delta", BaseLOS::minDelta);
    private_nh.getParam("LOS_max_delta", BaseLOS::maxDelta);
    private_nh.getParam("LOS_beta", BaseLOS::beta);
    private_nh.getParam("ned_lat", ned_lat);
    private_nh.getParam("ned_lon", ned_lon);

    private_nh.param("Kp", headingPID.Kp, headingPID.Kp);
    private_nh.param("Ki", headingPID.Ki, headingPID.Ki);
    private_nh.param("Kd", headingPID.Kd, headingPID.Kd);
    private_nh.param("heading_scale", heading_scale, 1.0);

    mkdir("/home/devbot/mybot_ws/log", 0777);
    std::string logFilename = generateLogFilename();
    guiderLog.open(logFilename);
    guiderLog << "time,x,y,yaw,desired_heading,CTE,ATE,left_vel,right_vel,P,I,D" << std::endl;
    if (!guiderLog.is_open()) {
        ROS_ERROR_STREAM("Khong the mo file log: " << logFilename);
    } else {
        ROS_INFO_STREAM("Dang ghi log tai: " << logFilename);
    }

dr_srv = new dynamic_reconfigure::Server<guider::GuiderConfig>(private_nh);
dynamic_reconfigure::Server<guider::GuiderConfig>::CallbackType f;
f = boost::bind(&Guider::onReconfig, this, _1, _2);
dr_srv->setCallback(f);


    ros::NodeHandle nh;
    //subscribe từ các topic /odom_2 ; /odom_1
    subOdom = nh.subscribe("odom_2", 10, &Guider::onOdomCallBack, this);    // receive data from GPS
    subGps = nh.subscribe("odom_1", 10, &Guider::onPosCallBack, this);    // receive data from GPS
    
    //publish lên topic /setpoint ; /error ; /diff_motor/vel 
    pubSP = nh.advertise<Setpoint>("setpoint", 1);           // dùng cho bộ điều khiển
    pubError = nh.advertise<Error>("error", 1);              // dùng cho bộ điều khiển và quan sát
    pubDiffVel = nh.advertise<DiffVel>("diff_motor/vel", 1); // cho việc điều khiển động cơ

    subItemList = nh.subscribe("mission/item_list", 10, &Guider::onItemListCallBack, this);
    sub_ID = nh.subscribe("/pointID", 10, &Guider::onCallbackpointID, this); 
    subSetpoint = nh.subscribe("setpoint", 10, &Guider::onSetpointCallBack, this);

/////
    sub_avoid = nh.subscribe("/position_avoid", 10, &Guider::onCallbackPosAvoid, this);
    subsonar = nh.subscribe("/sonar", 10, &Guider::onSonarCallBack, this);
    subPose = nh.subscribe("/amcl_pose", 10, &Guider::onCallbackPose, this );
    sub_mode = nh.subscribe("/mode_indoor", 10 , &Guider::onModeCallback, this);
    sub_goal_indoor_set = nh.subscribe("/goal_indoor", 10, &Guider::onCallbackGoalindoor, this);
    loopAddwaypointAvoid = nh.createTimer(ros::Duration(guiding_period*2), &Guider::addwaypoint, this);
    pub_complete = nh.advertise<Complete>("/Complete", 1);
/////

    //gọi hàm onControlLoop định kỳ để cập nhật tín hiệu điều khiển động cơ qua PID.
    loopControl = nh.createTimer(ros::Duration(controlling_period), &Guider::onControlLoop, this);
    //gọi hàm onGuidanceLoop định kỳ để cập nhật setpoint dựa trên LOS.
    loopGuidance = nh.createTimer(ros::Duration(guiding_period), &Guider::onGuidanceLoop, this);

    resStartMission = nh.advertiseService("command/start_mission", &Guider::onStartMissionCallBack, this);
    resSetSetpointParams = nh.advertiseService("parameter/set_setpoint_params", &Guider::onSetSetpointParamsCallBack, this);

    resSetLOSParams = nh.advertiseService("parameter/set_LOS_params", &Guider::onSetLOSParamsCallBack, this);
    resGetLOSParams = nh.advertiseService("parameter/get_LOS_params", &Guider::onGetLOSParamsCallBack, this);

    lastControlUpdateTime = lastSetpointTime = ros::Time::now();
    headingPID.threshold = diff_rpm;
}


// kệ nó
Guider::~Guider()
{
    if (ros::isStarted())
    {
        ros::shutdown();
        ros::waitForShutdown();
    }
}

/// indoor
void Guider::onCallbackGoalindoor(const goal_indoor::ConstPtr& goal_in){
  goal_indoor_set = goal_in->goal_indoor;

  waypoint_x_0 = goal_in->way_x_0;
  waypoint_x_1 = goal_in->way_x_1;
  waypoint_x_2 = goal_in->way_x_2;

  waypoint_y_0 = goal_in->way_y_0;
  waypoint_y_1 = goal_in->way_y_1;
  waypoint_y_2 = goal_in->way_y_2;

 // ROS_INFO_STREAM("Goal_indoor = " << goal_indoor_set);

  complete_indoor = waypoint_x_2 ;
}

void Guider::onModeCallback(const mode_indoor::ConstPtr& mode){
  mode_in = mode->mode_indoor ;
    if(mode_in == 1 && complete_indoor != complete_indoor_set){
      isMissionStarted = true ;
      straightLOSGuider.resetLOS();
      float waypoint_indoor_x[] = {waypoint_x_0, waypoint_x_1, waypoint_x_2 };
      float waypoint_indoor_y[] = {waypoint_y_0, waypoint_y_1 , waypoint_y_2 };
      int   size = sizeof(waypoint_indoor_x)/sizeof(waypoint_indoor_x[0]);
      float _x , _y;
      for (int i = 0; i < size ; i++)
      {
        _x = waypoint_indoor_x[i];
        _y = waypoint_indoor_y[i];
        ROS_INFO_STREAM("waypoint_indoor: x = " << _x << ", y = " << _y);   
        BaseLOS::waypoints.push_back(BaseLOS::Point(_x, _y));
      }
      sleep(5);
      straightLOSGuider.setupLOS();
    }  
}

void Guider::onCallbackPose(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& pos) {

    currX_indoor =  pos->pose.pose.position.x ;
    currY_indoor =  - pos->pose.pose.position.y ;

  // ROS_INFO_STREAM(currX_indoor);


  if (currX_indoor != 0.0){
      indoor = true;
    //   ROS_INFO_STREAM("Robot stay indoor !!!");
  }

    // ROS_INFO("odomX = %f odomY = %f" , currX_indoor,currY_indoor);
}

void Guider::onCallbackPosAvoid(const Position_avoid_obstacles::ConstPtr& pos_avoid){
    x_avoid = pos_avoid->position_avoid_x;
    y_avoid = pos_avoid->position_avoid_y;
    
    obstacles = pos_avoid->obstacles; 
}

void Guider::addwaypoint(const ros::TimerEvent& ){

    if(obstacles == true){
        if(mode_in == 1){
           
                ROS_INFO_STREAM("Obstacles !!!!!");
                isMissionStarted = true ;
              float waypoint_avoid_x[] = {currX_indoor, x_avoid, x_avoid};
              float waypoint_avoid_y[] = {currY_indoor, y_avoid , y_avoid + float(0.5)};
                straightLOSGuider.resetLOS();
              int   size = sizeof(waypoint_avoid_x)/sizeof(waypoint_avoid_x[0]);
              float _x , _y;
                  for (int i = 0; i < size ; i++)
                  {
                    _x = waypoint_avoid_x[i];
                    _y = waypoint_avoid_y[i];
                  ROS_INFO_STREAM("Waypoint_avoid_indoor: x = " << _x << ", y = " << _y);   
                  BaseLOS::waypoints.push_back(BaseLOS::Point(_x, _y));
                  }
                  straightLOSGuider.setupLOS();
                //  break;
         }
        else {            
            // float waypoint_avoid_x[] = {currX_outdoor, x_avoid};
            // float waypoint_avoid_y[] = {currY_outdoor, y_avoid};
            // straightLOSGuider.resetLOS();
            // int   size = sizeof(waypoint_avoid_x)/sizeof(waypoint_avoid_x[0]);
            // float _x , _y;
            // for (int i = 0; i < size ; i++) {
            //     _x = waypoint_avoid_x[i];
            //     _y = waypoint_avoid_y[i];
            //     ROS_INFO_STREAM("Waypoint_avoid_outdoor: x = " << _x << ", y = " << _y);   
            // }
            BaseLOS::waypoints.insert(BaseLOS::waypoints.begin() + straightLOSGuider.pointId, BaseLOS::Point(x_avoid, x_avoid));
            straightLOSGuider.setLOS();

      //  }
  }
      }
}

void Guider::onSonarCallBack(const obtacles::ConstPtr& msg){
    sonar_left = msg->sonar_left;
    sonar_right = msg->sonar_right;
}

///


// Các hàm callback chính


void Guider::onCallbackpointID(const utils::pointID::ConstPtr &id)   // thêm tọa độ waypoint mới vào
{
    straightLOSGuider.pointId = id->pointID;
}

void Guider::onOdomCallBack(const Odometry:: ConstPtr &msg)         // đọc heading từ IMU
{
    currHeading = msg->heading;
    // ROS_INFO("currHeading %f",currHeading);
    has_heading = true;
}
void Guider::onPosCallBack(const Odometry:: ConstPtr &msg)          // đọc x,y từ GPS
{
    currX_outdoor = msg->position.x;
    currY_outdoor = msg->position.y;
    // ROS_INFO("Posx:  %f",currX_outdoor);
    // ROS_INFO("Posy:  %f",currY_outdoor);
}

//đọc speed, heading từ topic setpoint(speed, heading) 
// speed là output PID
// heading là output LOS
void Guider::onSetpointCallBack(const Setpoint::ConstPtr& msg)
{
  lastSetpointTime = msg->header.stamp;

  speed = msg->linear_velocity.x;
  heading = msg->orientation.z;
}

void Guider::logToFile(double gpsX, double gpsY, double leftVel, double rightVel) {
    if (guiderLog.is_open()) {
        auto now = std::chrono::system_clock::now();
        std::time_t now_c = std::chrono::system_clock::to_time_t(now);
        auto ms = std::chrono::duration_cast<std::chrono::milliseconds>(now.time_since_epoch()) % 1000;
        std::tm* ptm = std::localtime(&now_c);

         guiderLog << std::put_time(ptm, "%H:%M:%S") << "." 
                  << std::setw(3) << std::setfill('0') << ms.count() << ","
                  << gpsX << "," << gpsY << ","
                  << currHeading * 180 / M_PI << ","
                  << heading * 180 / M_PI << ","
                  << straightLOSGuider.crossTrackError << ","    // CTE
                  << straightLOSGuider.alongTrackError << ","    // ATE
                  << leftVel << "," << rightVel << ","
                  << headingPID.P_part << "," 
                  << headingPID.I_part << "," 
                  << headingPID.D_part 
                  << std::endl;
    }
}


// Bộ điều khiển PID
void Guider::onControlLoop(const ros::TimerEvent& event)
{
  // auto start = std::chrono::steady_clock::now();

  if (!has_heading) {
    ROS_WARN("Chua nhan heading tu /odom2, waiting!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!...");
    return;
  }

  double dtc = (event.current_real - lastSetpointTime).toSec();
  if (dtc > 0.5)
  {
    DiffVel diffVelMsg;
    diffVelMsg.header.stamp = event.current_real;
    diffVelMsg.left_vel = 0;
    diffVelMsg.right_vel = 0;
    pubDiffVel.publish(diffVelMsg);
    return;
  }
  // if ((sonar_left) && (!sonar_right))
  // {
  //   DiffVel diffVelMsg;
  //   diffVelMsg.header.stamp = event.current_real;
  //   diffVelMsg.left_vel = 60;
  //   diffVelMsg.right_vel = 20;
  //   pubDiffVel.publish(diffVelMsg);
  //   return;
  // }
  // if ((!sonar_left) && (sonar_right))
  // {
  //   DiffVel diffVelMsg;
  //   diffVelMsg.header.stamp = event.current_real;
  //   diffVelMsg.left_vel = 20;
  //   diffVelMsg.right_vel = 60;
  //   pubDiffVel.publish(diffVelMsg);
  //   return;
  // }
  // if ((sonar_left) && (sonar_right))
  // {
  //   DiffVel diffVelMsg;
  //   diffVelMsg.header.stamp = event.current_real;
  //   diffVelMsg.left_vel = 0;
  //   diffVelMsg.right_vel = 0;
  //   pubDiffVel.publish(diffVelMsg);
  //   return;
  // }

  double dt = event.current_real.toSec() - lastControlUpdateTime.toSec();
  // ROS_INFO("Chu ky thuc te dt = %.6f giay", dt);
  // lấy error của heading để PID điều khiển 
  double deltaHeading = heading - currHeading;
  headingPID.error = atan2(sin(deltaHeading), cos(deltaHeading));
  headingPID.Ts = dt;
  headingPID.runPID();

  DiffVel diffVelMsg;
  diffVelMsg.header.stamp = event.current_real;
  // đổi 10 để tăng chậm
  
   double left_speed=(speed) / wheel_radius;
   double right_speed=(speed) / wheel_radius;

  double left =  left_speed + headingPID.output *  heading_scale;  // tune Kp,Kd và heading_scale
  double right = right_speed - headingPID.output * heading_scale;

diffVelMsg.left_vel  = clamp(left, 35.0, 85.0);
diffVelMsg.right_vel = clamp(right, 35.0, 85.0);

  //pub 
  pubDiffVel.publish(diffVelMsg);
  lastControlUpdateTime = event.current_real;

  // auto end = std::chrono::steady_clock::now();
  // double elapsed = std::chrono::duration<double, std::milli>(end - start).count();
  // ROS_INFO("[TIME] PID tính hết %.3f ms", elapsed);


ROS_INFO("[Vel] %.1f__%.1f | Head: %.2f - %.2f = %.2f | Err: x=%.2f y=%.2f | PID= %.2f %.2f %.2f | PID.output %.2f = %.2f + %.2f + %.2f",
         diffVelMsg.left_vel,
         diffVelMsg.right_vel,
         heading * 180 / M_PI,
         currHeading * 180 / M_PI,
         headingPID.error * 180 / M_PI,
         straightLOSGuider.alongTrackError,
         straightLOSGuider.crossTrackError,
         headingPID.Kp,
         headingPID.Ki,
         headingPID.Kd,
         headingPID.output,
         headingPID.P_part,
         headingPID.I_part,
         headingPID.D_part);

//   if (left != diffVelMsg.left_vel || right != diffVelMsg.right_vel)
// {
//   ROS_WARN("CLAMP! L: %.2f -> %.2f | R: %.2f -> %.2f",
//            left, diffVelMsg.left_vel,
//            right, diffVelMsg.right_vel);
// }

logToFile(currX, currY, diffVelMsg.left_vel, diffVelMsg.right_vel);

}

void Guider::onItemListCallBack(const utils::WaypointList::ConstPtr &msg)   // xử lí tọa độ global sang tọa độ local Đề - cát
{
    ROS_INFO_STREAM("Waypoint setup TIME !!!!!!!!!!!!!!!!!!!!");
    // lat_end_waypoint = msg->waypoints.back().x_lat ;

  float waypoint_indoor_x[] = {waypoint_x_0, waypoint_x_1, waypoint_x_2 };
  float waypoint_indoor_y[] = {waypoint_y_0, waypoint_y_1 , waypoint_y_2 };
  if(mode_in == 1){

          straightLOSGuider.resetLOS();
        int   size = sizeof(waypoint_indoor_x)/sizeof(waypoint_indoor_x[0]);
        float _x , _y;
        for (int i = 0; i < size ; i++)
        {
          _x = waypoint_indoor_x[i];
          _y = waypoint_indoor_y[i];
            ROS_INFO_STREAM("waypoint_indoor: x = " << _x << ", y = " << _y);   
            BaseLOS::waypoints.push_back(BaseLOS::Point(_x, _y));
          }
     straightLOSGuider.setupLOS();

for (auto it = msg->waypoints.begin(); it != msg->waypoints.end(); it++)
        {
          switch (it->command)
          {
          case 16: // MAV_CMD_NAV_WAYPOINT
          {
          //  BaseLOS::waypoints.clear();
            double x, y;
            
            convert_global_to_local_coords(it->x_lat, it->y_long, 10.7727, 106.66, x, y);
            ROS_INFO_STREAM("Waypoint___: x = " << x << ", y = " << y);
            BaseLOS::waypoints.push_back(BaseLOS::Point(x, y));
            break;
          }
          }
        }
    }
        
  else {
    straightLOSGuider.resetLOS(); // waypoits về 0
    for (auto it = msg->waypoints.begin(); it != msg->waypoints.end(); it++)
    {
        switch (it->command)
        {
        case 16: // MAV_CMD_NAV_WAYPOINT
        {
        double x, y;
        ROS_INFO_STREAM("hoho:" << it->x_lat << it->y_long);
        convert_global_to_local_coords(it->x_lat, it->y_long, ned_lat, ned_lon, x, y);
        ROS_INFO_STREAM("Waypoint: x = " << x << ", y = " << y);
        BaseLOS::waypoints.push_back(BaseLOS::Point(x, y));
        break;
        }

        case 20: // MAV_CMD_NAV_RETURN_TO_LAUNCH
        {
        BaseLOS::waypoints.push_back(BaseLOS::waypoints[0]);
        break;
        }
        }
    }
            
        straightLOSGuider.setLOS();
 }
}


void Guider::onGuidanceLoop(const ros::TimerEvent &event)         // hàm chạy
{
    // auto start = std::chrono::steady_clock::now();

    if (customMode == "1") // AUTO_HEADING 
    {
        publishSetpoint(desiredSpeed, desiredHeading);
    }


    if (customMode == "2") // LOS_STRAIGHT
    {
        if (mode_in == 1)
        {
            currX = currX_indoor;
            currY = currY_indoor;
        }

        else
        {
            currX = currX_outdoor;
            currY = currY_outdoor;
        }
        if (straightLOSGuider.runLOS(currX, currY))
        {
           
            publishSetpoint(desiredSpeed, straightLOSGuider.desiredHeading);
            publishError(straightLOSGuider.alongTrackError, straightLOSGuider.crossTrackError);
            // ROS_INFO("currX = %f currY = %f" , currX,currY);
            //ROS_INFO("alongTrackError %f",straightLOSGuider.alongTrackError);
            //ROS_INFO("crossTrackError %f",straightLOSGuider.crossTrackError);
        }
    }
    //  auto end = std::chrono::steady_clock::now();
    // double elapsed = std::chrono::duration<double, std::milli>(end - start).count();
    // ROS_INFO("[TIME] LOS tính hết %.3f ms", elapsed);
}







// chưa biết

bool Guider::onStartMissionCallBack(CommandLong::Request &req, CommandLong::Response &res)    // nhận nhiệm vụ để bắt đầu chạy
{
    isMissionStarted = true;
    res.result = 0; // MAV_RESULT_ACCEPTED
    ROS_INFO("Mission started");
    return true;
}

bool Guider::onSetLOSParamsCallBack(ParamSet::Request &req, ParamSet::Response &res)
{
    res.success = false;

    if (compare_string(req.param_id.data(), "LOS_radius"))
    {
        BaseLOS::radius = req.value.real;
        res.success = true;
    }
    if (compare_string(req.param_id.data(), "LOS_min_delta"))
    {
        BaseLOS::minDelta = req.value.real;
        res.success = true;
    }
    if (compare_string(req.param_id.data(), "LOS_max_delta"))
    {
        BaseLOS::maxDelta = req.value.real;
        res.success = true;
    }
    if (compare_string(req.param_id.data(), "LOS_beta"))
    {
        BaseLOS::beta = req.value.real;
        res.success = true;
    }

    res.value.real = req.value.real;
    return res.success;
}

bool Guider::onGetLOSParamsCallBack(ParamGet::Request& req, ParamGet::Response& res)
{
  res.success = false;

  if (compare_string(req.param_id.data(), "LOS_radius"))
  {
    res.value.real = BaseLOS::radius;
    res.success = true;
  }
  if (compare_string(req.param_id.data(), "LOS_min_delta"))
  {
    res.value.real = BaseLOS::minDelta;
    res.success = true;
  }
  if (compare_string(req.param_id.data(), "LOS_max_delta"))
  {
    res.value.real = BaseLOS::maxDelta;
    res.success = true;
  }
  if (compare_string(req.param_id.data(), "LOS_beta"))
  {
    res.value.real = BaseLOS::beta;
    res.success = true;
  }
}

bool Guider::onGetSetpointParamsCallBack(ParamGet::Request &req, ParamGet::Response &res)
{
    res.success = false;

    if (compare_string(req.param_id.data(), "set_speed"))
    {
        res.value.real = desiredSpeed;
        res.success = true;
    }
    if (compare_string(req.param_id.data(), "set_heading"))
    {
        res.value.real = desiredHeading;
        res.success = true;
    }

    return res.success;
}

bool Guider::onSetSetpointParamsCallBack(ParamSet::Request &req, ParamSet::Response &res)
{
    res.success = false;

    if (compare_string(req.param_id.data(), "set_speed"))
    {
        desiredSpeed = req.value.real;
        res.success = true;
    }
    if (compare_string(req.param_id.data(), "set_heading"))
    {
        desiredHeading = req.value.real;
        res.success = true;
    }

    res.value.real = req.value.real;
    return res.success;
}
 
//










// Publish setpoint (speed,heading)
void Guider::publishSetpoint(const double &speed, const double &heading)
{    
  // float waypoint_outdoor_x[5] = {0, -50.14959,-90.06785, 0, 1};
  //   float waypoint_outdoor_y[5] = {0, 73.16111, 48.04616, 1, 0};


    Setpoint setpointMsg;
    setpointMsg.header.stamp = ros::Time::now();
    setpointMsg.linear_velocity.x = speed;
    setpointMsg.orientation.z = heading;
    //ROS_INFO("setheading %f",heading);
    pubSP.publish(setpointMsg);
}


//Publish Error
void Guider::publishError(const double &xe, const double &ye)
{
    Error errorMsg;
    errorMsg.header.stamp = ros::Time::now();
    errorMsg.along_track = xe;
    errorMsg.cross_track = ye;
    pubError.publish(errorMsg);
}


int main(int argc, char **agrv)
{
    ros::init(argc, agrv, "guider_node");

    Guider guide;
    guide.straightLOSGuider.resetLOS();
    guide.straightLOSGuider.setupLOS();
    ros::spin();
    return 0;
}

void Guider::onReconfig(guider::GuiderConfig &config, uint32_t level)
{
    headingPID.Kp = config.Kp;
    headingPID.Ki = config.Ki;
    headingPID.Kd = config.Kd;
    heading_scale = config.heading_scale;

    // guiding_period = config.guiding_period;
    // controlling_period = config.controlling_period;
    desiredSpeed = config.set_speed;

    BaseLOS::minDelta = config.LOS_min_delta;
    BaseLOS::maxDelta = config.LOS_max_delta;
    BaseLOS::radius = config.LOS_radius;
}