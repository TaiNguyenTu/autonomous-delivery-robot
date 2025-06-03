#include <string>
#include <iomanip>
#include <sensor_msgs/NavSatFix.h>
#include <utils/Odometry.h>
#include <utils/DiffVel.h>
#include <cmath>
#include <geometry_msgs/PoseWithCovarianceStamped.h>

inline void convert_global_to_local_coords(const double& lat, const double& lon, const double& ref_lat,
                                           const double ref_lon, double& x, double& y) {

    if ((lat == ref_lat) && (lon == ref_lon))
        x = y = 0;

    double lat_rad = lat * M_PI / 180.0;
    double lon_rad = lon * M_PI / 180.0;

    double ref_lon_rad = ref_lon * M_PI / 180.0;
    double ref_lat_rad = ref_lat * M_PI / 180.0;

    // double _y = sin(lon_rad - ref_lon_rad ) * cos(lat_rad);
    // double _x = cos(ref_lat_rad) *sin (lat_rad);
    
    // double bearing = atan2(_y,_x);
    // double a = sin((lat_rad-ref_lat_rad)/2)*sin((lat_rad - ref_lat_rad)/2) + cos(ref_lat_rad)*cos(lat_rad)*sin((lon_rad - ref_lon_rad)/2)*sin((lon_rad-ref_lon_rad)/2);
    // double c = 2*atan2(sqrt(a),sqrt(1-a));
    // double distance = 6371000 * c;

    // double Heading = 2*M_PI-bearing;
    // x = distance*cos(Heading);
    // y = distance*sin(Heading);

  double sin_lat = sin(lat_rad);
  double cos_lat = cos(lat_rad);
  double cos_d_lon = cos(lon_rad - ref_lon_rad);

  double ref_sin_lat = sin(ref_lat_rad);
  double ref_cos_lat = cos(ref_lat_rad);

  double c = acos(ref_sin_lat * sin_lat + ref_cos_lat * cos_lat * cos_d_lon);
  double k = (fabs(c) < std::numeric_limits<double>::epsilon()) ? 1.0 : (c / sin(c));

  x = k * (ref_cos_lat * sin_lat - ref_sin_lat * cos_lat * cos_d_lon) * 6371000;
  y = k * cos_lat * sin(lon_rad - ref_lon_rad) * 6371000;
  
}
inline double CalLat2Deg(double Lat)
  {
    static uint8_t Deg = 0;
    static double Min = 0, Result = 0;
    Deg = Lat / 100;
    Min = Lat - Deg * 100;
    Result = Deg + Min / 60;
    return Result;
  }
inline double CalLong2Deg(double Long)
{
    static uint16_t Deg = 0;
    static double Min = 0, Result = 0;
    Deg = Long / 100;
    Min = Long - Deg * 100;
    Result = Deg + Min / 60;
    return Result;
}

inline double convert_num_5(double dnum) {
    return static_cast<double>(int(dnum*100000))/100000;
}
inline double convert_num_4(double dnum) {
    return static_cast<double>(int(dnum*100000))/100000;
}
void serialCallback(const std::string& data, serial::Serial& serialPort, ros::Publisher& base_pub,  ros::Publisher& data_pub, double ref_lat, double ref_lon) {

    std::vector<std::string> dataList;
    std::stringstream ss(data);
    std::string token;
    while (std::getline(ss, token, ',')) {
        dataList.push_back(token);
    }
    // std_msgs::String msg;
    // for (const std::string& data : dataList) {
    //     msg.data = data;
    //     data_pub.publish(msg);
    // }
    if (data.size() >= 30) {
        ROS_INFO_STREAM("Data: " << " || " << dataList[0]);
        if (dataList[0] == "GNGLL") {
            try {
                double latitude = CalLat2Deg(std::stof(dataList[2]));
                double longtitude = CalLong2Deg(std::stof(dataList[4]));
                sensor_msgs::NavSatFix gps_msg;
                gps_msg.latitude = latitude;
                gps_msg.longitude = longtitude;

                // convert_global_to_local_coords(convert_num_5(odomMsg.latitude),convert_num_5(odomMsg.longitude), ref_lat,ref_lon,
                //                      odomMsg.position.x, odomMsg.position.y);
                // double x1 = convert_num_5(odomMsg.position.x);
                // double y1 = convert_num_5(odomMsg.position.y);

                base_pub.publish(gps_msg);
                std::ostringstream lat_str;
                lat_str << std::fixed << std::setprecision(5) << x1;
                std::ostringstream long_str;
                long_str << std::fixed << std::setprecision(5) << y1;
                ROS_INFO_STREAM("Latitude value: " << lat_str.str());
                ROS_INFO_STREAM("Longtitude value: " << long_str.str());
            } catch (const std::invalid_argument& e) {
                ROS_INFO_STREAM("Invalid argument: " << e.what());
            } catch (const std::out_of_range& e) {
                ROS_INFO_STREAM("Out of range: " << e.what());
            }
        }
        else if (dataList == "GNGGA") {
            
        }
        
    }
        ROS_INFO_STREAM("Received "  << data.size()  << " bytes " << "so phan tu: " << dataList.size());
}
int main(int argc, char** argv) {
    ros::init(argc, argv, "base_node");
    ros::NodeHandle nh;
    ros::NodeHandle private_nh("~");
    bool base_enabled;
    std::string base_port;
    int base_baudrate;
    double ref_lon, ref_lat;
    private_nh.getParam("enable_base", base_enabled);
    private_nh.getParam("base_port", base_port);
    private_nh.getParam("base_baud", base_baudrate);
    private_nh.getParam("ref_lat", ref_lat);
    private_nh.getParam("ref_lon", ref_lon);


    serial::Serial serialPort;
    try {
        serialPort.setPort("/dev/ttyUSB2");
        serialPort.setBaudrate(115200);
        serial::Timeout timeout = serial::Timeout::simpleTimeout(1000);
        serialPort.setTimeout(timeout);
        ROS_INFO_STREAM("12356");
        serialPort.open();
    } catch (serial::IOException& e) {
        ROS_ERROR_STREAM("Unable to open serial port: " << e.what());
        return -1;
    }
    if (!serialPort.isOpen()) {
        ROS_ERROR_STREAM("Failed to open serial port.");
        return -1;
    }
    ros::Publisher data_pub = nh.advertise<std_msgs::String>("base_1", 10);
    ros::Publisher base_pub = nh.advertise<sensor_msgs::NavSatFix>("base", 100);
    try {
        while (ros::ok()) {
            if (1) {
                ROS_INFO_STREAM("lmao:");
                std::string data = serialPort.readline(200, "$");
                ROS_INFO_STREAM("data: " << data);
                serialCallback(data, serialPort, base_pub, data_pub, ref_lat, ref_lon);
            }
            ros::spinOnce();
        }
    } catch (serial::IOException& e) {
        ROS_ERROR_STREAM("Error reading from serial port: " << e.what());
    }

    serialPort.close();
    
    return 0;
}