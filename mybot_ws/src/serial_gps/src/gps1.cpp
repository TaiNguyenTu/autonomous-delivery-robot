#include <ros/ros.h>
#include <serial/serial.h>
#include <std_msgs/String.h>
#include <QByteArray>
#include <QList>
#include <sstream>
#include <vector>
#include <string>
#include <iomanip>
#include <sensor_msgs/NavSatFix.h>
#include <utils/Odometry.h>
#include <utils/DiffVel.h>
#include <cmath>
#include <geometry_msgs/PoseWithCovarianceStamped.h>



//
// Throughput – Tổng số byte nhận mỗi giây

// Loss Rate – Số gói GGA nhận được mỗi giây (so với kỳ vọng, ví dụ 5Hz)

// Delay Detection – Đo thời gian giữa 2 lần nhận frame GGA


// ✅ Loss Rate	        ✔️ Rất cần thiết	        Xác định mất gói
// ✅ Delay Detection	✔️ Có thể đo delta > 0.2	Nhận diện gói chậm
// ✅ Throughput	        ✔️ Đơn giản	                Báo lượng byte/s
// ❌ Checksum	        ❌ Không cần với NMEA	  Đã có sẵn
// ❌ Latency	        ❌ Không có timestamp GPS  Khó sync
// ❌ Time Coherence	    ❌ Cần GPS có clock thực	  Không khả thi

int count_gga = 0;
int total_bytes_received = 0;
ros::Time last_gga_time;
const int expected_gps_packets_per_sec = 5;



constexpr double a = 6378137.0; // Bán kính trục lớn (WGS-84)
constexpr double f = 1.0 / 298.257223563;
constexpr double b = a * (1 - f); // Bán kính trục nhỏ
constexpr double e_sq = f * (2 - f); // Độ lệch tâm bình phương



// inline void convert_global_to_local_coords(const double& lat, const double& lon, const double& ref_lat,const double ref_lon, double& x, double& y) {

//     if ((lat == ref_lat) && (lon == ref_lon))
//         x = y = 0;

//     double lat_rad = lat * M_PI / 180.0;
//     double lon_rad = lon * M_PI / 180.0;

//     double ref_lon_rad = ref_lon * M_PI / 180.0;
//     double ref_lat_rad = ref_lat * M_PI / 180.0;

//   double sin_lat = sin(lat_rad);
//   double cos_lat = cos(lat_rad);
//   double cos_d_lon = cos(lon_rad - ref_lon_rad);

//   double ref_sin_lat = sin(ref_lat_rad);
//   double ref_cos_lat = cos(ref_lat_rad);

//   double c = acos(ref_sin_lat * sin_lat + ref_cos_lat * cos_lat * cos_d_lon);
//   double k = (fabs(c) < std::numeric_limits<double>::epsilon()) ? 1.0 : (c / sin(c));

//   x = k * (ref_cos_lat * sin_lat - ref_sin_lat * cos_lat * cos_d_lon) * 6371000;
//   y = k * cos_lat * sin(lon_rad - ref_lon_rad) * 6371000;
  
  
// }



inline void convert_global_to_local_coords(const double& lat, const double& lon, const double& ref_lat,const double ref_lon, double& x, double& y) {

// Giả định độ cao (altitude) = 0, vì không truyền vào
    double alt = 0.0;
    double ref_alt = 0.0;
    
    // Chuyển độ sang radian
    double lat_rad = lat * M_PI / 180.0;
    double lon_rad = lon * M_PI / 180.0;
    double ref_lat_rad = ref_lat * M_PI / 180.0;
    double ref_lon_rad = ref_lon * M_PI / 180.0;

    // ECEF của điểm hiện tại
    double sin_lat = sin(lat_rad), cos_lat = cos(lat_rad);
    double sin_lon = sin(lon_rad), cos_lon = cos(lon_rad);
    double N = a / sqrt(1 - e_sq * sin_lat * sin_lat);

    double x_ecef = (N + alt) * cos_lat * cos_lon;
    double y_ecef = (N + alt) * cos_lat * sin_lon;
    double z_ecef = (N * (1 - e_sq) + alt) * sin_lat;

    // ECEF của điểm tham chiếu
    double sin_ref_lat = sin(ref_lat_rad), cos_ref_lat = cos(ref_lat_rad);
    double sin_ref_lon = sin(ref_lon_rad), cos_ref_lon = cos(ref_lon_rad);
    double N_ref = a / sqrt(1 - e_sq * sin_ref_lat * sin_ref_lat);

    double x_ref = (N_ref + ref_alt) * cos_ref_lat * cos_ref_lon;
    double y_ref = (N_ref + ref_alt) * cos_ref_lat * sin_ref_lon;
    double z_ref = (N_ref * (1 - e_sq) + ref_alt) * sin_ref_lat;

    // Tính chênh lệch
    double dx = x_ecef - x_ref;
    double dy = y_ecef - y_ref;
    double dz = z_ecef - z_ref;

    // ECEF → ENU
    x = -cos_ref_lon * sin_ref_lat * dx - sin_ref_lon * sin_ref_lat * dy + cos_ref_lat * dz;
    y = -sin_ref_lon * dx + cos_ref_lon * dy;


    // Nếu cần trục Z (altitude tương đối), có thể thêm đầu ra z nếu cần sau
}


inline double convert_num_5(double dnum) {
    return static_cast<double>(int(dnum*100000))/100000;

}

inline double convert_num_6(double dnum) {
    return static_cast<double>(int(dnum*1000000))/1000000;
}

inline double convert_num_7(double dnum) {
    return static_cast<double>(int(dnum * 10000000)) / 10000000;
}

inline double CalLat2Deg(double Lat)
  {
    static uint8_t Deg = 0;
    static double Min = 0, Result = 0;
    Deg = Lat / 100;
    Min = Lat - Deg * 100;
    Result = Deg + Min / 60;
    return convert_num_7(Result);
  }



inline double CalLong2Deg(double Long)
{
    static uint16_t Deg = 0;
    static double Min = 0, Result = 0;
    Deg = Long / 100;
    Min = Long - Deg * 100;
    Result = Deg + Min / 60;
    return convert_num_7(Result);
}
//
void evaluate_gps(const ros::TimerEvent&) {
    float loss = 100.0f * (expected_gps_packets_per_sec - count_gga) / expected_gps_packets_per_sec;
    if (count_gga == 0)
        ROS_ERROR("Khong nhan duoc goi GGA");
    else if (count_gga < expected_gps_packets_per_sec)
        ROS_WARN_STREAM("Mat goi GGA: " << loss << "%");
    else
        ROS_INFO("GGA du goi (5Hz)");

    ROS_INFO_STREAM("Throughput GPS: " << total_bytes_received << " bytes/s");
    count_gga = 0;
    total_bytes_received = 0;
}
//

void serialCallback(const std::string& data, serial::Serial& serialPort, ros::Publisher& gps_pub,  ros::Publisher& data_pub, ros::Publisher& pubOdom, double ref_lat, double ref_lon) {

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

        

        // // Delay detection
        // ros::Time now = ros::Time::now();
        // if (!last_gga_time.isZero()) {
        //     double delta = (now - last_gga_time).toSec();
        //     double tre = delta - 0.2;
        //     if (tre > 0.01) {
        //         ROS_WARN_STREAM("Goi GGA den tre: " << std::fixed << std::setprecision(6) << tre << "s");
        //     }
        // }
        // last_gga_time = now;



if (dataList[0] == "GGA") {
    // // Chỉ tính nếu là GGA
    // count_gga++;
    // total_bytes_received += data.size();

    // Kiểm tra dữ liệu có đủ không?
    if (dataList.size() >= 10 &&
        !dataList[2].empty() && 
        !dataList[4].empty() && 
        !dataList[9].empty()) 
    {
        ROS_INFO_STREAM("---------------------------------------------------------------");
        // ROS_INFO_STREAM("Type: " << " || " << dataList[0]);

            try {
                double latitude = CalLat2Deg(std::stof(dataList[2]));
                double longtitude = CalLong2Deg(std::stof(dataList[4]));
                double altitude = std::stof(dataList[9]);

                utils::Odometry odomMsg;
                sensor_msgs::NavSatFix gps_msg;
                odomMsg.latitude = latitude;
                odomMsg.longitude = longtitude;
                odomMsg.altitude = altitude;
                gps_msg.latitude = latitude;
                gps_msg.longitude = longtitude;
                gps_msg.altitude = altitude;

                convert_global_to_local_coords(convert_num_5(odomMsg.latitude),convert_num_5(odomMsg.longitude), ref_lat,ref_lon,
                                     odomMsg.position.x, odomMsg.position.y);   // lưu x,y vào odomMsg.position.x, odomMsg.position.y
                odomMsg.position.z = -odomMsg.altitude;

                double x1 = odomMsg.position.x;
                double y1 = odomMsg.position.y;

                gps_pub.publish(gps_msg);
                pubOdom.publish(odomMsg);

                std::ostringstream lat_str;
                lat_str << std::fixed << std::setprecision(7) << x1;
                std::ostringstream long_str;
                long_str << std::fixed << std::setprecision(7) << y1;

                //in fix type
                 if (dataList[6] == "1") {
                    ROS_WARN_STREAM("GPS not RTK (Fix type = " << dataList[6] << ")");
                } else if (dataList[6] == "5") {
                    ROS_WARN_STREAM("GPS RTK (Fix type = " << dataList[6] << ")");
                } else {
                    ROS_WARN_STREAM("GPS khac (Fix type = " << dataList[6] << ")");
                }
                // In số vệ tinh
                if (dataList.size() > 7 && !dataList[7].empty()) {
                    try {
                        int satellites = std::stoi(dataList[7]);
                        ROS_INFO_STREAM("So ve tinh dang theo doi: " << satellites);
                        if (satellites < 6) {
                            ROS_WARN_STREAM("So ve tinh thap (<6): " << satellites);
                        }
                    } catch (...) {
                        ROS_WARN("Khong the parse so ve tinh.");
                    }
                }
                // In x,y
                ROS_INFO_STREAM("x: " << lat_str.str());
                ROS_INFO_STREAM("y: " << long_str.str());
                ROS_INFO_STREAM("---------------------------------------------------------------");
            
            } catch (const std::invalid_argument& e) {
                ROS_INFO_STREAM("Invalid argument: " << e.what());
            } catch (const std::out_of_range& e) {
                ROS_INFO_STREAM("Out of range: " << e.what());
            }        
    } else {
    ROS_WARN_STREAM("GPS mat tin hieu. Size = " << dataList.size() << ". Bo qua frame nay.");
    return;
    }
}
    //dạng này không dùng
    // else if (dataList[0] == "GL") {
    //     // ROS_INFO_STREAM("data: " << data);
    //     ROS_INFO_STREAM("asdfgh: " << " || " << dataList[0]);
    //     if (dataList.size() <= 30) {
    //         try {
    //             double latitude = CalLat2Deg(std::stof(dataList[1]));
    //             double longtitude = CalLong2Deg(std::stof(dataList[3]));
    //             double altitude = 0;
    //             utils::Odometry odomMsg;
    //             sensor_msgs::NavSatFix gps_msg;
    //             odomMsg.latitude = latitude;
    //             odomMsg.longitude = longtitude;
    //             odomMsg.altitude = altitude;
    //             gps_msg.latitude = latitude;
    //             gps_msg.longitude = longtitude;
    //             gps_msg.altitude = altitude;
    //             convert_global_to_local_coords(convert_num_5(odomMsg.latitude),convert_num_5(odomMsg.longitude), ref_lat,ref_lon,
    //                                  odomMsg.position.x, odomMsg.position.y);
    //             odomMsg.position.z = -odomMsg.altitude;
    //             double x1 = convert_num_5(odomMsg.position.x);
    //             double y1 = convert_num_5(odomMsg.position.y);

    //             gps_pub.publish(gps_msg);
    //             pubOdom.publish(odomMsg);
    //             std::ostringstream lat_str;
    //             lat_str << std::fixed << std::setprecision(5) << x1;
    //             std::ostringstream long_str;
    //             long_str << std::fixed << std::setprecision(5) << y1;
    //             std::ostringstream alt_str;
    //             alt_str << std::fixed << std::setprecision(3) << altitude;

    //             ROS_INFO_STREAM("x1: " << lat_str.str());
    //             ROS_INFO_STREAM("y1: " << long_str.str());
    //             ROS_INFO_STREAM("Atitude value: " << alt_str.str());
    //         } catch (const std::invalid_argument& e) {
    //             ROS_INFO_STREAM("Invalid argument: " << e.what());
    //         } catch (const std::out_of_range& e) {
    //             ROS_INFO_STREAM("Out of range: " << e.what());
    //         }
    //     }
    //     ROS_INFO_STREAM("Received GLL"  << data.size()  << " bytes " << "so phan tu: " << dataList.size());
    // }
}

void onCallbackPose(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg) {
// 
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "gps_node1");
    ros::NodeHandle nh;
    ros::NodeHandle private_nh("~");
    bool gps_enabled;
    std::string gps_port;
    int gps_baudrate;
    double ref_lon, ref_lat;
    private_nh.getParam("enable_gps", gps_enabled);
    private_nh.getParam("gps_port", gps_port);
    private_nh.getParam("gps_baud", gps_baudrate);
    private_nh.getParam("ref_lat", ref_lat);
    private_nh.getParam("ref_lon", ref_lon);

    // ros::Timer eval_timer = nh.createTimer(ros::Duration(1.0), evaluate_gps);
    
    serial::Serial serialPort;
    try {
        serialPort.setPort(gps_port);
        serialPort.setBaudrate(gps_baudrate);
        serial::Timeout timeout = serial::Timeout::simpleTimeout(1000);
        serialPort.setTimeout(timeout);
        serialPort.open();
    } catch (serial::IOException& e) {
        ROS_ERROR_STREAM("Unable to open serial port: " << e.what());
        return -1;
    }


    if (!serialPort.isOpen()) {
        ROS_ERROR_STREAM("Failed to open serial port.");
        return -1;
    }
    // Publish lên các topic /gps_1 ; /gps ; /odom_1 ; /amcl_pose
    ros::Publisher data_pub = nh.advertise<std_msgs::String>("gps_1", 10);
    ros::Publisher gps_pub = nh.advertise<sensor_msgs::NavSatFix>("gps", 100); //rostopic echo /gps
    ros::Publisher pubOdom = nh.advertise<utils::Odometry>("odom_1", 10);
    ros::Subscriber sub_pose = nh.subscribe("/amcl_pose", 10, onCallbackPose);
    
    try {
        while (ros::ok()) {
            if (1) {
                std::string data = serialPort.readline(100, "$GN");
                serialCallback(data, serialPort, gps_pub, data_pub, pubOdom, ref_lat, ref_lon);
            //   ROS_INFO_STREAM("data: " << data);
            }
            ros::spinOnce();
        }
    } catch (serial::IOException& e) {
        ROS_ERROR_STREAM("Error reading from serial port: " << e.what());
    }

    serialPort.close();
    
    return 0;
}
