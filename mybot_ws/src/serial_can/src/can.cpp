#include <ros/ros.h>
#include <serial/serial.h>
#include <sstream>
#include <vector>
#include <string>
#include <iomanip>
#include <mutex>
#include <sensor_msgs/NavSatFix.h>
#include <sensor_msgs/Imu.h>
#include <utils/DiffVel.h>
#include <utils/Odometry.h>
#include <cmath>
#include <utils/Encoder.h>
#include <boost/bind.hpp>
#include <boost/algorithm/string.hpp>
#include <utils/tf.h>
#include "std_msgs/String.h"
#include <can.h>
#include <fstream>
#include <ctime>
using namespace std;


vel_pid left_vel_PID;
vel_pid right_vel_PID;
imudata imu_data;


int total_bytes_received = 0;
int throughput_bytes_last_sec = 0;
int count_velocity = 0;           // Đếm số gói vận tốc nhận được mỗi giây
int count_yaw = 0;                // Đếm số gói yaw nhận được mỗi giây
const int expected_packets_per_sec = 5;  // Số gói kỳ vọng mỗi giây (tùy theo chu kỳ gửi từ STM32)


ros::Time last_vel_time;
ros::Time last_yaw_time;

int log_counter = 0;


std::ofstream log_file("/home/devbot/mybot_ws/log/can_log.csv", std::ios::out | std::ios::trunc);

void logHeader(std::ofstream& log_file) {
    if (log_file.is_open()) {
        log_file << "time,left_vel,right_vel,yaw,throughput,lossrate,data_integrity,sanity_check,delay_detection\n";
        log_file.flush();
    }
}


void logData(std::ofstream& log_file, double time, float left_v, float right_v, float yaw, int throughput, float lossrate, 
             const std::string& integrity, const std::string& sanity, const std::string& delay) {
          
    if (log_file.is_open()) {
        log_file << std::fixed << std::setprecision(3) << time << ","
                 << std::setprecision(4) << left_v << ","
                 << right_v << ","
                 << yaw << ",";
        if (log_counter % 5 == 0) {
            log_file << throughput << "," << lossrate << ",";
        } else {
            log_file << ",,";
        }
        log_file << integrity << ","
                 << sanity << ","
                 << delay << "\n";
        log_file.flush();  // Bắt buộc ghi ngay
        log_counter++;
    }
}


//// Hàm normalizeAngle: đưa góc về khoảng (-180, 180)
double normalizeAngle(double angle) {
    angle = fmod(angle, 360.0);  
    if (angle > 180.0) {
        angle -= 360.0; 
    } else if (angle < -180.0) {
        angle += 360.0;
    }
    return angle;
}

// Hàm chuyển đổi số thực sang mảng ký tự dạng số (dạng ASCII) với 1 chữ số thập phân
void float_to_array(float numf, uint8_t* array) {
    if (1) {
    int chuc = static_cast<int>(numf);
    float donvi = numf - chuc;
    int tens = chuc / 10;
    int units = chuc % 10;
    long int thapphan = static_cast<long int>(donvi * 10000);
    array[0] = static_cast<uint8_t>(tens) + 0x30;
    array[1] = static_cast<uint8_t>(units) + + 0x30;
    array[2] = 0x2E;
    array[3] = static_cast<uint8_t>(thapphan / 1000)+ 0x30;
    array[4] = static_cast<uint8_t>((thapphan / 100) % 10)+ 0x30;
    array[5] = static_cast<uint8_t>((thapphan / 10) % 10)+ 0x30;
    array[6] = static_cast<uint8_t>(thapphan % 10)+ 0x30;
    }
}

// Hàm tach_data_byte:
// Nếu gặp byte điều khiển (0xA5) và byte tiếp theo thuộc tập {0xA5, 0xAA, 0x55},
// thì bỏ qua byte escape 0xA5 và chỉ giữ byte sau.
vector<uint8_t> tach_data_byte(const vector<uint8_t>& data_byte) {
    vector<uint8_t> unescaped;
    for (size_t i = 0; i < data_byte.size(); i++) {
        if (data_byte[i] == 0xA5) {
            if (i + 1 < data_byte.size() && (data_byte[i+1] == 0xA5 || data_byte[i+1] == 0xAA || data_byte[i+1] == 0x55)) {
                i++; // Bỏ qua byte điều khiển, chỉ thêm byte sau đó.
                unescaped.push_back(data_byte[i]);
            } else {
                unescaped.push_back(data_byte[i]);
            }
        } else {
            unescaped.push_back(data_byte[i]);
        }
    }
    return unescaped;
}

//Dùng cho subscriber . Khi nhận dữ liệu float từ topic. Đổi thành định dạng gói CAN 
vector<uint8_t> createCanFrame(uint8_t id, float v1, float v2) {
        vector<uint8_t> data;
        data.reserve(40);
        //framehead
        data.push_back(0xAA);
        data.push_back(0xAA);
        //ID=0x01 là vel
        data.push_back(id);
        //mặc định bằng 0
        data.push_back(0x00);
        data.push_back(0x00);
        data.push_back(0x00);
        //data
        auto pushEscaped = [&](uint8_t byte) {   // Hàm lambda để thêm escape khi có ký tự điều khiển
        if(byte == 0xAA || byte == 0x55 || byte == 0xA5) {
            data.push_back(0xA5);
        }
        data.push_back(byte);
        };
       
        uint8_t left_byte[4];
        uint8_t right_byte[4];
        memcpy(left_byte, &v1, sizeof(float));
        memcpy(right_byte, &v2, sizeof(float));

        for (int i = 0; i < 4; i++) {
            pushEscaped(left_byte[i]);
        }
        for (int i = 0; i < 4; i++) {
            pushEscaped(right_byte[i]);
        }
        // checksum
        uint16_t sum = id+8;
        for (int i = 0; i < 4; i++) {
            sum += left_byte[i];
        }
        for (int i = 0; i < 4; i++) {
            sum += right_byte[i];
        }
        uint8_t checksumValue = sum & 0xFF;
        data.push_back(0x08);
        data.push_back(0x00);
        data.push_back(0x00);
        data.push_back(0x00);
        pushEscaped(checksumValue);
        // frametail
        data.push_back(0x55);
        data.push_back(0x55);
        return data;
}


// Các hàm đánh giá dữ liệu ///////////////////////////////////////////////////////////////////////////////////////////////////

void evaluate_Throughput(const ros::TimerEvent&) {
    // ROS_WARN_STREAM("Throughput: " << total_bytes_received << " bytes/s");
    throughput_bytes_last_sec = total_bytes_received;
    total_bytes_received = 0;
}

void evaluate_LossRate(const ros::TimerEvent&) {
    float loss_v = 100.0f * (expected_packets_per_sec - count_velocity) / expected_packets_per_sec;
    float loss_yaw = 100.0f * (expected_packets_per_sec - count_yaw) / expected_packets_per_sec;

    if (count_velocity == 0)
        ROS_ERROR_STREAM("Khong nhan duoc goi van toc");
    else if (count_velocity < expected_packets_per_sec)
        ROS_WARN_STREAM("Thieu goi van toc: " << loss_v << "%");
    else
        ROS_WARN_STREAM("Van toc du goi (5Hz)");

    if (count_yaw == 0)
        ROS_ERROR_STREAM("Khong nhan duoc goi yaw");
    else if (count_yaw < expected_packets_per_sec)
        ROS_WARN_STREAM("Thieu goi yaw: " << loss_yaw << "%");
    else
        ROS_WARN_STREAM("Yaw du goi (5Hz)");

    count_velocity = 0;
    count_yaw = 0;
}

bool evaluate_checksum(const std::vector<uint8_t>& data_tach) {

    // data_tach phải luôn 17 byte: ID+0+0+0+8_byte_data+8+0+0+0+checksum
    if (data_tach.size() < 17) {
        ROS_WARN("Du lieu data_tach it hon 17 byte");
        return false;
    }

    uint8_t id = data_tach[0];
    uint8_t length = data_tach[12];
    // // ROS_INFO_STREAM("  length = " << hex << static_cast<int>(length) << dec);
    uint8_t received_checksum = data_tach[16];

    // Tính checksum=id+length+data
    uint16_t sum = id + length;
    if (id == 0x01 && length == 0x08) {
        for (int i = 4; i < 12; ++i) {
        sum += data_tach[i];
        // ROS_INFO("Add van toc: data_tach[%d] = 0x%X", i, data_tach[i]);
        }
    }
    else if (id == 0x02 && length == 0x08) {
        for (int i = 4; i < 12; ++i)  {
        sum += data_tach[i];
        // ROS_INFO("Add yaw: data_tach[%d] = 0x%X", i, data_tach[i]);
        }
    }
    else {
        ROS_WARN("ID hoac Length sai. ID = 0x%02X, Length = %d", id, length);
        return false;
    }
    uint8_t computed_checksum = sum & 0xFF;

    // Kiểm tra checksum
    if (computed_checksum != received_checksum) {
        ROS_WARN_STREAM("Checksum sai, Nhan = " << int(received_checksum)   
                         << ", Tinh = " << int(computed_checksum));
        return false;
    }

    return true;  // OK checksum đúng
}
/////////////////////////////////////////////////////////////////////////////////////////////////////////////



// Nhận can frame từ serial. Tách frame để lấy data
// void serialCallback(const string& data, serial::Serial& serialPort, ros::Publisher& pubOdom, ros::Publisher& pubEncode, ros::Publisher pubdata, ros::Publisher pubImu) {
    void serialCallback(const string& data, serial::Serial& serialPort, ros::Publisher& pubOdom, ros::Publisher& pubEncode, 
                    ros::Publisher pubdata, ros::Publisher pubImu, std::ofstream& log_file) {

    // ros::Time now = ros::Time::now();
    // string data_integrity = "OK", sanity_check = "NA", delay_detection = "NA";
    float left_v = 0.0f;
    float right_v = 0.0f;
    // float yaw = 0.0f;
    
    
            vector<uint8_t> frame_byte(data.begin(), data.end()); //vectơ frame truyền 
                // // hiện full frame
                // std::ostringstream oss;
                // oss << "FULL FRAME: ";
                // for (size_t i = 0; i < frame_byte.size(); ++i) {
                // oss << "0x" << std::hex << std::uppercase << std::setw(2) << std::setfill('0') << (int)frame_byte[i] << " ";
                // }
                // ROS_INFO_STREAM(oss.str());
                // //

            //Throughput
            // total_bytes_received += frame_byte.size();  // Tính tổng byte truyền
            //

            uint8_t id = frame_byte[2];                           // id của frame
            vector<uint8_t> data_byte(frame_byte.begin()+2, frame_byte.end() - 2); // lấy ID_0_0_0_8Data_length_0_0_0_Checksum_escape
            vector<uint8_t> data_tach = tach_data_byte(data_byte);                 // lấy ID_0_0_0_8Data_length_0_0_0_Checksum 
            //checksum
            bool valid_checksum = evaluate_checksum(data_tach);
            // data_integrity = valid_checksum ? "OK" : "NG";
            if (!valid_checksum) {
                // logData(log_file, now.toSec(), 0, 0, 0, throughput_bytes_last_sec, 0.0, data_integrity, sanity_check, delay_detection);
                return;
            }
            //

                if(id == 0x01) {
                    
                    //Loss Rate
                    // count_velocity++;  // đếm số gói vận tốc nhận
                    //

                    // // === Phát hiện trễ gói vận tốc ===Delay Detection
                    //  if (!last_vel_time.isZero()) {
                    //     double tre = (now - last_vel_time).toSec() - 0.2;
                    //     delay_detection = (tre > 0.010) ? "NG" : "OK";
                    // if (tre > 0.010)
                    //     ROS_WARN_STREAM("Goi van toc den tre: " << tre << "s");
                    // }
                    // last_vel_time = now;
                    //

                    memcpy(&left_v, &data_tach[4], sizeof(float));
                    memcpy(&right_v, &data_tach[8], sizeof(float));

                    // yaw = imu_data.yaw;

                    // Sanity check
                    // sanity_check = (fabs(left_v) > 60.0f || fabs(right_v) > 60.0f) ? "NG" : "OK";
                    // if (sanity_check == "NG") {
                    //     ROS_WARN("Van toc vuot nguong hop li! L = %.2f, R = %.2f", left_v, right_v);
                    // }
                    // gán dữ liệu vận tốc vào biến 
                    encode.left_twist = left_v;
                    encode.right_twist =right_v;
                    //
                    // ROS_INFO_STREAM("------------------------------------------------------------------------------------------");
                    // ROS_INFO_STREAM(" Data of V:");
                    // ROS_INFO_STREAM("  Receive left_vel = " << std::fixed << std::setprecision(4) << left_v);
                    // ROS_INFO_STREAM("  Receive right_vel = " << std::fixed << std::setprecision(4) << right_v); 
                    // ROS_INFO_STREAM("------------------------------------------------------------------------------------------");
                   

                }

                else if(id == 0x02) {
                    
                    //Loss Rate
                    // count_yaw++;  // đếm gố gói yaw nhận
                    //
                    
                    // === Phát hiện trễ gói yaw ===Delay Detection
                
                    //  if (!last_yaw_time.isZero()) {
                    //     double tre = (now - last_yaw_time).toSec() - 0.2;
                    //     delay_detection = (tre > 0.010) ? "NG" : "OK";
                    //  if (tre > 0.010)
                    //     ROS_WARN_STREAM("Goi yaw den tre: " << tre << "s");
                    //  }
                    //  last_yaw_time = now;
                    //

                    memcpy(&imu_data.yaw,&data_tach[4], sizeof(float));
                    // left_v = encode.left_twist;
                    // right_v = encode.right_twist;
                    // yaw = imu_data.yaw;
                    // Sanity check
                //    sanity_check = (imu_data.yaw < -180.0f || imu_data.yaw > 180.0f) ? "NG" : "OK";
                //      if (sanity_check == "NG") {
                //         ROS_WARN("Yaw vuot nguong hop li! Yaw = %.2f do", imu_data.yaw);
                    //  }
                     // gán dữ liệu yaw vào biến 
                    imu.orientation = get_quaternion_from_RPY(0.0, 0.0 , imu_data.yaw * M_PI / 180.0);
                    imu.header.frame_id = "imu";
                    imu.header.stamp = ros::Time::now();
                    odom.orientation = get_quaternion_from_RPY(0.0, 0.0 , imu_data.yaw * M_PI / 180.0);
                    odom.YPY.x = 0.0;
                    odom.YPY.y = 0.0;
                    odom.YPY.z = imu_data.yaw * M_PI / 180.0;
                    odom.heading = imu_data.yaw * M_PI / 180.0;
                    odom.header.frame_id = "imu";
                    odom.header.stamp = ros::Time::now();  
                    //
                    // ROS_INFO_STREAM(" Data of IMU:");
                    // ROS_INFO_STREAM("  Receive yaw  = " << std::fixed << std::setprecision(4) << imu_data.yaw); 
                    // ROS_INFO_STREAM("------------------------------------------------------------------------------------------");
                    

                }
               // Ghi log
                // logData(log_file, now.toSec(), left_v, right_v, yaw,
                //         throughput_bytes_last_sec, 0.0,
                //         data_integrity, sanity_check, delay_detection);


                // else if(id == 0x06) {
                //     memcpy(&imu_data.ax, &data_tach[4], sizeof(float));
                //     memcpy(&imu_data.ay, &data_tach[8], sizeof(float));

                //     // ROS_INFO_STREAM("  ID = 0x" << hex << static_cast<int>(id) << dec);
                //     // ROS_INFO_STREAM("  Receive ax   = " << std::fixed << std::setprecision(4) << imu_data.ax);
                //     // ROS_INFO_STREAM("  Receive ay   = " << std::fixed << std::setprecision(4) << imu_data.ay);

                //     imu.linear_acceleration.x = imu_data.ax * 9.81;
                //     imu.linear_acceleration.y = imu_data.ay* 9.81;
                //     odom.linear_acceleration.x = imu_data.ax;
                //     odom.linear_acceleration.y = imu_data.ay;
                    
                // }
                // else if(id == 0x07) {
                //     memcpy(&imu_data.az, &data_tach[4], sizeof(float));
                //     memcpy(&imu_data.gx, &data_tach[8], sizeof(float));

                //     // ROS_INFO_STREAM("  ID = 0x" << hex << static_cast<int>(id) << dec);
                //     // ROS_INFO_STREAM("  Receive az   = " << std::fixed << std::setprecision(4) << imu_data.az);
                //     // ROS_INFO_STREAM("  Receive gx   = " << std::fixed << std::setprecision(4) << imu_data.gx);


                //     imu.linear_acceleration.z = imu_data.az* 9.81;
                //     imu.angular_velocity.x = imu_data.gx * M_PI / 180;
                //     odom.linear_acceleration.z = imu_data.az;
                //     odom.angular_velocity.x = imu_data.gx;
                // }
                // else if(id == 0x08) {
                //     memcpy(&imu_data.gy, &data_tach[4], sizeof(float));
                //     memcpy(&imu_data.gz, &data_tach[8], sizeof(float));

                //     // ROS_INFO_STREAM("  ID = 0x" << hex << static_cast<int>(id) << dec);
                //     // ROS_INFO_STREAM("  Receive gy   = " << std::fixed << std::setprecision(4) << imu_data.gy);
                //     // ROS_INFO_STREAM("  Receive gz   = " << std::fixed << std::setprecision(4) << imu_data.gz);

                //     imu.angular_velocity.y = imu_data.gy * M_PI / 180;
                //     imu.angular_velocity.z = imu_data.gz * M_PI / 180;
                    
                //     odom.angular_velocity.y = imu_data.gy;
                //     odom.angular_velocity.z = imu_data.gz;  
                // }
                                     
} 

void updatePub(const ros::TimerEvent&) {
    pubImu.publish(imu);
    pubOdom.publish(odom);
    pubEncode.publish(encode);
}

// Biến toàn cục cho Gửi vận tốc THEO CHU KỲ 200ms
utils::DiffVel current_vel;
std::mutex vel_mutex;


int main(int argc, char** argv) {
    ros::init(argc, argv, "can_node");               //khởi tạo can_node 
    ros::NodeHandle nh;
    ros::NodeHandle private_nh("~");
    private_nh.getParam("enable_can", can_enabled);  // Lấy các tham số từ Parameter Server (được set trong file launch)
    private_nh.getParam("can_port", can_port);
    private_nh.getParam("can_baud", can_baudrate);

    // ros::Timer eval_timer = nh.createTimer(ros::Duration(1.0), evaluate_LossRate);
    // ros::Timer throughput_timer = nh.createTimer(ros::Duration(1.0), evaluate_Throughput);

    // std::ofstream log_file("/home/devbot/mybot_ws/log/can_log.csv", std::ios::out | std::ios::trunc);
    // if (log_file.is_open()) {
    // logHeader(log_file);
    // }


    // Publish lên các topic /odom_2, /odom_test, /diff_motor/real_vel, /imu/data
    pubOdom = nh.advertise<utils::Odometry>("odom_2", 10);
    pubdata = nh.advertise<std_msgs::String>("odom_test", 10);
    pubEncode = nh.advertise<utils::Encoder>("diff_motor/real_vel", 10);
    pubImu = nh.advertise<sensor_msgs::Imu>("imu/data", 10);
    // Publish dữ liệu định kỳ
    loopPub = nh.createTimer(ros::Duration(0.01), updatePub);
    
    serial::Serial serialPort;                      
    try {
        serialPort.setPort(can_port);
        serialPort.setBaudrate(can_baudrate);
        serial::Timeout timeout = serial::Timeout::simpleTimeout(1000);
        serialPort.setTimeout(timeout);
        serialPort.open();              // Mở cổng serial
    } catch (serial::IOException& e) {
        ROS_ERROR_STREAM("Unable to open serial port: " << e.what());
        return -1;
    }
    if (!serialPort.isOpen()) {
        ROS_ERROR_STREAM("Failed to open serial port.");
        return -1;
    }


    //Subscriber từ các topic /diff_motor/vel ; /diff_motor/kp ; /diff_motor/ki ; /diff_motor/kd
    // ========== Gửi vận tốc THEO CHU KỲ 200ms==========
    ros::Subscriber sub_vel = nh.subscribe<utils::DiffVel>("/diff_motor/vel", 1000,
        [&](const utils::DiffVel::ConstPtr& msg) {
            std::lock_guard<std::mutex> lock(vel_mutex);
            current_vel = *msg;
        });
    //
    //  ros::Time last_send_time;
    //
    ros::Timer velSendTimer = nh.createTimer(ros::Duration(0.2),  // 5Hz
        [&](const ros::TimerEvent&) {
            std::lock_guard<std::mutex> lock(vel_mutex);
            vector<uint8_t> frame_vel = createCanFrame(0x01, current_vel.left_vel, current_vel.right_vel);
            serialPort.write(frame_vel.data(), frame_vel.size());

            // Tính thời gian giữa 2 lần gửi
            // ros::Time now = ros::Time::now();
            // double dt = (now - last_send_time).toSec();
            // last_send_time = now;
            //
            // ROS_INFO_STREAM("Sent V : left_vel=" << current_vel.left_vel << ", right_vel=" << current_vel.right_vel);
            // // ROS_INFO_STREAM("Sent V (dt = " << dt << "s): L=" << current_vel.left_vel << ", R=" << current_vel.right_vel);
        });
    //Subscriber Kp1 và Kp2
    ros::Subscriber sub_kp = nh.subscribe<utils::DiffVel>("/diff_motor/kp", 1000, [&](const utils::DiffVel::ConstPtr& msg) { 
        left_vel_PID.Kp = msg->left_vel;
        right_vel_PID.Kp = msg->right_vel;
        vector<uint8_t> frame_kp = createCanFrame(0x03, left_vel_PID.Kp, right_vel_PID.Kp);
        serialPort.write(frame_kp.data(), frame_kp.size());
        // ROS_INFO_STREAM("Sent P: Kp1=" << left_vel_PID.Kp << ", Kp2=" << right_vel_PID.Kp);
    });
    //Subscriber Ki1 và Ki2
    ros::Subscriber sub_ki = nh.subscribe<utils::DiffVel>("/diff_motor/ki", 1000, [&](const utils::DiffVel::ConstPtr& msg) { 
        left_vel_PID.Ki = msg->left_vel;
        right_vel_PID.Ki = msg->right_vel;
        vector<uint8_t> frame_ki = createCanFrame(0x04, left_vel_PID.Ki, right_vel_PID.Ki);
        serialPort.write(frame_ki.data(), frame_ki.size());
        // ROS_INFO_STREAM("Sent I: Ki1=" << left_vel_PID.Ki << ", Ki2=" << right_vel_PID.Ki);
    });
    //Subscriber Kd1 và Kd2
    ros::Subscriber sub_kd = nh.subscribe<utils::DiffVel>("/diff_motor/kd", 1000, [&](const utils::DiffVel::ConstPtr& msg) { 
        left_vel_PID.Kd = msg->left_vel;
        right_vel_PID.Kd = msg->right_vel;
        vector<uint8_t> frame_kd = createCanFrame(0x05, left_vel_PID.Kd, right_vel_PID.Kd);
        serialPort.write(frame_kd.data(), frame_kd.size());
        // ROS_INFO_STREAM("Sent D: Kd1=" << left_vel_PID.Kd << ", Kd2=" << right_vel_PID.Kd);
    });

    // Vòng lặp chính: đọc dữ liệu từ serial và xử lý
    try {
        vector<uint8_t> frame_head; 
        while (ros::ok()) {
            if (serialPort.available()) {
                string finish_byte;
                finish_byte.push_back(0x55);
                finish_byte.push_back(0x55);
                finish_byte.push_back(0xAA);
                finish_byte.push_back(0xAA);

                string raw = serialPort.readline(50, finish_byte);

                // Bỏ 2 byte cuối (0xAA 0xAA)
                if (raw.size() >= 2) raw.erase(raw.end() - 2, raw.end());

                // Thêm frame_head từ lần trước (nếu có) vào đầu raw
                string data;
                for (uint8_t b : frame_head) {
                    data.push_back(static_cast<char>(b));
                }
                data += raw;

                // Ghi nhớ 2 byte cuối để gắn vào frame sau
                frame_head.clear();
                if (raw.size() >= 2) {
                    frame_head.push_back(0xAA);
                    frame_head.push_back(0xAA);
                }

                // Xử lý dữ liệu vừa nhận
                // serialCallback(data, serialPort, pubOdom, pubEncode, pubdata, pubImu);    //Gọi serialCallback() để xử lý dữ liệu 
                serialCallback(data, serialPort, pubOdom, pubEncode, pubdata, pubImu, log_file);
            }
            ros::spinOnce();
        } 
    }       
    catch (serial::IOException& e) {
        ROS_ERROR_STREAM("Error reading from serial port: " << e.what());
    }
    serialPort.close(); // Đóng cổng serial khi kết thúc
    return 0;
}