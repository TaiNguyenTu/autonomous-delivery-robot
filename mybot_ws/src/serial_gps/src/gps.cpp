#include <ros/ros.h>
#include <serial/serial.h>
#include <std_msgs/String.h>
#include <QByteArray>


serial::Serial serialPort;


double CalLat2Deg(double Lat)
  {
    static uint8_t Deg = 0;
    static double Min = 0, Result = 0;
    Deg = Lat / 100;
    Min = Lat - Deg * 100;
    Result = Deg + Min / 60;
    return Result;
  }
double CalLong2Deg(double Long)
  {
    static uint16_t Deg = 0;
    static double Min = 0, Result = 0;
    Deg = Long / 100;
    Min = Long - Deg * 100;
    Result = Deg + Min / 60;
    return Result;
  }

void serialCallback(QByteArray& buffer, bool& receivingData, int& bytesToRead) {
    uint8_t data[8] = "$Gdfghj";
    serialPort.write(data, sizeof(data));
    ROS_INFO_STREAM("oke oke");
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "serial_node");
    ros::NodeHandle nh;
    try {
        serialPort.setPort("/dev/ttyUSB0");
        serialPort.setBaudrate(115200);
        serial::Timeout timeout = serial::Timeout::simpleTimeout(1000);
        serialPort.setTimeout(timeout);
        serialPort.open();
    } catch (serial::IOException& e) {
        ROS_ERROR_STREAM("Unable to open serial port: " << e.what());
        return -1;
    }

    if (serialPort.isOpen()) {
        ROS_INFO_STREAM("Serial port opened successfully.");
    } else {
        ROS_ERROR_STREAM("Failed to open serial port.");
        return -1;
    }

    QByteArray buffer;
    bool receivingData = false;
    int bytesToRead = 0;

    ros::Rate loop_rate(10);

    while (ros::ok()) {
        if (serialPort.isOpen()) {
            // uint8_t byte;
            // serialPort.read(&byte, 1);
            serialCallback(buffer, receivingData, bytesToRead);
        }

        ros::spinOnce();
        loop_rate.sleep();
    }

    serialPort.close();
    return 0;
}
