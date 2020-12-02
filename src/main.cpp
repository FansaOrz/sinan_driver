/**
 *  @example serial_stream_read.cpp
 */

#include <libserial/SerialStream.h>
#include <libserial/SerialPort.h>
#include <sensor_msgs/NavSatFix.h>
#include <ros/ros.h>
#include <string>
#include <cstdlib>
#include <eigen3/Eigen/Core>
//#include <GeographicLib/LambertConformalConic.hpp>
#include <iostream>
#include <unistd.h>
#include <LocalCartesian.hpp>

//constexpr const char* const SERIAL_PORT_1 = "/dev/ttyUSB0" ;
using namespace std;
using namespace LibSerial;
using namespace GeographicLib;
/**
 * @brief This example demonstrates configuring a serial stream and
 *        reading serial stream data.
 */

ros::Publisher GPS_pub;
sensor_msgs::NavSatFix GPS_msg = sensor_msgs::NavSatFix();
bool if_first = true;
bool got_init_pose = false;

Eigen::Vector3d init_lla;
Eigen::Vector3d current_point_lla;
Eigen::Vector3d ENU_point;

void ConvertLLA(const Eigen::Vector3d& init_lla,
                const Eigen::Vector3d& point_lla,
                Eigen::Vector3d* point){
    static LocalCartesian local_cartesian;
    local_cartesian.Reset(init_lla(0), init_lla(1), init_lla(2));
    local_cartesian.Forward(point_lla(0), point_lla(1), point_lla(2),
                            point->data()[0], point->data()[1], point->data()[2]);

}

void Split(const std::string& src, const std::string& separator, std::vector<std::string>& dest) //字符串分割到数组
{

    //参数1：要分割的字符串；参数2：作为分隔符的字符；参数3：存放分割后的字符串的vector向量

    string str = src;
    string substring;
    string::size_type start = 0, index;
    dest.clear();
    index = str.find_first_of(separator,start);
    do
    {
        if (index != string::npos)
        {
            substring = str.substr(start,index-start );
            dest.push_back(substring);
            start =index+separator.size();
            index = str.find(separator,start);
            if (start == string::npos) break;
        }
    }while(index != string::npos);

    //the last part
    substring = str.substr(start);
    dest.push_back(substring);
}

double convert_LLA(double origin){
    double result = origin / 100;
    int integer = int(result);
    double decimals = result - integer;
    result = integer + decimals * 5 / 3;
    return result;
}

void publish_data(string data){
    vector<std::string> ster;
    vector<std::string>::iterator iter; //声明一个迭代器
    Split(data, ",", ster);
    string Insta;
    cout << data << endl;

    double latitude_GPS = 0;
    double longitude_GPS = 0;
    double altitude_GPS = 0;
    double status = 0;
    double service = 0;
    try {
        if(ster[2] == ""){
            return;
        }
        // 经纬度和海拔
        latitude_GPS = stod(ster[2]);
        longitude_GPS = stod(ster[4]);
        altitude_GPS = stod(ster[9]);
        // 当前解状态     0:无效解;  1:单点定位解;  2:伪距差分;  4:固定解;  5:浮动解;  6:组合导航解;  7:固定坐标;  9:WAAS 差分;
        status = stod(ster[6]);
        // 当前卫星数量
        service = stod(ster[7]);
    } catch (int e) {
        ROS_WARN("port data is invalid");
        latitude_GPS = 0;
        longitude_GPS = 0;
        altitude_GPS = 0;
        status = -1;
        service = -1;
    }
    GPS_msg.altitude = convert_LLA(altitude_GPS);
    GPS_msg.longitude = convert_LLA(longitude_GPS);
    GPS_msg.status.status = status;
    GPS_msg.status.service = service;
    GPS_msg.latitude = latitude_GPS;
    GPS_msg.header.frame_id = "GPS";
    GPS_msg.header.stamp = ros::Time::now();
    GPS_pub.publish(GPS_msg);
    if(!got_init_pose){
        current_point_lla << GPS_msg.longitude, GPS_msg.latitude, GPS_msg.altitude;
        init_lla << GPS_msg.longitude, GPS_msg.latitude, GPS_msg.altitude;
    } else {
        current_point_lla << GPS_msg.longitude, GPS_msg.latitude, GPS_msg.altitude;
    }
    ConvertLLA(init_lla, current_point_lla, &ENU_point);
//    cout << "XYZ:::\nX===: " << ENU_point(0) << "\nY===: " << ENU_point(1) << "\nZ===: " << ENU_point(2) << endl;
}

int main(int argc, char** argv)
{

    ros::init(argc, argv, "serial_port");
    //创建句柄（虽然后面没用到这个句柄，但如果不创建，运行时进程会出错）
    ros::NodeHandle n;
    string device_port = "/dev/ttyUSB0";
    GPS_pub = n.advertise<sensor_msgs::NavSatFix>("/sinan_GPS", 5);
    ros::param::get("~port", device_port);
    // Instantiate a SerialStream object.
    SerialStream serial_stream ;
    try
    {
        // Open the Serial Port at the desired hardware port.
        serial_stream.Open(device_port) ;
        ROS_INFO("\033[1;32m---->\033[0m start reading the .");
    }
    catch (const OpenFailed&)
    {
        std::cerr << "The serial port did not open correctly." << std::endl ;
        return EXIT_FAILURE ;
    }
    // Set the baud rate of the serial port.
    serial_stream.SetBaudRate(BaudRate::BAUD_115200) ;
    // Set the number of data bits.
    serial_stream.SetCharacterSize(CharacterSize::CHAR_SIZE_8) ;
    // Turn off hardware flow control.
    serial_stream.SetFlowControl(FlowControl::FLOW_CONTROL_NONE) ;
    // Disable parity.
    serial_stream.SetParity(Parity::PARITY_NONE) ;
    // Set the number of stop bits.
    serial_stream.SetStopBits(StopBits::STOP_BITS_1) ;
    // Wait for data to be available at the serial port.
    while(serial_stream.rdbuf()->in_avail() == 0)
    {
        usleep(1000) ;
    }

    // Keep reading data from serial port and print it to the screen.
    string data = "";
    while(ros::ok())
    {
        // Variable to store data coming from the serial port.
        char data_byte ;

        // Read a single byte of data from the serial port.
        serial_stream.get(data_byte) ;

        // Show the user what is being read from the serial port.
        if (data_byte == '\n') {
            if (if_first) {
                if_first = false;
                data = "";
            } else {
                publish_data(data);
                data = "";
            }
        }
        data.append(1, data_byte);
        // Wait a brief period for more data to arrive.
        usleep(100) ;

    }

    // Successful program completion.
    std::cout << "Don" << std::endl ;
    return EXIT_SUCCESS ;
}
