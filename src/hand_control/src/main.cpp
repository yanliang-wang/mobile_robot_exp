//
// Created by jason on 4/11/18.
//
#include "SerialClass.hpp"
SerialClass *serClass;
int main(int argc,char *argv[])
{
    ros::init(argc,argv,"hand_control");
    ros::NodeHandle nh;
    ros::Subscriber cmd = nh.subscribe<hand_control::hand_control_cmd>("hand_control_cmd",1000,SerialClass::callback);
    string port = "/dev/ttyUSB0";
    int baudrate = 115200;
    if(ros::param::has("~port_name")){
        ros::param::get("~port_name",port);
    }
    if(ros::param::has("~baudrate")){
        ros::param::get("~baudrate",baudrate);
    }
    ROS_INFO_STREAM(port);
    serClass = new SerialClass(port,baudrate);

    serClass->run();
    return 0;
}
