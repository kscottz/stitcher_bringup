// A simple helloword.cpp

#include <ros/ros.h>    // Standard ROS classes

int main(int argc, char **argv) {
    ros::init(argc, argv, "hello_ros");     // Initialize ROS system

    ros::NodeHandle nh;                     // Establish this program as a ROS node
    
    ROS_INFO_STREAM("Hello, ROS!");         // output to log message
}
