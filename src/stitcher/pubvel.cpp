// This program publish velocity massages to turtlesim

#include <ros/ros.h>                // Standard ROS classes
#include <geometry_msgs/Twist.h>    // For geometry_msgs::Twist
#include <stdlib.h>                 // For rand() ans RAND_MAX

int main(int argc, char **argv) {
    ros::init(argc, argv, "publish_velocity"); // Initialize ROS system
    ros::NodeHandle nh;             // Establish this program as a ROS node

    // Create a publisher object
    ros::Publisher pub = nh.advertise<geometry_msgs::Twist> ("turtle1/cmd_vel", 1000);

    srand(time(0));

    ros::Rate rate(2);              // Loop at 2Hz
    while(ros::ok()) {
        geometry_msgs::Twist msg;
        msg.linear.x = double(rand()) / double(RAND_MAX);
        msg.angular.z = 2 * double(rand()) / double(RAND_MAX) - 1;

        pub.publish(msg);           // Publish the message

        ROS_INFO_STREAM("Sending random velocity command:"
            << "linear=" << msg.linear.x
            << "angular=" << msg.angular.z);
        
        rate.sleep();
    }

}
