// This program ubcribe turtle1/pose and shows its messages.

#include <ros/ros.h>                // Standard ROS classes
#include <turtlesim/Pose.h>
#include <iomanip>

// Callback function
void poseMessageReceived(const turtlesim::Pose& msg) {
    ROS_INFO_STREAM(std::setprecision(2) << std::fixed
        << "position=(" << msg.x << "," << msg.y << ")"
        << " direction=" << msg.theta);
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "subscribe_to_pose"); // Initialize ROS system
    ros::NodeHandle nh;             // Establish this program as a ROS node

    // Create a subscriber object
    ros::Subscriber sub = nh.subscribe("turtle1/pose", 1000, &poseMessageReceived);

    ros::spin();    // Let ROS take over.
}
