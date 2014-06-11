#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <dynamic_reconfigure/server.h>
#include <stitcher/StitcherConfig.h>

namespace enc = sensor_msgs::image_encodings;
static const char WINDOW[] = "Image Processed";
image_transport::Publisher pub;
int threshold_value = 0;
int threshold_type = 3;

void imageCallback(const sensor_msgs::ImageConstPtr& original_image) {
    cv_bridge::CvImagePtr cv_ptr;
    try {
        cv_ptr = cv_bridge::toCvCopy(original_image, enc::BGR8);
    } catch (cv_bridge::Exception& e) {
        ROS_ERROR("tutorialROSOpenCV::main.cpp::cv_bridge exception: %s", e.what());
        return;
    }

    threshold(cv_ptr->image, cv_ptr->image, threshold_value, 255, threshold_type);

    cv::imshow(WINDOW, cv_ptr->image);
    cv::waitKey(3);
    pub.publish(cv_ptr->toImageMsg());
}

void configCallBack(stitcher::StitcherConfig &config, uint32_t level) {
    threshold_value = config.threshold_value;
    threshold_type = config.threshold_type;
    
    ROS_INFO("Reconfigure Request: %d %d",
                config.threshold_value,
                config.threshold_type);
}
 
int main(int argc, char **argv) {
    ros::init(argc, argv, "image_processor");
    ros::NodeHandle nh;
    
    dynamic_reconfigure::Server<stitcher::StitcherConfig> server;
    dynamic_reconfigure::Server<stitcher::StitcherConfig>::CallbackType f;
    f = boost::bind(&configCallBack, _1, _2);
    server.setCallback(f);

    image_transport::ImageTransport it(nh);
    cv::namedWindow(WINDOW, CV_WINDOW_AUTOSIZE);
    image_transport::Subscriber sub = it.subscribe("usb_cam/image_raw", 1, imageCallback);
    cv::destroyWindow(WINDOW);
    
    pub = it.advertise("usb_cam/image_processed", 1);
    ros::spin();
    ROS_INFO("tutorialROSOpenCV::main.cpp::No error.");
}
