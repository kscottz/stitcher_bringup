#include <unistd.h>
#include <vector>
#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/stitching/stitcher.hpp>
#include <dynamic_reconfigure/server.h>
#include <stitcher/StitcherConfig.h>

namespace enc = sensor_msgs::image_encodings;
static const char WINDOW[] = "Image Processed";
image_transport::Publisher pub;
std::vector <cv::Mat> vImg;

int stitcher_num_frames = 2;
double registr_resol = 0.6;
double seam_est_resol = 0.1;
double compose_resol = -1;
double conf_thresh = 1;
bool do_wave_correct = true;

void stitcherCallback(const sensor_msgs::ImageConstPtr& original_image) {
    cv_bridge::CvImagePtr cv_ptr;
    try {
        cv_ptr = cv_bridge::toCvCopy(original_image, enc::BGR8);
    } catch (cv_bridge::Exception& e) {
        ROS_ERROR("tutorialROSOpenCV::main.cpp::cv_bridge exception: %s", e.what());
        return;
    }

    if (vImg.size() == stitcher_num_frames - 1) { 
        vImg.push_back(cv_ptr->image);
        
        cv::Stitcher stitcher = cv::Stitcher::createDefault();
        stitcher.stitch(vImg, cv_ptr->image);
        
        cv::imwrite("images/Panorama.jpg", cv_ptr->image);
        cv::imshow(WINDOW, cv_ptr->image);
        cv::waitKey(3);
        pub.publish(cv_ptr->toImageMsg());
        sleep(2);
        vImg.clear();
    } else {
        vImg.push_back(cv_ptr->image);
    }
}

void configCallBack(stitcher::StitcherConfig &config, uint32_t level) {
    stitcher_num_frames = config.stitcher_num_frames;
    registr_resol = config.registr_resol;
    seam_est_resol = config.seam_est_resol;
    compose_resol = config.compose_resol;
    conf_thresh = config.conf_thresh;
    do_wave_correct = config.do_wave_correct;

    ROS_INFO("Reconfigure Request: %d %f %f %f %f %d",
                stitcher_num_frames,
                registr_resol,
                seam_est_resol,
                compose_resol,
                conf_thresh,
                do_wave_correct);
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
    image_transport::Subscriber sub = it.subscribe("usb_cam/image_raw", 1, stitcherCallback);
    cv::destroyWindow(WINDOW);
    
    pub = it.advertise("usb_cam/image_processed", 1);
    
    while (ros::ok()) {
        ros::spinOnce();
    }
    
    ROS_INFO("tutorialROSOpenCV::main.cpp::No error.");
}
