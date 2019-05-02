#include <memory>
#include <opencv2/opencv.hpp>
#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include "stereo_rectifier.h"

using namespace message_filters;
using namespace sensor_msgs;

std::shared_ptr<StereoRectifier> stereo_rectifier;
void callback(const ImageConstPtr& rgb_msg, const ImageConstPtr& ir_msg) {
    cv::Mat rgb_img = cv_bridge::toCvCopy(rgb_msg, "bgr8")->image;
    cv::Mat ir_img = cv_bridge::toCvCopy(ir_msg, "bgr8")->image;
    cv::Mat rgb_rect_img, ir_rect_img;
    stereo_rectifier->Rectify(rgb_img, ir_img, rgb_rect_img, ir_rect_img);
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "stereo_validator_node");
    ros::NodeHandle nh("~");

    std::string config_file, rbg_topic, ir_topic;
    nh.getParam("config_file", config_file);
    cv::FileStorage fs(config_file, cv::FileStorage::READ);
    cv::FileNode rgb_fn, ir_fn;
    rgb_fn = fs["cam0"];
    rgb_fn["rostopic"] >> rbg_topic;

    ir_fn = fs["cam1"];
    ir_fn["rostopic"] >> ir_topic;
    fs.release();

    stereo_rectifier = std::make_shared<StereoRectifier>(config_file, true);

    message_filters::Subscriber<Image> rgb_sub(nh, rbg_topic, 10);
    message_filters::Subscriber<Image> ir_sub(nh, ir_topic, 10);

    Synchronizer<sync_policies::ApproximateTime<Image, Image>> sync(sync_policies::ApproximateTime<Image, Image>(10), rgb_sub, ir_sub);
    sync.registerCallback(boost::bind(&callback, _1, _2));

    ros::spin();
    return 0;
}
