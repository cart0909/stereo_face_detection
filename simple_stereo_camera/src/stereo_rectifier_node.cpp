#include <memory>
#include <thread>
#include <condition_variable>
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

class Node {
public:
    Node(ros::NodeHandle& nh, const std::string& config_file)
        : stereo_rectifier(config_file, false) {
        pub_rect_rgb = nh.advertise<Image>("/rect_rgb/image_raw", 1000);
        pub_rect_ir = nh.advertise<Image>("/rect_ir/image_raw", 1000);
    }

    void Callback(const ImageConstPtr& rgb_msg, const ImageConstPtr& ir_msg) {
        cv::Mat rgb_img = cv_bridge::toCvCopy(rgb_msg, "bgr8")->image;
        cv::Mat ir_img = cv_bridge::toCvCopy(ir_msg, "bgr8")->image;
        stereo_rectifier.Rectify(rgb_img, ir_img, rgb_img, ir_img);

        cv_bridge::CvImage rect_rgb_img, rect_ir_img;
        rect_rgb_img.header = rgb_msg->header;
        rect_ir_img.header = rgb_msg->header;
        rect_rgb_img.encoding = "bgr8";
        rect_ir_img.encoding = "bgr8";
        rect_rgb_img.image = rgb_img;
        rect_ir_img.image = ir_img;

        pub_rect_rgb.publish(rect_rgb_img.toImageMsg());
        pub_rect_ir.publish(rect_ir_img.toImageMsg());
    }

    StereoRectifier stereo_rectifier;
    ros::Publisher pub_rect_rgb;
    ros::Publisher pub_rect_ir;
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "stereo_rectifier_node", ros::init_options::NoSigintHandler);
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

    Node node(nh, config_file);
    message_filters::Subscriber<Image> rgb_sub(nh, rbg_topic, 10);
    message_filters::Subscriber<Image> ir_sub(nh, ir_topic, 10);

    Synchronizer<sync_policies::ApproximateTime<Image, Image>> sync(sync_policies::ApproximateTime<Image, Image>(10), rgb_sub, ir_sub);
    sync.registerCallback(boost::bind(&Node::Callback, &node, _1, _2));

    ros::spin();
    return 0;
}
