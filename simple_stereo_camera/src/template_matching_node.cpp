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
    Node(const std::string& config_file)
        : stereo_rectifier(config_file, false) {
        process_thread = std::thread(&Node::Process, this);
        clahe = cv::createCLAHE(3.0);
        width = stereo_rectifier.GetWidth();
        height = stereo_rectifier.GetHeight();
    }

    void Callback(const ImageConstPtr& rgb_msg, const ImageConstPtr& ir_msg) {
        cv::Mat rgb_img = cv_bridge::toCvCopy(rgb_msg, "bgr8")->image;
        cv::Mat ir_img = cv_bridge::toCvCopy(ir_msg, "bgr8")->image;
        buffer_mutex.lock();
        rgb_img_buffer.emplace_back(rgb_img);
        ir_img_buffer.emplace_back(ir_img);
        buffer_mutex.unlock();
        buffer_cv.notify_one();
    }

    void Process() {
        while(1) {
            cv::Mat rgb_img, gray_img, ir_img, ir_gray_img;
            std::unique_lock<std::mutex> lock(buffer_mutex);
            buffer_cv.wait(lock, [&] {
                if(!rgb_img_buffer.empty()) {
                    rgb_img = rgb_img_buffer.back();
                    ir_img = ir_img_buffer.back();
                    rgb_img_buffer.clear();
                    ir_img_buffer.clear();
                }
                return !rgb_img.empty();
            });

            stereo_rectifier.Rectify(rgb_img, ir_img, rgb_img, ir_img);
            cv::cvtColor(rgb_img, gray_img, CV_BGR2GRAY);
            cv::cvtColor(ir_img, ir_gray_img, CV_BGR2GRAY);
            clahe->apply(gray_img, gray_img);
            clahe->apply(ir_gray_img, ir_gray_img);

            cv::Rect rect(width/2 - 64, height/2 - 128, 64, 128);
            cv::Mat template_img = gray_img(rect).clone(), image_matched;
            cv::matchTemplate(ir_gray_img, template_img, image_matched, cv::TM_CCOEFF_NORMED);

            // find the best matching local
            double minVal, maxVal;
            cv::Point minLoc, maxLoc;
            cv::minMaxLoc(image_matched, &minVal, &maxVal, &minLoc, &maxLoc);

            cv::Rect match_rect(maxLoc.x, maxLoc.y, 64, 128);
            cv::rectangle(ir_img, match_rect, cv::Scalar(0, 255, 0), 1);
            cv::rectangle(rgb_img, rect, cv::Scalar(0, 0, 255), 1);
            cv::imshow("img", rgb_img);
            cv::imshow("ir", ir_img);
            cv::waitKey(1);
        }
    }

    int width, height;
    StereoRectifier stereo_rectifier;
    std::thread process_thread;
    std::condition_variable buffer_cv;
    std::mutex buffer_mutex;
    std::vector<cv::Mat> rgb_img_buffer;
    std::vector<cv::Mat> ir_img_buffer;
    cv::Ptr<cv::CLAHE> clahe;
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "template_matching_node", ros::init_options::NoSigintHandler);
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

    Node node(config_file);
    message_filters::Subscriber<Image> rgb_sub(nh, rbg_topic, 10);
    message_filters::Subscriber<Image> ir_sub(nh, ir_topic, 10);

    Synchronizer<sync_policies::ApproximateTime<Image, Image>> sync(sync_policies::ApproximateTime<Image, Image>(10), rgb_sub, ir_sub);
    sync.registerCallback(boost::bind(&Node::Callback, &node, _1, _2));

    ros::spin();
    return 0;
}
