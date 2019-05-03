#include <ros/ros.h>
#include <opencv2/opencv.hpp>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/PointCloud.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <cv_bridge/cv_bridge.h>

using namespace message_filters;
using namespace sensor_msgs;

class Node {
public:
    Node(ros::NodeHandle& nh) {
        clahe = cv::createCLAHE(5.0);
    }

    void Callback(const ImageConstPtr& rgb_msg, const ImageConstPtr& ir_msg, const ImageConstPtr& template_msg,
                  const PointCloudConstPtr& bbox_msg) {
        cv::Mat rgb_img = cv_bridge::toCvCopy(rgb_msg, "bgr8")->image;
        cv::Mat ir_color_img = cv_bridge::toCvCopy(ir_msg, "bgr8")->image;
        cv::Mat template_img = cv_bridge::toCvCopy(template_msg, "mono8")->image;
        cv::Rect rgb_face_bbox;
        rgb_face_bbox.x = bbox_msg->points[0].x;
        rgb_face_bbox.y = bbox_msg->points[0].y;
        rgb_face_bbox.width = bbox_msg->points[1].x;
        rgb_face_bbox.height = bbox_msg->points[1].y;
        cv::Mat ir_gray_img;
        cv::cvtColor(ir_color_img, ir_gray_img, CV_BGR2GRAY);
        clahe->apply(template_img, template_img);
        clahe->apply(ir_gray_img, ir_gray_img);

        cv::Mat image_matched;
        cv::matchTemplate(ir_gray_img, template_img, image_matched, cv::TM_CCOEFF_NORMED);
        // find the best matching local
        double minVal, maxVal;
        cv::Point minLoc, maxLoc;
        cv::minMaxLoc(image_matched, &minVal, &maxVal, &minLoc, &maxLoc);

        cv::Rect match_rect(maxLoc.x, maxLoc.y, rgb_face_bbox.width , rgb_face_bbox.height);
        cv::rectangle(ir_color_img, match_rect, cv::Scalar(0, 255, 0), 1);
        cv::rectangle(rgb_img, rgb_face_bbox, cv::Scalar(0, 0, 255), 1);
        cv::imshow("img", rgb_img);
        cv::imshow("ir", ir_color_img);
        cv::waitKey(1);
    }

    cv::Ptr<cv::CLAHE> clahe;
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "template_matching_node");
    ros::NodeHandle nh("~");
    Node node(nh);

    message_filters::Subscriber<Image> sub_rgb(nh, "/rect_rgb/image_raw", 100),
                                       sub_ir(nh, "/rect_ir/image_raw", 100),
                                       sub_template(nh, "/face/image_raw", 100);
    message_filters::Subscriber<PointCloud> sub_bbox(nh, "/face/bbox", 100);
    TimeSynchronizer<Image, Image, Image, PointCloud> sync(sub_rgb, sub_ir, sub_template, sub_bbox, 1);
    sync.registerCallback(boost::bind(&Node::Callback, &node, _1, _2, _3, _4));
    ros::spin();
}
