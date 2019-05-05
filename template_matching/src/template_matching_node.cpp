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
        clahe = cv::createCLAHE();
        pub_result_img = nh.advertise<Image>("/result/image_raw", 100);
        pub_ir_face_bbox = nh.advertise<PointCloud>("/ir_face/bbox", 100);
    }

    void Callback(const ImageConstPtr& rgb_msg, const ImageConstPtr& ir_msg,
                  const PointCloudConstPtr& bbox_msg) {
        cv::Mat rgb_img = cv_bridge::toCvCopy(rgb_msg, "bgr8")->image;
        cv::Mat ir_color_img = cv_bridge::toCvCopy(ir_msg, "bgr8")->image;
        cv::Mat ir_gray_img, gray_img;
        cv::cvtColor(rgb_img, gray_img, CV_BGR2GRAY);
        cv::cvtColor(ir_color_img, ir_gray_img, CV_BGR2GRAY);

        clahe->apply(ir_gray_img, ir_gray_img);

        PointCloud ir_face_bbox_msg;
        ir_face_bbox_msg.header = rgb_msg->header;

        for(int i = 0, n = bbox_msg->points.size() / 2; i < n; ++i) {
            cv::Rect rgb_face_bbox;
            rgb_face_bbox.x = bbox_msg->points[2*i].x;
            rgb_face_bbox.y = bbox_msg->points[2*i].y;
            rgb_face_bbox.width = bbox_msg->points[2*i+1].x;
            rgb_face_bbox.height = bbox_msg->points[2*i+1].y;
            cv::Mat template_img = gray_img(rgb_face_bbox);
            clahe->apply(template_img, template_img);

            cv::Mat image_matched;
            cv::matchTemplate(ir_gray_img, template_img, image_matched, cv::TM_CCOEFF_NORMED);

            // find the best matching local
            double minVal, maxVal;
            cv::Point minLoc, maxLoc;
            cv::minMaxLoc(image_matched, &minVal, &maxVal, &minLoc, &maxLoc);

            cv::Rect match_rect(maxLoc.x, maxLoc.y, rgb_face_bbox.width , rgb_face_bbox.height);

            // debug
            cv::rectangle(ir_color_img, match_rect, color_table[i], 1);
            cv::rectangle(rgb_img, rgb_face_bbox, color_table[i], 1);

            geometry_msgs::Point32 pt;
            pt.x = match_rect.x;
            pt.y = match_rect.y;
            pt.z = 0;
            ir_face_bbox_msg.points.emplace_back(pt);
            pt.x = match_rect.width;
            pt.y = match_rect.height;
            ir_face_bbox_msg.points.emplace_back(pt);
        }

        cv::Mat result;
        cv::hconcat(rgb_img, ir_color_img, result);
        cv_bridge::CvImage result_img_msg;
        result_img_msg.header = rgb_msg->header;
        result_img_msg.encoding = "bgr8";
        result_img_msg.image = result;
        pub_result_img.publish(result_img_msg);
        pub_ir_face_bbox.publish(ir_face_bbox_msg);
    }

    cv::Ptr<cv::CLAHE> clahe;
    ros::Publisher pub_result_img;
    ros::Publisher pub_ir_face_bbox;

    cv::Scalar color_table[10] = {
        cv::Scalar(0, 0, 255),
        cv::Scalar(0, 128, 255),
        cv::Scalar(0, 255, 255),
        cv::Scalar(0, 255, 128),
        cv::Scalar(0, 255, 0),
        cv::Scalar(128, 255, 0),
        cv::Scalar(255, 255, 0),
        cv::Scalar(255, 128, 0),
        cv::Scalar(255, 0, 0),
        cv::Scalar(255, 0, 127),
    };
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "template_matching_node");
    ros::NodeHandle nh("~");
    Node node(nh);

    message_filters::Subscriber<Image> sub_rgb(nh, "/rect_rgb/image_raw", 100),
                                       sub_ir(nh, "/rect_ir/image_raw", 100);
    message_filters::Subscriber<PointCloud> sub_bbox(nh, "/face/bbox", 100);
    TimeSynchronizer<Image, Image, PointCloud> sync(sub_rgb, sub_ir, sub_bbox, 1);
    sync.registerCallback(boost::bind(&Node::Callback, &node, _1, _2, _3));
    ros::spin();
}
