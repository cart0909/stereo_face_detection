#include <memory>
#include <thread>
#include <chrono>
#include <opencv2/opencv.hpp>
#include <experimental/filesystem>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/Image.h>
#include <std_msgs/Empty.h>
#include <cv_bridge/cv_bridge.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include "dataset_loader.h"

using namespace sensor_msgs;
using namespace message_filters;

class Node {
public:
    Node(ros::NodeHandle& nh) {
        loader = std::make_shared<DatasetLoader>("/datasets/ICLINK_TAIPEI_FACE_DATA");
        pub_rgb = nh.advertise<Image>("/cam1/image_raw", 100);
        pub_ir = nh.advertise<Image>("/cam2/image_raw", 100);
    }

    void PubFirstImages() {
        PubNextImage();
    }

    void Callback(const ImageConstPtr& rgb_msg, const ImageConstPtr& ir_msg,
                  const PointCloudConstPtr& rgb_bbox_msg, const PointCloudConstPtr& ir_bbox_msg) {
        // save last result
        // TODO
        // pub next
        PubNextImage();
    }

    void NoFaceCallback(const std_msgs::EmptyConstPtr& noface_msg) {
        PubNextImage();
    }

private:
    void PubNextImage() {
        cv::Mat rgb_image = cv::imread(loader->v_rgb_ir_filename[index].first, cv::IMREAD_COLOR);
        cv::Mat ir_image = cv::imread(loader->v_rgb_ir_filename[index].second, cv::IMREAD_COLOR);
        cv_bridge::CvImage rgb_msg, ir_msg;
        std_msgs::Header header;
        header.seq = index;
        header.stamp = ros::Time::now();
        rgb_msg.header = header;
        ir_msg.header = header;
        rgb_msg.encoding = "bgr8";
        ir_msg.encoding = "bgr8";
        rgb_msg.image = rgb_image;
        ir_msg.image = ir_image;

        pub_rgb.publish(rgb_msg.toImageMsg());
        pub_ir.publish(ir_msg.toImageMsg());
        ++index;
    }
    std::shared_ptr<DatasetLoader> loader;
    int index = 0;
    ros::Publisher pub_rgb, pub_ir;
};


int main(int argc, char** argv) {
    ros::init(argc, argv, "dataset_publisher_node");
    ros::NodeHandle nh("~");
    Node node(nh);

    message_filters::Subscriber<Image> sub_rect_rgb(nh, "/rect_rgb/image_raw", 100),
                                       sub_rect_ir(nh, "/rect_ir/image_raw", 100);
    message_filters::Subscriber<PointCloud> sub_rgb_bbox(nh, "/face/bbox", 100),
                                            sub_ir_bbox(nh, "/ir_face/bbox", 100);
    TimeSynchronizer<Image, Image, PointCloud, PointCloud> sync(sub_rect_rgb, sub_rect_ir, sub_rgb_bbox, sub_ir_bbox, 100);
    sync.registerCallback(boost::bind(&Node::Callback, &node, _1, _2, _3, _4));
    ros::Subscriber sub_no_face = nh.subscribe<std_msgs::Empty>("/noface", 100, boost::bind(&Node::NoFaceCallback, &node, _1));

    std::this_thread::sleep_for(std::chrono::seconds(1));
    node.PubFirstImages();

    ros::spin();
    return 0;
}
