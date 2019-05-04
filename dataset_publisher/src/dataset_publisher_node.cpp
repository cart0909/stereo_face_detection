#include <memory>
#include <thread>
#include <chrono>
#include <opencv2/opencv.hpp>
#include <experimental/filesystem>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>
#include "dataset_loader.h"

using namespace sensor_msgs;

class Node {
public:
    Node(ros::NodeHandle& nh) {
        loader = std::make_shared<DatasetLoader>("/datasets/ICLINK_TAIPEI_FACE_DATA");
        pub_rgb = nh.advertise<Image>("/cam1/image_raw", 100);
        pub_ir = nh.advertise<Image>("/cam2/image_raw", 100);
    }

    void PubFirstImages() {
        cv::Mat rgb_image = cv::imread(loader->v_rgb_ir_filename[0].first, cv::IMREAD_COLOR);
        cv::Mat ir_image = cv::imread(loader->v_rgb_ir_filename[0].second, cv::IMREAD_COLOR);
        cv_bridge::CvImage rgb_msg, ir_msg;
        std_msgs::Header header;
        header.seq = 0;
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

    void Callback(const PointCloudConstPtr& ir_bbox_msg) {
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

    ros::Subscriber sub_ir_bbox = nh.subscribe<PointCloud>("/ir_face/bbox", 100, boost::bind(&Node::Callback, &node, _1));

    std::this_thread::sleep_for(std::chrono::seconds(1));
    node.PubFirstImages();

//    filesystem::path root("/datasets/ICLINK_TAIPEI_FACE_DATA");
//    ROS_ASSERT(filesystem::is_directory(root));
//    filesystem::directory_iterator it_end;
//    for(filesystem::directory_iterator it(root); it != it_end; ++it) {
//        filesystem::path env_path = it->path();
//        for(filesystem::directory_iterator it(env_path); it != it_end; ++it) {
//            filesystem::path name_path = it->path();
//        }
//    }

    ros::spin();
    return 0;
}
