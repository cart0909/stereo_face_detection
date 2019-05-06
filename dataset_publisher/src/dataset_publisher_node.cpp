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
#include <jsoncpp/json/json.h>
#include "dataset_loader.h"

using namespace sensor_msgs;
using namespace message_filters;
namespace fs = std::experimental::filesystem;

class Node {
public:
    Node(ros::NodeHandle& nh) {
        loader = std::make_shared<DatasetLoader>("/datasets/ICLINK_TAIPEI_FACE_DATA");
        pub_rgb = nh.advertise<Image>("/cam1/image_raw", 100);
        pub_ir = nh.advertise<Image>("/cam2/image_raw", 100);

        output_root_string = loader->root.string() + "_output";
        fs::remove_all(output_root_string);
        fs::create_directory(output_root_string);
    }

    void PubFirstImages() {
        PubNextImage();
    }

    void Callback(const ImageConstPtr& rgb_msg, const ImageConstPtr& ir_msg,
                  const PointCloudConstPtr& rgb_bbox_msg, const PointCloudConstPtr& ir_bbox_msg) {
        // save last result
        std::string src_rgb_filename = loader->v_rgb_ir_filename[index - 1].first;
        std::string src_rgb_folder(src_rgb_filename.begin(), src_rgb_filename.begin() + src_rgb_filename.rfind('/'));
        std::string dst_rgb_folder = output_root_string + std::string(src_rgb_folder.begin() + loader->root.string().size(), src_rgb_folder.end());
        std::string src_ir_filename = loader->v_rgb_ir_filename[index - 1].second;
        std::string src_ir_folder(src_ir_filename.begin(), src_ir_filename.begin() + src_ir_filename.rfind('/'));
        std::string dst_ir_folder = output_root_string + std::string(src_ir_folder.begin() + loader->root.string().size(), src_ir_folder.end());

        if(!fs::exists(dst_rgb_folder)) {
            fs::create_directories(dst_rgb_folder);
            fs::create_directories(dst_ir_folder);
            dst_folder_frame_count = 0;
        }

        cv::Mat rgb_img = cv_bridge::toCvCopy(rgb_msg, "bgr8")->image;
        cv::imwrite(dst_rgb_folder + "/" + std::to_string(dst_folder_frame_count) + ".jpg", rgb_img);
        Json::Value rgb_bbox_root;
        {
            for(int i = 0, n = rgb_bbox_msg->points.size()/2; i < n; ++i) {
                Json::Value array;
                Json::Value item;
                int x = rgb_bbox_msg->points[2*i].x;
                int y = rgb_bbox_msg->points[2*i].y;
                int w = rgb_bbox_msg->points[2*i+1].x;
                int h = rgb_bbox_msg->points[2*i+1].y;
                item.append(x);
                item.append(y);
                item.append(w);
                item.append(h);
                array["bbox"] = item;
                rgb_bbox_root.append(array);
            }
        }
        std::ofstream rgb_bbox_out(dst_rgb_folder + "/" + std::to_string(dst_folder_frame_count) + ".json");
        rgb_bbox_out << rgb_bbox_root.toStyledString();
        rgb_bbox_out.close();

        cv::Mat ir_img = cv_bridge::toCvCopy(ir_msg, "bgr8")->image;
        cv::imwrite(dst_ir_folder + "/" + std::to_string(dst_folder_frame_count) + ".jpg", ir_img);
        Json::Value ir_bbox_root;
        {
            for(int i = 0, n = ir_bbox_msg->points.size()/2; i < n; ++i) {
                Json::Value array;
                Json::Value item;
                int x = ir_bbox_msg->points[2*i].x;
                int y = ir_bbox_msg->points[2*i].y;
                int w = ir_bbox_msg->points[2*i+1].x;
                int h = ir_bbox_msg->points[2*i+1].y;
                item.append(x);
                item.append(y);
                item.append(w);
                item.append(h);
                array["bbox"] = item;
                ir_bbox_root.append(array);
            }
        }
        std::ofstream ir_bbox_out(dst_ir_folder + "/" + std::to_string(dst_folder_frame_count) + ".json");
        ir_bbox_out << ir_bbox_root.toStyledString();
        ir_bbox_out.close();

        ++dst_folder_frame_count;
        // pub next
        PubNextImage();
    }

    void NoFaceCallback(const std_msgs::EmptyConstPtr& noface_msg) {
        PubNextImage();
    }

private:
    void PubNextImage() {
        if(index == loader->v_rgb_ir_filename.size()) {
            ROS_INFO_STREAM("Process finished!");
            return;
        }
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

        if(index % 1000 == 0) {
            ROS_INFO_STREAM("Process " << index << " / " << loader->v_rgb_ir_filename.size());
        }
    }
    std::shared_ptr<DatasetLoader> loader;
    int index = 0;
    ros::Publisher pub_rgb, pub_ir;
    std::string output_root_string;
    int dst_folder_frame_count = 0;
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
