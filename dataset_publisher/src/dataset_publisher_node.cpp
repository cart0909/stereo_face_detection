#include <ros/ros.h>
#include <memory>
#include <experimental/filesystem>
#include "dataset_loader.h"

class Node {
public:
    Node(ros::NodeHandle& nh) {
        loader = std::make_shared<DatasetLoader>("/datasets/ICLINK_TAIPEI_FACE_DATA");
    }

    std::shared_ptr<DatasetLoader> loader;
};


int main(int argc, char** argv) {
    ros::init(argc, argv, "dataset_publisher_node");
    ros::NodeHandle nh("~");
    Node node(nh);


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
