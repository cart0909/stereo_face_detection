#include <ros/ros.h>
#include <experimental/filesystem>
#include "dataset_loader.h"

using namespace std::experimental;

int main(int argc, char** argv) {
    ros::init(argc, argv, "dataset_publisher_node");
    ros::NodeHandle nh("~");

    filesystem::path root("/datasets/ICLINK_TAIPEI_FACE_DATA");
    ROS_ASSERT(filesystem::is_directory(root));
    filesystem::directory_iterator it_end;
    for(filesystem::directory_iterator it(root); it != it_end; ++it) {
        filesystem::path env_path = it->path();
        for(filesystem::directory_iterator it(env_path); it != it_end; ++it) {
            filesystem::path name_path = it->path();
        }
    }

    ros::spin();
    return 0;
}
