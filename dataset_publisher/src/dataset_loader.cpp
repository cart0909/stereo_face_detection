#include "dataset_loader.h"
#include <ros/ros.h>
using namespace std;

DatasetLoader::DatasetLoader(const std::string& root_path)
    : root(root_path)
{
    ROS_INFO_STREAM("loading dataset...");
    ROS_ASSERT(filesystem::is_directory(root));
    filesystem::directory_iterator it_end;
    for(filesystem::directory_iterator it(root); it != it_end; ++it) {
        if(!filesystem::is_directory(it->path()))
            continue;
        DataPair data_pair;
        data_pair.first = it->path().filename().string();
        map<string, pair<set<string>, set<string>>>& name_folder = data_pair.second;
        for(filesystem::directory_iterator it2(it->path()); it2 != it_end; ++it2) {
            if(!filesystem::is_directory(it2->path()))
                continue;
            string name = it2->path().filename().string();
            pair<set<string>, set<string>> rgb_ir_images;
            filesystem::path rgb_path(it2->path().string() + "/1"), ir_path(it2->path().string() + "/2");
            for(filesystem::directory_iterator rgb_it(rgb_path); rgb_it != it_end; ++rgb_it) {
                string image_name = rgb_it->path().filename().string();
                if(image_name.size() < 4 || string(image_name.end() - 4, image_name.end()) != ".jpg") {
                    continue;
                }
                rgb_ir_images.first.insert(image_name);
            }

            for(filesystem::directory_iterator ir_it(ir_path) ; ir_it != it_end; ++ir_it) {
                string image_name = ir_it->path().filename().string();
                if(image_name.size() < 4 || string(image_name.end() - 4, image_name.end()) != ".jpg") {
                    continue;
                }
                rgb_ir_images.second.insert(image_name);
            }

            name_folder.insert(std::make_pair(name, rgb_ir_images));
        }
        data_struct.insert(data_pair);
    }
    ROS_INFO_STREAM("loading finished!");

    for(auto& it : data_struct) {
        for(auto& it2 : it.second) {
            std::cout << it.first + "/" + it2.first << std::endl;
            std::cout << it2.second.first.size() << " " << it2.second.second.size() << std::endl;
        }
    }
}
