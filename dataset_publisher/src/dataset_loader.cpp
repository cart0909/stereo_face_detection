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
        map<string, pair<set<int>, set<int>>>& name_folder = data_pair.second;
        for(filesystem::directory_iterator it2(it->path()); it2 != it_end; ++it2) {
            if(!filesystem::is_directory(it2->path()))
                continue;
            string name = it2->path().filename().string();
            pair<set<int>, set<int>> rgb_ir_images;
            filesystem::path rgb_path(it2->path().string() + "/1"), ir_path(it2->path().string() + "/2");
            for(filesystem::directory_iterator rgb_it(rgb_path); rgb_it != it_end; ++rgb_it) {
                string image_name = rgb_it->path().filename().string();
                if(image_name.size() < 4 || string(image_name.end() - 4, image_name.end()) != ".jpg") {
                    continue;
                }
                // remove frame_
                image_name.erase(image_name.begin(), image_name.begin() + 6);
                // remove .jpg
                image_name.erase(image_name.end() - 4, image_name.end());
                rgb_ir_images.first.insert(std::stoi(image_name));
            }

            for(filesystem::directory_iterator ir_it(ir_path) ; ir_it != it_end; ++ir_it) {
                string image_name = ir_it->path().filename().string();
                if(image_name.size() < 4 || string(image_name.end() - 4, image_name.end()) != ".jpg") {
                    continue;
                }
                // remove frame_
                image_name.erase(image_name.begin(), image_name.begin() + 6);
                // remove .jpg
                image_name.erase(image_name.end() - 4, image_name.end());
                rgb_ir_images.second.insert(std::stoi(image_name));
            }

            name_folder.insert(std::make_pair(name, rgb_ir_images));
        }
        data_struct.insert(data_pair);
    }

    for(auto& it : data_struct) {
        for(auto& it2 : it.second) {
            auto rgb_it = it2.second.first.begin(), rgb_end = it2.second.first.end();
            auto ir_it = it2.second.second.begin(), ir_end = it2.second.second.end();
            while(!(rgb_it == rgb_end || ir_it == ir_end)) {
                if(*rgb_it == *ir_it) {
                    std::string rgb_filename = root.string() + "/" + it.first + "/" + it2.first + "/1/frame_" + std::to_string(*rgb_it) + ".jpg";
                    std::string ir_filename = root.string() + "/" + it.first + "/" + it2.first + "/2/frame_" + std::to_string(*ir_it) + ".jpg";
                    v_rgb_ir_filename.emplace_back(rgb_filename, ir_filename);
                    ++rgb_it;
                    ++ir_it;
                }
                else if(*rgb_it > *ir_it) {
                    ++ir_it;
                }
                else if(*ir_it > *rgb_it) {
                    ++rgb_it;
                }
            }
        }
    }

    ROS_INFO_STREAM("loading finished!");
    ROS_INFO_STREAM("total image pair " << v_rgb_ir_filename.size());
}
