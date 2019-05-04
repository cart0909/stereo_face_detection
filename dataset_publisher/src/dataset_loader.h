#pragma once
#include <string>
#include <experimental/filesystem>
#include <set>
#include <map>
using namespace std::experimental;

class DatasetLoader {
public:
    DatasetLoader(const std::string& root_path);

    filesystem::path root;
    // backlight_lester carl_close
    using DataPair = std::pair<std::string, std::map<std::string, std::pair<std::set<int>, std::set<int>>>>;
    using DataStruct = std::map<std::string, std::map<std::string, std::pair<std::set<int>, std::set<int>>>>;
    DataStruct data_struct;
    std::vector<std::pair<std::string, std::string>> v_rgb_ir_filename;
};
