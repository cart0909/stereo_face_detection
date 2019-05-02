#pragma once
#include <string>
#include <opencv2/opencv.hpp>

class StereoRectifier {
public:
    StereoRectifier(const std::string& config_file, bool debug_show_ = false);

    void Rectify(const cv::Mat& rgb_img, const cv::Mat& ir_img,
                 cv::Mat& rgb_rect_img, cv::Mat& ir_rect_img) const;

    int GetWidth() const {
        return image_size.width;
    }

    int GetHeight() const {
        return image_size.height;
    }

private:
    bool debug_show;
    cv::Mat M1rgb, M2rgb, M1ir, M2ir;
    cv::Mat K0, K1, D0, D1;
    cv::Mat T10, R10, t10;
    cv::Size image_size;
};
