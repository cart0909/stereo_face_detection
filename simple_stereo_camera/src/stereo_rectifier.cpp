#include "stereo_rectifier.h"

StereoRectifier::StereoRectifier(const std::string& config_file, bool debug_show_)
    : debug_show(debug_show_)
{
    cv::FileStorage fs(config_file, cv::FileStorage::READ);
    cv::FileNode rgb_fn, ir_fn;
    std::vector<double> intrinsics, distortions;
    rgb_fn = fs["cam0"];
    rgb_fn["intrinsics"] >> intrinsics;
    rgb_fn["distortion_coeffs"] >> distortions;
    K0 = (cv::Mat_<double>(3, 3) << intrinsics[0], 0, intrinsics[2],
                                    0, intrinsics[1], intrinsics[3],
                                    0, 0, 1);
    D0 = (cv::Mat_<double>(4, 1) << distortions[0], distortions[1], distortions[2], distortions[3]);

    ir_fn = fs["cam1"];
    ir_fn["intrinsics"] >> intrinsics;
    ir_fn["distortion_coeffs"] >> distortions;
    K1 = (cv::Mat_<double>(3, 3) << intrinsics[0], 0, intrinsics[2],
                                    0, intrinsics[1], intrinsics[3],
                                    0, 0, 1);
    D1 = (cv::Mat_<double>(4, 1) << distortions[0], distortions[1], distortions[2], distortions[3]);
    fs["T10"] >> T10;
    image_size.width = fs["width"];
    image_size.height = fs["height"];
    fs.release();

    cv::Mat R10, t10;
    T10.colRange(0, 3).rowRange(0, 3).copyTo(R10);
    T10.col(3).rowRange(0, 3).copyTo(t10);
    cv::Mat R1, R2, P1, P2;
    cv::stereoRectify(K0, D0, K1, D1, image_size, R10, t10, R1, R2, P1, P2, cv::noArray(), cv::CALIB_ZERO_DISPARITY, 0);
    cv::initUndistortRectifyMap(K0, D0, R1, P1, image_size, CV_32FC1, M1rgb, M2rgb);
    cv::initUndistortRectifyMap(K1, D1, R2, P2, image_size, CV_32FC1, M1ir, M2ir);
}

void StereoRectifier::Rectify(const cv::Mat& rgb_img, const cv::Mat& ir_img,
                              cv::Mat& rgb_rect_img, cv::Mat& ir_rect_img) const {
    cv::remap(rgb_img, rgb_rect_img, M1rgb, M2rgb, cv::INTER_LINEAR);
    cv::remap(ir_img, ir_rect_img, M1ir, M2ir, cv::INTER_LINEAR);

    if(debug_show) {
        cv::Mat result;
        cv::vconcat(ir_rect_img, rgb_rect_img, result);

        for(int i = 1; i < 20; ++i) {
            cv::line(result, cv::Point(32*i, 0), cv::Point(32*i, result.rows), cv::Scalar::all(255), 1);
        }
        cv::imshow("result", result);
        cv::waitKey(1);
    }
}
