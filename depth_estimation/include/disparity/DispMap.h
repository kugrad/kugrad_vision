#ifndef __DISP_MAP_H__ 
#define __DISP_AMP_H__ 

#include "disparity/ReadStereoFS.h"
#include "cam_calrec/CamConfig.h"

#include <opencv2/core/mat.hpp>
#include <opencv2/videoio.hpp>

class DispMap {
public:
    // init. setting value have to be constant value.
    DispMap(const CamConfig& cam_config, const ReadStereoFS& config_fs);
    DispMap(
        const std::string& left_camera_fd_idx,
        const std::string& right_camera_fd_idx,
        const uint32_t& camera_width,
        const uint32_t& camera_height,
        const uint8_t& camera_fps,
        const cv::Mat& cameraMat_left,
        const cv::Mat& distCoeff_left,
        const cv::Mat& rectificationMat_left,
        const cv::Mat& projectionMat_left,
        const cv::Mat& cameraMat_right,
        const cv::Mat& distCoeff_right,
        const cv::Mat& rectificationMat_right,
        const cv::Mat& projectionMat_right
    );
    ~DispMap();

    std::pair<cv::Mat, cv::Mat> readImageFromStream();



private:
    const std::string left_camera_fd;
    const std::string right_camera_fd;
    const uint32_t camera_width;
    const uint32_t camera_height;
    const uint8_t camera_fps;

    const cv::Mat cam_l;
    const cv::Mat cam_r;
    const cv::Mat dist_coeff_l;
    const cv::Mat dist_coeff_r;
    const cv::Mat recti_l;
    const cv::Mat recti_r;
    const cv::Mat proj_l;
    const cv::Mat proj_r;

    // undistort map for left camera
    std::pair<cv::Mat, cv::Mat> left_stereo_map;
    // undistort map for right camera
    std::pair<cv::Mat, cv::Mat> right_stereo_map;

    cv::VideoCapture stream_l;
    cv::VideoCapture stream_r;

    cv::Mat image_left, image_right;

};

#endif