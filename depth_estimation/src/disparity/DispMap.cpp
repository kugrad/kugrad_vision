#include "disparity/DispMap.h"
#include "utils.h"

#include <opencv2/calib3d/calib3d.hpp>

using namespace cv;

DispMap::DispMap(const CamConfig& cam_config, const ReadStereoFS& config_fs)
    :
    left_camera_fd(cam_config.leftCameraFdIdx()),
    right_camera_fd(cam_config.rightCameraFdIdx()),
    camera_width(cam_config.camWidth()),
    camera_height(cam_config.camHeight()),
    camera_fps(cam_config.camFps()),
    cam_l(config_fs.cameraMat_left()),
    cam_r(config_fs.cameraMat_right()),
    dist_coeff_l(config_fs.distCoeff_left()),
    dist_coeff_r(config_fs.distCoeff_right()),
    recti_l(config_fs.rectifyMat_left()),
    recti_r(config_fs.rectifyMat_right()),
    proj_l(config_fs.projectionMat_left()),
    proj_r(config_fs.rectifyMat_right()),
    stream_l(VideoCapture(left_camera_fd)),
    stream_r(VideoCapture(right_camera_fd))
{   
    if (!stream_l.isOpened() || !stream_r.isOpened()) {
        alert::critic_runtime_error("left or right camera stream is not opened!!");
    }

    stream_l.set(CAP_PROP_FRAME_WIDTH, camera_width);
    stream_l.set(CAP_PROP_FRAME_HEIGHT, camera_height);
    stream_l.set(CAP_PROP_FPS, camera_fps);
    stream_r.set(CAP_PROP_FRAME_WIDTH, camera_width);
    stream_r.set(CAP_PROP_FRAME_HEIGHT, camera_height);
    stream_r.set(CAP_PROP_FPS, camera_fps); 

    initUndistortRectifyMap(
        cam_l, dist_coeff_l, recti_l, proj_l,
        Size(camera_width, camera_height),
        CV_32FC1,
        left_stereo_map.first,
        left_stereo_map.second
    );

    initUndistortRectifyMap(
        cam_r, dist_coeff_r, recti_r, proj_r,
        Size(camera_width, camera_height),
        CV_32FC1,
        right_stereo_map.first,
        right_stereo_map.second
    );
}

DispMap::DispMap(
    const std::string &left_camera_fd_idx,
    const std::string &right_camera_fd_idx,
    const uint32_t &camera_width,
    const uint32_t &camera_height,
    const uint8_t &camera_fps,
    const cv::Mat &cameraMat_left,
    const cv::Mat &distCoeff_left,
    const cv::Mat &rectificationMat_left,
    const cv::Mat &projectionMat_left,
    const cv::Mat &cameraMat_right,
    const cv::Mat &distCoeff_right,
    const cv::Mat &rectificationMat_right,
    const cv::Mat &projectionMat_right
) :
    left_camera_fd(left_camera_fd_idx),
    right_camera_fd(right_camera_fd_idx),
    camera_width(camera_width),
    camera_height(camera_height),
    camera_fps(camera_fps),
    cam_l(cameraMat_left),
    cam_r(cameraMat_right),
    dist_coeff_l(distCoeff_left),
    dist_coeff_r(distCoeff_right),
    recti_l(rectificationMat_left),
    recti_r(rectificationMat_right),
    proj_l(projectionMat_left),
    proj_r(projectionMat_right),
    stream_l(VideoCapture(left_camera_fd)),
    stream_r(VideoCapture(right_camera_fd))
{ 
    if (!stream_l.isOpened() || !stream_r.isOpened()) {
        alert::critic_runtime_error("left or right camera stream is not opened!!");
    }

    stream_l.set(CAP_PROP_FRAME_WIDTH, camera_width);
    stream_l.set(CAP_PROP_FRAME_HEIGHT, camera_height);
    stream_l.set(CAP_PROP_FPS, camera_fps);
    stream_r.set(CAP_PROP_FRAME_WIDTH, camera_width);
    stream_r.set(CAP_PROP_FRAME_HEIGHT, camera_height);
    stream_r.set(CAP_PROP_FPS, camera_fps); 


    initUndistortRectifyMap(
        cam_l, dist_coeff_l, recti_l, proj_l,
        Size(camera_width, camera_height),
        CV_32FC1,
        left_stereo_map.first,
        left_stereo_map.second
    );

    initUndistortRectifyMap(
        cam_r, dist_coeff_r, recti_r, proj_r,
        Size(camera_width, camera_height),
        CV_32FC1,
        right_stereo_map.first,
        right_stereo_map.second
    );

}

DispMap::~DispMap() {  }

/**
 * @brief 
 *      read Image from VideoCapture stream 
 * @return std::pair<Mat, Mat>  first  => image from left stream
 *                              second => image from right stream
 */
std::pair<Mat, Mat> DispMap::readImageFromStream() {

    stream_l.read(image_left); 
    stream_r.read(image_right);

    return std::make_pair(image_left, image_right);
}