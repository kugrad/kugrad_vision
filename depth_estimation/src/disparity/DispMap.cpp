#include "disparity/DispMap.h"
#include "utils.h"

#include <opencv2/calib3d/calib3d.hpp>

using namespace cv;

DispMap::DispMap(const CalibConfig& cam_config, const ReadStereoFS& config_fs)
    :
    cam_l(config_fs.cameraMat_left()),
    cam_r(config_fs.cameraMat_right()),
    dist_coeff_l(config_fs.distCoeff_left()),
    dist_coeff_r(config_fs.distCoeff_right()),
    recti_l(config_fs.rectifyMat_left()),
    recti_r(config_fs.rectifyMat_right()),
    proj_l(config_fs.projectionMat_left()),
    proj_r(config_fs.rectifyMat_right())
{   
    // TODO change hard coding Size() method
    initUndistortRectifyMap(
        cam_l, dist_coeff_l, recti_l, proj_l,
        Size(640, 480),
        CV_32FC1,
        left_stereo_map.first,
        left_stereo_map.second
    );

    initUndistortRectifyMap(
        cam_r, dist_coeff_r, recti_r, proj_r,
        Size(640, 480),
        CV_32FC1,
        right_stereo_map.first,
        right_stereo_map.second
    );
}

DispMap::DispMap(
    const cv::Mat &cameraMat_left,
    const cv::Mat &distCoeff_left,
    const cv::Mat &rectificationMat_left,
    const cv::Mat &projectionMat_left,
    const cv::Mat &cameraMat_right,
    const cv::Mat &distCoeff_right,
    const cv::Mat &rectificationMat_right,
    const cv::Mat &projectionMat_right
) :
    cam_l(cameraMat_left),
    cam_r(cameraMat_right),
    dist_coeff_l(distCoeff_left),
    dist_coeff_r(distCoeff_right),
    recti_l(rectificationMat_left),
    recti_r(rectificationMat_right),
    proj_l(projectionMat_left),
    proj_r(projectionMat_right)
{ 

    initUndistortRectifyMap(
        cam_l, dist_coeff_l, recti_l, proj_l,
        Size(640, 480),
        CV_32FC1,
        left_stereo_map.first,
        left_stereo_map.second
    );

    initUndistortRectifyMap(
        cam_r, dist_coeff_r, recti_r, proj_r,
        Size(640, 480),
        CV_32FC1,
        right_stereo_map.first,
        right_stereo_map.second
    );

}

DispMap::~DispMap() {  }
