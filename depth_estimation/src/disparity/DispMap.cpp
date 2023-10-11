#include "disparity/DispMap.h"
#include "utils.h"

#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/imgproc/imgproc.hpp>

using namespace cv;

DispMap::DispMap(ReadStereoFS* config_fs)
    :
DispMap(
    config_fs->cameraMat_left(),
    config_fs->distCoeff_left(),
    config_fs->rectifyMat_left(),
    config_fs->projectionMat_left(),
    config_fs->cameraMat_right(),
    config_fs->distCoeff_right(),
    config_fs->rectifyMat_right(),
    config_fs->projectionMat_right()
) {  }

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
    proj_r(projectionMat_right),
#if STEREO_SGBM
    stereo(StereoSGBM::create(
        min_disparity,
        num_disparity, 
        window_block_size,
        P1,
        P2,
        disp12MaxDiff,
        preFilterCap,
        uniquenessRatio,
        speckleWindowSize,
        speckleRange,
        mode
    )),
#else
    stereo(StereoBM::create(
        num_disparity,
        window_block_size
    )),
#endif
    stereoR(ximgproc::createRightMatcher(stereo)),
    wls_filter(ximgproc::createDisparityWLSFilter(stereo)),
    initiation_undistored(false)
{  }

DispMap::~DispMap() { 
    left_stereo_map.first.release();
    left_stereo_map.second.release();
    right_stereo_map.first.release();
    right_stereo_map.second.release();
}

void DispMap::initUndistored(const cv::Size left_img_size_, const cv::Size right_img_size_) {

    initUndistortRectifyMap(
        cam_l, dist_coeff_l, recti_l, proj_l,
        left_img_size_,
        CV_32FC1,
        left_stereo_map.first,
        left_stereo_map.second
    );

    initUndistortRectifyMap(
        cam_r, dist_coeff_r, recti_r, proj_r,
        right_img_size_,
        CV_32FC1,
        right_stereo_map.first,
        right_stereo_map.second
    );

    initiation_undistored = true;

}

void DispMap::makingDisparityProcess(const Mat& left_img_, const Mat& right_img_) {

    if (!initiation_undistored) {
        alert::critic_runtime_error("%s Undistorted parameter is not taken.", __METHOD_NAME__);
    }

    cv::Mat flat_img_left, flat_img_right;
    remap(left_img_, flat_img_left, left_stereo_map.first, left_stereo_map.second, INTER_LINEAR, BORDER_CONSTANT);
    remap(right_img_, flat_img_right, right_stereo_map.first, right_stereo_map.second, INTER_LINEAR, BORDER_CONSTANT);

    Mat gray_img_l, gray_img_r;
    cvtColor(flat_img_left, gray_img_l, COLOR_RGB2GRAY);
    cvtColor(flat_img_right, gray_img_r, COLOR_RGB2GRAY);
    
    // ------- disparity map
    Mat stereo_compute_l, disp_l;
    stereo->compute(gray_img_l, gray_img_r, stereo_compute_l);
    stereo_compute_l.convertTo(disp_l, CV_16S);

    Mat stereo_compute_r, disp_r;
    stereoR->compute(gray_img_r, gray_img_l, stereo_compute_r);
    stereo_compute_r.convertTo(disp_r, CV_16S);

    // ------- filter stereo

    Mat filtered_b4_normalize, filtered_normalize;
    wls_filter->filter(disp_l, left_img_, filtered_b4_normalize, disp_r);
    normalize(filtered_b4_normalize, filtered_normalize, 255.0, 0.0, NORM_MINMAX);

    Mat final_filtered;
    filtered_normalize.convertTo(final_filtered, CV_8U);

    disparity_filtered_image = final_filtered.clone();
}

Mat DispMap::disparityImage() const {
    return disparity_filtered_image;
}