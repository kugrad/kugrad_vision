#ifndef __DISP_MAP_H__ 
#define __DISP_AMP_H__ 

#include "disparity/ReadStereoFS.h"

#include <opencv2/core/mat.hpp>
#include <opencv2/videoio.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/ximgproc/disparity_filter.hpp>

class DispMap {
public:
    // init. setting value have to be constant value.
    // DispMap(const ReadStereoFS& config_fs);
    DispMap(ReadStereoFS* config_fs_ptr);
    DispMap(
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

    void initUndistored(const cv::Size left_img_size_, const cv::Size right_img_size_);
    void makingDisparityProcess(const cv::Mat& left_img_, const cv::Mat& right_img_);
    cv::Mat disparityImage() const;

private:

    const cv::Mat cam_l;
    const cv::Mat cam_r;
    const cv::Mat dist_coeff_l;
    const cv::Mat dist_coeff_r;
    const cv::Mat recti_l;
    const cv::Mat recti_r;
    const cv::Mat proj_l;
    const cv::Mat proj_r;

    bool initiation_undistored;
    // undistort map for left camera
    std::pair<cv::Mat, cv::Mat> left_stereo_map;
    // undistort map for right camera
    std::pair<cv::Mat, cv::Mat> right_stereo_map;

    // cv::Mat image_l, image_r; // store undistored images
    cv::Mat disparity_filtered_image;

#if STEREO_SGBM
    cv::Ptr<cv::StereoSGBM> stereo;

    // Stereo SBGM parameter value
    constexpr static int smoothing_factor = 4; // use it for P1 and P2
    constexpr static int window_block_size = 3;
    constexpr static int min_disparity = 0;
    // constexpr static int num_disparity = 16 * 5 - mindisparity; 
    constexpr static int num_disparity = 16 * 5;
    constexpr static int P1 = 8 * window_block_size * window_block_size * smoothing_factor;
    constexpr static int P2 = 32 * window_block_size * window_block_size * smoothing_factor;
    // constexpr static int disp12MaxDiff = 5;
    constexpr static int disp12MaxDiff = 0;
    constexpr static int preFilterCap = 25;
    constexpr static int uniquenessRatio = 10;
    constexpr static int speckleWindowSize = 100;
    constexpr static int speckleRange = 1;
    constexpr static int mode = cv::StereoSGBM::MODE_SGBM_3WAY;
#else // STEREO_BM
    cv::Ptr<cv::StereoBM> stereo;

    // Stereo BM parameter value
    constexpr static int num_disparity = 16;
    constexpr static int window_block_size = 11;
#endif

    cv::Ptr<cv::StereoMatcher> stereoR;
    cv::Ptr<cv::ximgproc::DisparityWLSFilter> wls_filter;

    // wls filter parameter value
    constexpr static double lmbda = 8000.0;
    constexpr static double sigma = 1.8;

};

#endif