#ifndef __CAM_MODIFY_H__
#define __CAM_MODIFY_H__

#include "ReadStereoFS.h"

#include <opencv2/videoio/videoio.hpp>
#include <opencv2/ximgproc.hpp>
#include <opencv2/calib3d.hpp>

#define TEST 1
#define STEREOSGBM 1

typedef std::pair<cv::Mat, cv::Mat> pairMatMat;

class CamModify {
public:
    CamModify(const ReadStereoFS& config_storage);
    ~CamModify();

    std::pair<pairMatMat, pairMatMat> undistortInfoMat();
    pairMatMat takePicture();
    pairMatMat undistortImage();
    pairMatMat imageCvt2Gray();
    pairMatMat makeDisparityImages();
    cv::Mat filterStereoImage();
    void showResultImage();
    cv::Mat calculate3DCoordinate();



private:
    const ReadStereoFS config_storage;

    cv::VideoCapture stream_l, stream_r;

    pairMatMat left_stereo_map;
    pairMatMat right_stereo_map;

    cv::Mat image_l, image_r;
    cv::Mat gray_l, gray_r;
    cv::Mat disp_l, disp_r;
    cv::Mat filtered_img;
    cv::Mat point_cloud;

#if STEREOSGBM
    cv::Ptr<cv::StereoSGBM> stereo;
#else
    cv::Ptr<cv::StereoBM> stereo;
#endif
    cv::Ptr<cv::StereoMatcher> stereoR;
    cv::Ptr<cv::ximgproc::DisparityWLSFilter> wls_filter;


#if STEREOSGBM
    // Stereo SBGM parameter value
    constexpr static int window_block_size = 3;
    constexpr static int min_disparity = 2;
    constexpr static int num_disparity = 130 - min_disparity; 
    constexpr static int P1 = 8 * 3 * (window_block_size * window_block_size);
    constexpr static int P2 = 32 * 3 * (window_block_size * window_block_size);
    constexpr static int disp12MaxDiff = 5;
    constexpr static int preFilterCap = 10;
    constexpr static int uniquenessRatio = 10;
    constexpr static int speckleWindowSize = 100;
    constexpr static int speckleRange = 32;
    constexpr static int mode = cv::StereoSGBM::MODE_SGBM_3WAY;
#else
    const static int num_disparity = 16;
    const static int window_block_size = 21;
#endif

    // wls filter parameter value
    constexpr static double lmbda = 80000;
    constexpr static double sigma = 1.8;

};

#endif