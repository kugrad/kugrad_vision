#include "disparity/DispMap.h"
#include "utils.h"

#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

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

Point2d DispMap::undistortPoint(const std::pair<uint32_t, uint32_t> coord) {
    // Mat coord_mat()
    // Point2d coord_arr(coord.first, coord.second);
    // Mat coord_mat = (Mat_<int>(2, 1) << coord.first, coord.se
    Mat coord_mat = (Mat_<int>(1, 2) << coord.first, coord.second);
    Mat coord_mat_after;
    undistortPoints(coord_mat, coord_mat_after, cam_l, dist_coeff_l);

    return Point2d(coord_mat_after.at<uint32_t>(0, 0), coord_mat_after.at<uint32_t>(0, 1));
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

    Mat undistorted_img_l, undistorted_img_r;
    remap(left_img_, undistorted_img_l, left_stereo_map.first, left_stereo_map.second, INTER_LINEAR, BORDER_CONSTANT);
    remap(right_img_, undistorted_img_r, right_stereo_map.first, right_stereo_map.second, INTER_LINEAR, BORDER_CONSTANT);

    Mat gray_img_l, gray_img_r;
    cvtColor(undistorted_img_l, gray_img_l, COLOR_RGB2GRAY);
    cvtColor(undistorted_img_r, gray_img_r, COLOR_RGB2GRAY);

    // ------- Gaussian blur
    cv::Mat after_gaus_l, after_gaus_r;
    cv::GaussianBlur(gray_img_l, after_gaus_l, Size(3, 3), 0, 0, INTER_LINEAR);
    cv::GaussianBlur(gray_img_r, after_gaus_r, Size(3, 3), 0, 0, INTER_LINEAR);

    // ------- Canny endge
    // cv::Mat canny_l, canny_r;
    // cv::Canny(gaus_l, canny_l, 50, 150);
    // cv::Canny(gaus_r, canny_r, 50, 150);
    
    // ------- disparity map
    Mat disp_l, disp_r;
    stereo->compute(after_gaus_l, after_gaus_r, disp_l);
    stereoR->compute(after_gaus_r, after_gaus_l, disp_r);
    disp_l.convertTo(disp_l, CV_16S);
    disp_r.convertTo(disp_r, CV_16S);

    l_disparity_m = disp_l.clone();

    // imshow("disp_l", disp_l / (16 * 5));

    // ------- filter stereo

    Mat filtered_b4_normalize, filtered_normalize;
    wls_filter->filter(disp_l, gray_img_l, filtered_b4_normalize, disp_r);
    normalize(filtered_b4_normalize, filtered_normalize, 255.0, 0.0, NORM_MINMAX);

    Mat final_filtered;
    filtered_normalize.convertTo(final_filtered, CV_8U);

    disparity_filtered_image = final_filtered.clone();
}

const Mat DispMap::leftDisparityMap() const {
    return l_disparity_m;
}

const Mat DispMap::wlsFilteredImage() const {

    Mat color_filtered;
    cv::applyColorMap(disparity_filtered_image, color_filtered, cv::COLORMAP_OCEAN);

    return color_filtered;
}

const double DispMap::cameraFocalLength() const {

    double avg_focal_len_px = 
    (
        cam_l.at<double>(0, 0) +
        cam_l.at<double>(1, 1) +
        cam_r.at<double>(0, 0) +
        cam_r.at<double>(1, 1)
    ) / 4;

    return avg_focal_len_px;
}