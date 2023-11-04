#include "CamModify.h"

// #include <pcl_ros/point_cloud.h> // ros point cloud libraries

#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

using namespace cv;

CamModify::CamModify(const ReadStereoFS& config_storage)
    :
    config_storage(config_storage),
#if TEST
    stream_l(VideoCapture(DATA_DIR_PATH "left.mp4")),
    stream_r(VideoCapture(DATA_DIR_PATH "right.mp4")),
#else
    stream_l(VideoCapture(0)),
    stream_r(VideoCapture(2)),
#endif
#if STEREOSGBM
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
    wls_filter(ximgproc::createDisparityWLSFilter(stereo))
{ 
    if (!stream_l.isOpened()) {
        throw std::runtime_error(
            "Left camera is not opened. _ Constructor CamModify"
        );
    }

    if (!stream_l.isOpened()) {
        throw std::runtime_error(
            "Right camera is not opened. _ Constructor CamModify"
        );
    }

    stream_l.set(CAP_PROP_FRAME_WIDTH, 640);
    stream_l.set(CAP_PROP_FRAME_HEIGHT, 480);
    stream_l.set(CAP_PROP_FPS, 30.0);
    stream_r.set(CAP_PROP_FRAME_WIDTH, 640);
    stream_r.set(CAP_PROP_FRAME_HEIGHT, 480);
    stream_r.set(CAP_PROP_FPS, 30.0);

    wls_filter->setLambda(lmbda);
    wls_filter->setSigmaColor(sigma);
}

CamModify::~CamModify() { 
    stream_l.release();
    stream_r.release();

    left_stereo_map.first.release();
    left_stereo_map.second.release();
    right_stereo_map.first.release();
    right_stereo_map.second.release();

    image_l.release();
    image_r.release();
    gray_l.release();
    gray_r.release();
    point_cloud.release();
    filtered_img.release();

    stereo.release();
    stereoR.release();
    wls_filter.release();
 }

std::pair<pairMatMat, pairMatMat> CamModify::undistortInfoMat() {
    initUndistortRectifyMap(
        config_storage.cameraMat_left(),
        config_storage.distCoeff_left(),
        config_storage.rectifyMat_left(),
        config_storage.projectionMat_left(),
        Size(640, 480),
        CV_32FC1,
        left_stereo_map.first,
        left_stereo_map.second
    );

    initUndistortRectifyMap(    
        config_storage.cameraMat_right(),
        config_storage.distCoeff_right(),
        config_storage.rectifyMat_right(),
        config_storage.projectionMat_right(),
        Size(640, 480),
        CV_32FC1,
        right_stereo_map.first,
        right_stereo_map.second
    );

    return std::make_pair(left_stereo_map, right_stereo_map);
}

pairMatMat CamModify::takePicture() {
    stream_l.read(image_l);
    stream_r.read(image_r);

    return std::make_pair(image_l, image_r);
}

pairMatMat CamModify::undistortImage() {

    Mat out_l, out_r;

    // remap(image_l, out_l, left_stereo_map.first, left_stereo_map.second, INTER_LANCZOS4, BORDER_CONSTANT);
    // remap(image_r, out_r, right_stereo_map.first, right_stereo_map.second, INTER_LANCZOS4, BORDER_CONSTANT);
    remap(image_l, out_l, left_stereo_map.first, left_stereo_map.second, INTER_LINEAR, BORDER_CONSTANT);
    remap(image_r, out_r, right_stereo_map.first, right_stereo_map.second, INTER_LINEAR, BORDER_CONSTANT);

    image_l = out_l.clone();
    image_r = out_r.clone();

    out_l.release();
    out_r.release();

    return std::make_pair(image_l, image_r);
}

#include <opencv2/imgproc.hpp>  

pairMatMat CamModify::imageCvt2Gray() {
    cvtColor(image_l, gray_l, COLOR_RGB2GRAY);
    cvtColor(image_r, gray_r, COLOR_RGB2GRAY);

    return std::make_pair(gray_l, gray_r);
}

pairMatMat CamModify::makeDisparityImages() {

    Mat out_left, out_right;

    stereo->compute(gray_l, gray_r, out_left);
    out_left.convertTo(disp_l, CV_16S);

    stereoR->compute(gray_r, gray_l, out_right);
    out_right.convertTo(disp_r, CV_16S);

    out_left.release();
    out_right.release();

    return std::make_pair(disp_l, disp_r);
}

cv::Mat CamModify::filterStereoImage() {

    Mat out;

    wls_filter->filter(disp_l, image_l, out, disp_r);
    normalize(out, out, 255.0, 0.0, NORM_MINMAX);

    out.convertTo(filtered_img, CV_8U);

    out.release();

    return filtered_img;
}

void CamModify::showResultImage() {
    imshow("result", filtered_img);
}

cv::Mat CamModify::calculate3DCoordinate() {
    reprojectImageTo3D(filtered_img, point_cloud, config_storage.disparity_Q(), true);
    imshow("point", point_cloud);

    return point_cloud;
}





// TODO Continuing CamModify class based on Distance-Estimation-and-Depth-Map-creation-using-OpenCV source. 
    //     // Calculate 3D co-ordinates from disparity image
    //     reprojectImageTo3D(disp_compute, pointcloud, Q, true);

    //     // Draw green rectangle around 40 px wide square area im image
    //     int xmin = leftVidFrame.cols/2 - 20, xmax = leftVidFrame.cols/2 + 20, ymin = leftVidFrame.rows/2 - 20,
    // ymax = leftVidFrame.rows/2 + 20;
    //     rectangle(leftVidFrame_rect, Point(xmin, ymin), Point(xmax, ymax), Scalar(0, 255, 0));
    //     waitKey(100);

    //     // Extract depth of 40 px rectangle and print out their mean
    //     pointcloud = pointcloud(Range(ymin, ymax), Range(xmin, xmax));
    //     Mat z_roi(pointcloud.size(), CV_32FC1);
    //     int from_to[] = {2, 0};
    //     mixChannels(&pointcloud, 1, &z_roi, 1, from_to, 1);

    //     cout << "Depth: " << mean(z_roi) << " CM" << endl;

    //     imshow("Disparity", disp_show);
    //     imshow("Left", leftVidFrame_rect);
    //     waitKey(100);