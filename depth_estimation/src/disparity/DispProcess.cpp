#include "disparity/DispProcess.h"

#include <cv_bridge/cv_bridge.h>
#include <boost/bind.hpp>

#include <fmt/core.h>

#include <opencv2/highgui/highgui.hpp>

using namespace cv;

#if SHOW_IMAGE
#include <vector>
#include <iostream>
void windowMouseCallback(int event, int x, int y, int flag, void* params) {

    Mat disp = *(static_cast<Mat*>(((void**) params)[0]));

    int baseline = 0.037;
    int focal_length = 7.16;


    if (event == EVENT_LBUTTONDOWN) {

        int disparity_value = disp.at<short>(x, y);

        fmt::print("x, y >> ({}, {})\n", x, y);
        fmt::print("disparity value: {}\n", disparity_value / (16 * 5));
        fmt::print("distance {}\n", (baseline * focal_length) / static_cast<double>(disparity_value));

    }


}
#endif

DispProcess::DispProcess()
    : 
    imgTrans(nh),
#if USE_IMAGE_TRANSPORT_SUBSCRIBER_FILTER
    left_image_sub(imgTrans, "stereo/left_image", 1),
    right_image_sub(imgTrans, "stereo/right_image", 1),
#else
    left_image_sub(nh, "stereo/left_image", 1),
    right_image_sub(nh, "stereo/right_image" 1),
#endif /* USE_IMAGE_TRANSPORT_SUBSCRIBER_FILTER */
    sync( SyncPolicy( 1 ), left_image_sub, right_image_sub ),
    config_fs(std::make_shared<ReadStereoFS>(CONFIG_DIR_PATH "calib_storage.yaml")),
    disp(std::make_shared<DispMap>(config_fs.get()))
{
    sync.registerCallback( boost::bind(&DispProcess::processCallback, this, _1, _2) );
}

void DispProcess::processCallback(const sensor_msgs::ImageConstPtr& left_image_, const sensor_msgs::ImageConstPtr& right_image_) {

    left_image = cv_bridge::toCvShare(left_image_, left_image_->encoding)->image;
    right_image = cv_bridge::toCvShare(right_image_, right_image_->encoding)->image;

    static bool undistored_ = false;
    if (!undistored_) {
        disp->initUndistored(left_image.size(), right_image.size());
    }

#if SHOW_IMAGE
    imshow("left_image", left_image);
    imshow("right_image", right_image);
#endif /* SHOW_IMAGE */

    // * ---------------------- START Code related with disparity map START ---------------------- *
    disp->makingDisparityProcess(left_image, right_image);

    Mat disp_map = disp->disparityImage();

#if SHOW_IMAGE
    void* param[] = { &disp_map };
    imshow("disparity map", disp_map);
    setMouseCallback("disparity map", windowMouseCallback, (void*) param);
#endif /* SHOW_IMAGE */

    // * ----------------------  END  Code related with disparity map  END  ---------------------- *

#if SHOW_IMAGE
    waitKey(1); // To display imshow
#endif
}