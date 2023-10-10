#include "disparity/DispProcess.h"

#include <cv_bridge/cv_bridge.h>

#include <boost/bind.hpp>

#include <opencv2/highgui/highgui.hpp>

using namespace cv;

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
    sync( SyncPolicy( 10 ), left_image_sub, right_image_sub ),
    config_fs(ReadStereoFS(CONFIG_DIR_PATH "caminfo_sotrage.yaml")),
    config_calrec(CalibConfig(CONFIG_DIR_PATH "calib_recti_config.json")),
    disp(DispMap(config_calrec, config_fs))
{
    /*
       TODO disparity map initialization here
     */
    sync.registerCallback( boost::bind(&DispProcess::processCallback, this, _1, _2) );
}

void DispProcess::processCallback(const sensor_msgs::ImageConstPtr& left_image_, const sensor_msgs::ImageConstPtr& right_image_) {

    left_image = cv_bridge::toCvShare(left_image_, left_image_->encoding)->image;
    right_image = cv_bridge::toCvShare(right_image_, right_image_->encoding)->image;

#if SHOW_IMAGE
    imshow("left_image", left_image);
    imshow("right_image", right_image);
#endif /* SHOW_IMAGE */

    // * ---------------------- START Code related with disparity map START ---------------------- *
    disp.makingDisparityProcess(left_image, right_image);

    // * ----------------------  END  Code related with disparity map  END  ---------------------- *

#if SHOW_IMAGE
    waitKey(1); // To display imshow
#endif
}