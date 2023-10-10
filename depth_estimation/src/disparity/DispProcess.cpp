#include "disparity/DispProcess.h"
#include "disparity/ReadStereoFS.h"
#include "disparity/DispMap.h"

#include <cv_bridge/cv_bridge.h>

#include <boost/bind.hpp>

#include <opencv2/highgui/highgui.hpp>

#define SHOW_IMAGE 1

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
    sync( SyncPolicy( 10 ), left_image_sub, right_image_sub )
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
    cv::imshow("left_image", left_image);
    cv::imshow("right_image", right_image);
    cv::waitKey(1);
#endif /* SHOW_IMAGE */

    // TODO code process here

}