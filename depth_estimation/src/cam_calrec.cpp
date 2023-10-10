#include "ros/ros.h"

#include <iostream>

#include "cam_calrec/CalibConfig.h"
#include "cam_calrec/StereoCalib.h"

// void stereoImage_cb(const stream_image_communication::Stereo_image_msg::ConstPtr& msg) {
//     // TODO callback method

// }

int main(int argc, char* argv[]){

    ros::init(argc, argv, "stereo_image_listener");

    ros::NodeHandle nh;

    // ros::Subscriber stereo_image_sub = nh.subscribe("stereo_image", 1000, stereoImage_cb);

    ros::spin();

    return 0;
    // CalibConfig config(CONFIG_DIR_PATH "cam_configuration.json");

    // StereoCalib s_calib(
    //     config.leftCameraFdIdx(),
    //     config.rightCameraFdIdx(),
    //     config.numHorizontalCorner(),
    //     config.numVerticalCorner(),
    //     config.chessboardSquareLength()
    // );
    
    // // TODO implement stereo calibration below
    // s_calib.startStereoCalibNRect();

    // return 0;

}