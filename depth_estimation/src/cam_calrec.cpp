#include "ros/ros.h"

#include "cam_calrec/CalibConfig.h"
#include "cam_calrec/StereoCalib.h"

int main(int argc, char* argv[]){

    ros::init(argc, argv, "stereo_image_listener");

    ros::NodeHandle nh;

    CalibConfig config(CONFIG_DIR_PATH "calib_recti_config.json");

    StereoCalib s_calib(
        config.numHorizontalCorner(),
        config.numVerticalCorner(),
        config.chessboardSquareLength()
    );

    // s_calib.startStereoCalibNRect();

    ros::spin();

    return 0;
}