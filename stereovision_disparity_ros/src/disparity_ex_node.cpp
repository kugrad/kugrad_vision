#include <ros/ros.h>

#include <iostream>
// #include <chrono>
// #include <thread>

#include <chrono>
#include <thread>

#include <opencv2/highgui/highgui.hpp>

#include "ReadStereoFS.h"
#include "CamModify.h"

using namespace std::chrono;

int main(int argc, char* argv[]){

    ros::init(argc, argv, "stereovision_disparity_ros");

    double desiredFrequency = 30.0; // 30Hz
    std::chrono::duration<double> timeInterval(1.0 / desiredFrequency);

    ReadStereoFS config_storage(CONFIG_DIR_PATH "calib_storage.yaml");
    CamModify core(config_storage);

    core.undistortInfoMat();

    ros::Time::init();
    ros::Rate loop_rate(30);

    while (ros::ok()) {

        core.takePicture();
        core.undistortImage();
        core.imageCvt2Gray();
        core.makeDisparityImages();
        core.filterStereoImage();
        core.showResultImage();
        core.calculate3DCoordinate();

        if (cv::waitKey(1) == 27)
            break;

        loop_rate.sleep();
        
        ros::spinOnce();
    }

    return 0;
}

