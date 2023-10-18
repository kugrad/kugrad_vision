#include <ros/ros.h>

#include <iostream>
#include <chrono>
#include <thread>

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

    // Initialize a timer
    auto startTime = std::chrono::high_resolution_clock::now();

    while (true) {

        core.takePicture();
        core.undistortImage();
        core.imageCvt2Gray();
        core.makeDisparityImages();
        core.filterStereoImage();
        core.showResultImage();

        auto currentTime = std::chrono::high_resolution_clock::now();
        std::chrono::duration<double> elapsedTime = currentTime - startTime;

        if (elapsedTime >= timeInterval) {
            // Reset the timer
            startTime = std::chrono::high_resolution_clock::now();

            // Your code to be executed at 30Hz here
            // std::cout << "Running at 30Hz\n";
        }

        // Sleep for a short duration to avoid busy-waiting
        std::this_thread::sleep_for(std::chrono::milliseconds(1));

        if (cv::waitKey(1) == 27)
            break;
        
        ros::spinOnce();
    }

    return 0;
}

