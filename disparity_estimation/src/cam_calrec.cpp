#include <iostream>

#include "CalibConfig.h"
#include "StereoCalib.h"

int main(int argc, char* argv[]){

    CalibConfig config(CONFIG_DIR_PATH "configuration.json");

    StereoCalib s_calib(
        config.leftCameraFdIdx(),
        config.rightCameraFdIdx(),
        config.numHorizontalCorner(),
        config.numVerticalCorner(),
        config.chessboardSquareLength()
    );
    
    // TODO implement stereo calibration below
    s_calib.startStereoCalibNRect();

    return 0;
}