#include <iostream>

#include "cam_calrec/CamConfig.h"
#include "cam_calrec/StereoCalib.h"

int main(int argc, char* argv[]){

    CamConfig config(CONFIG_DIR_PATH "cam_configuration.json");

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