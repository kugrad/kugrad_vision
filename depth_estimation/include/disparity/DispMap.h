#ifndef __DISP_MAP_H__ 
#define __DISP_AMP_H__ 

#include "disparity/ReadStereoFS.h"
#include "opencv2/core/mat.hpp"

class DispMap {
public:
    DispMap(ReadStereoFS config_fs);
    DispMap(
        cv::Mat cameraMat_left,
        cv::Mat distCoeff_left,
        cv::Mat rectificationMat_left,
        cv::Mat projectionMat_left,
        cv::Mat cameraMat_right,
        cv::Mat distCoeff_right,
        cv::Mat rectificationMat_right,
        cv::Mat projectionMat_right
    );
    ~DispMap();
};

#endif