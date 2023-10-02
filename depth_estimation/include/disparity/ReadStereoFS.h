#ifndef __READ_FS_H__
#define __READ_FS_H__

#include <opencv2/core/mat.hpp> 

class ReadStereoFS {
private:
    cv::Mat camera_mat_l;
    cv::Mat camera_mat_r;

    cv::Mat dist_coefficient_l;
    cv::Mat dist_coefficient_r;

    cv::Mat rectify_mat_l; 
    cv::Mat rectify_mat_r; 

    cv::Mat projection_mat_l;
    cv::Mat projection_mat_r;

public:
    ReadStereoFS() = delete;
    ReadStereoFS(std::string calrec_config_path);
    ~ReadStereoFS();

    const cv::Mat cameraMat_left() const;
    const cv::Mat cameraMat_right() const;
    const cv::Mat distCoeff_left() const;
    const cv::Mat distCoeff_right() const;
    const cv::Mat rectifyMat_left() const;
    const cv::Mat rectifyMat_right() const;
    const cv::Mat projectionMat_left() const;
    const cv::Mat projectionMat_right() const;
};

#endif