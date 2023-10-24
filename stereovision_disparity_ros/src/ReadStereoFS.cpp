#include "ReadStereoFS.h"

#include <opencv2/core/persistence.hpp>

using namespace cv;

ReadStereoFS::ReadStereoFS(std::string calrec_config_path)
    : camera_mat_l(Mat(3, 3, CV_64FC1)), camera_mat_r(Mat(3, 3, CV_64FC1))
{
    FileStorage storage(calrec_config_path, FileStorage::READ);

    storage["camera_matrix_left"] >> camera_mat_l; 
    storage["camera_matrix_right"] >> camera_mat_r;

    storage["dist_coefficient_left"] >> dist_coefficient_l;
    storage["dist_coefficient_right"] >> dist_coefficient_r;

    storage["rectification_left"] >> rectify_mat_l;
    storage["rectification_right"] >> rectify_mat_r;

    storage["projection_left"] >> projection_mat_l;
    storage["projection_right"] >> projection_mat_r;

    storage["disparity"] >> Q;

    storage.release();
}

ReadStereoFS::~ReadStereoFS() {
    // ? DO NOTHING
}

const Mat ReadStereoFS::cameraMat_left() const {
    return camera_mat_l;
}

const Mat ReadStereoFS::cameraMat_right() const {
    return camera_mat_r;
}

const Mat ReadStereoFS::distCoeff_left() const {
    return dist_coefficient_l;
}

const Mat ReadStereoFS::distCoeff_right() const {
    return dist_coefficient_r;
}

const Mat ReadStereoFS::rectifyMat_left() const {
    return rectify_mat_l; 
}

const Mat ReadStereoFS::rectifyMat_right() const {
    return rectify_mat_r;
}

const Mat ReadStereoFS::projectionMat_left() const {
    return projection_mat_l;
}

const Mat ReadStereoFS::projectionMat_right() const {
    return projection_mat_r;
}

const Mat ReadStereoFS::disparity_Q() const {
    return Q;
}