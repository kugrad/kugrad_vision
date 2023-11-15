#ifndef __EST_COORD_H__
#define __EST_COORD_H__

#include <ros/ros.h>

#include <geometry_msgs/Pose2D.h>
#include "depth_estimation/corner_infos.h"
#include "disparity/ReadStereoFS.h"

#include <opencv2/core/mat.hpp>

#include <mutex>

class EstimationCoord {

public:
    EstimationCoord();
    ~EstimationCoord();

    void indexMapCoordCallback(
        const depth_estimation::corner_infos::ConstPtr& index_coords_
    );
    void imageCoordCallback(
        const geometry_msgs::Pose2D::ConstPtr& image_coord_
    );

private:

    ros::NodeHandle nh;

    ros::Subscriber index_map_coord_sub;
    ros::Subscriber change_obj_image_coord_sub;

    std::mutex corner_map_coord_mx;

    bool corner_idx_is_set;
    cv::Mat transformation_matrix;

    const double idx_x_len = 100.0f;
    const double idx_y_len = 100.0f;
    double scaling_factor_X, scaling_factor_Y;

    std::shared_ptr<ReadStereoFS> config_fs;
};

#endif