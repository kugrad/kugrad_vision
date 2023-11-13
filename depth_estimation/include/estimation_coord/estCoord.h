#ifndef __EST_COORD_H__
#define __EST_COORD_H__

#include <ros/ros.h>

#include <geometry_msgs/Pose2D.h>
#include "depth_estimation/corner_infos.h"

#include <mutex>

class EstimationCoord {

public:
    EstimationCoord();
    ~EstimationCoord();

    void indexMapCoordCallback(
        const depth_estimation::corner_infos::ConstPtr& index_coords
    );
    void imageCoordCallback(
        const geometry_msgs::Pose2D::ConstPtr& image_coord 
    );

private:
    ros::NodeHandle nh;

    ros::Subscriber index_map_coord_sub;
    ros::Subscriber change_obj_image_coord_sub;


    std::mutex index_map_coord_mx;
    std::mutex image_coord_mx;

};

#endif