#include "estimation_coord/estCoord.h"

#include <vector>
#include <algorithm>

EstimationCoord::EstimationCoord()
    :
    index_map_coord_sub(nh.subscribe("index_map_image", 10, &EstimationCoord::indexMapCoordCallback, this)),
    change_obj_image_coord_sub(nh.subscribe("changed_coordinate_from_image", 100, &EstimationCoord::imageCoordCallback, this))
{

}

EstimationCoord::~EstimationCoord()
{  }

void EstimationCoord::indexMapCoordCallback(
    const depth_estimation::corner_infos::ConstPtr& index_coords_
) {

    auto corners = index_coords_.get()->depth_corners;
    // std::vector<depth_estimation::corner_info*> infos(corners.begin(), corners.end());
    std::sort(
        corners.begin(), corners.end(),
        [](const depth_estimation::corner_info& former, const depth_estimation::corner_info& latter) {
            if (former.x < latter.x)
                return true;
            return false;
        }
    );
    // std::sort(infos.begin(), infos.end())
    corner_map_coord_mx.lock(); 
    if (corners.at(0).y < corners.at(1).y) {
        left_down = corners.at(0);
        left_up = corners.at(1);
    } else {
        left_down = corners.at(1);
        left_up = corners.at(0);
    }

    if (corners.at(2).y < corners.at(3).y) {
        right_down = corners.at(2);
        right_up = corners.at(3);
    } else {
        right_down = corners.at(3);
        right_up = corners.at(2);
    }
    corner_map_coord_mx.unlock(); 
}

void EstimationCoord::imageCoordCallback(
    const geometry_msgs::Pose2D::ConstPtr& image_coord
) {
    image_coord_mx.lock();
    changed_coord = image_coord;
    image_coord_mx.unlock();
}
