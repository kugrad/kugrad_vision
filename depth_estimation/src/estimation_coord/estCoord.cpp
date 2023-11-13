#include "estimation_coord/estCoord.h"

EstimationCoord::EstimationCoord()
    :
    index_map_coord_sub(nh.subscribe("index_map_image", 10, &EstimationCoord::indexMapCoordCallback, this)),
    change_obj_image_coord_sub(nh.subscribe("coordinate_from_image", 100, &EstimationCoord::imageCoordCallback, this))
{

}

EstimationCoord::~EstimationCoord()
{  }

void EstimationCoord::indexMapCoordCallback(
    const depth_estimation::corner_infos::ConstPtr& index_coords
) {

}

void EstimationCoord::imageCoordCallback(
    const geometry_msgs::Pose2D::ConstPtr& image_coord
) {

}
