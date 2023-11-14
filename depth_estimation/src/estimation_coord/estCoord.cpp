#include "estimation_coord/estCoord.h"
#include "utils.h"

#include <algorithm>
#include <cmath>

#include <opencv2/core.hpp>
#include <opencv2/opencv.hpp>
#include <fmt/core.h>
#include <fmt/color.h>

// typedef struct {
//     double x;
//     double y;
// } Vector2D;

// typedef struct {
//     double m11, m12;
//     double m21, m22;
// } Matrix2x2;

using namespace cv;

EstimationCoord::EstimationCoord()
    :
    config_fs(std::make_shared<ReadStereoFS>(CONFIG_DIR_PATH "calib_storage.yaml")),
    index_map_coord_sub(nh.subscribe("index_map_image", 10, &EstimationCoord::indexMapCoordCallback, this)),
    change_obj_image_coord_sub(nh.subscribe("changed_coordinate_from_image", 100, &EstimationCoord::imageCoordCallback, this)),
    corner_idx_is_set(false)
{ 
}

EstimationCoord::~EstimationCoord()
{  }

void EstimationCoord::indexMapCoordCallback(
    const depth_estimation::corner_infos::ConstPtr& index_coords_
) {
    ROS_INFO("received index_coords_");

    auto corners = index_coords_.get()->depth_corners;

    if (corners.at(0).x == 0 && corners.at(0).y == 0) {
        corner_map_coord_mx.lock();
        corner_idx_is_set = false;
        corner_map_coord_mx.unlock();
    }

    std::sort(
        corners.begin(), corners.end(),
        [](const depth_estimation::corner_info& former, const depth_estimation::corner_info& latter) {
            if (former.x < latter.x)
                return true;
            return false;
        }
    );

    depth_estimation::corner_info left_up, left_down, right_up, right_down;

    if (corners.at(0).y < corners.at(1).y) {
        left_down = corners.at(1);
        left_up = corners.at(0);
    } else {
        left_down = corners.at(0);
        left_up = corners.at(1);
    }

    if (corners.at(2).y < corners.at(3).y) {
        right_down = corners.at(3);
        right_up = corners.at(2);
    } else {
        right_down = corners.at(2);
        right_up = corners.at(3);
    }

    std::vector<Point2f> src_corners;
    src_corners.push_back(Point2f(left_down.x, left_down.y));
    src_corners.push_back(Point2f(left_up.x, left_up.y));
    src_corners.push_back(Point2f(right_down.x, right_down.y));
    src_corners.push_back(Point2f(right_up.x, right_up.y));

    std::vector<Point2f> dst_corners;
    dst_corners.push_back(Point2f(0.0f, 0.0f));
    dst_corners.push_back(Point2f(0.0f, idx_y_len));
    dst_corners.push_back(Point2f(idx_x_len, 0.0f));
    dst_corners.push_back(Point2f(idx_x_len, idx_y_len));

    corner_map_coord_mx.lock();
    transformation_matrix = getPerspectiveTransform(src_corners, dst_corners);
    corner_idx_is_set = true;
    corner_map_coord_mx.unlock();
}

void EstimationCoord::imageCoordCallback(
    const geometry_msgs::Pose2D::ConstPtr& image_coord
) {
    bool is_pass;
    corner_map_coord_mx.lock();
    is_pass = corner_idx_is_set;
    corner_map_coord_mx.unlock();

    if (!is_pass)
        return;

    std::vector<Point2f> d_points({ Point2f(image_coord->x, image_coord->y) });
    std::vector<Point2f> ud_points;
    cv::undistortPoints(d_points, ud_points,
        config_fs->cameraMat_left(),
        config_fs->distCoeff_left(),
        config_fs->rectifyMat_left(),
        config_fs->projectionMat_left()
    );

    std::vector<Point2f> transfomed_points;
    perspectiveTransform(ud_points, transfomed_points, transformation_matrix); 

    Point2f& pt = transfomed_points[0];

    alert::info_message( "Axis - ( %lf ,%lf )\n", pt.x, pt.y);
}
