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

EstimationCoord::EstimationCoord()
    :
    config_fs(std::make_shared<ReadStereoFS>(CONFIG_DIR_PATH "calib_storage.yaml")),
    index_map_coord_sub(nh.subscribe("index_map_image", 10, &EstimationCoord::indexMapCoordCallback, this)),
    change_obj_image_coord_sub(nh.subscribe("changed_coordinate_from_image", 100, &EstimationCoord::imageCoordCallback, this)),
    corner_idx_is_set(false)
{  }

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

    // for (const auto it : corners) {
    //     std::cout << it.x << ", " << it.y << "\n";
    // }

    depth_estimation::corner_info left_up, left_down, right_up, right_down;

    // corner_map_coord_mx.lock(); 
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
    // corner_idx_is_set = true;
    // corner_map_coord_mx.unlock(); 

    alert::info_message(
        fmt::format(
            "({}, {}), ({}, {}), ({}, {}), ({}, {})",
            left_down.x, left_down.y,
            left_up.x, left_up.y,
            right_down.x, right_down.y,
            right_up.x, right_up.y
        ).c_str()
        // "{%d, %d}, {%d, %d}, {%d, %d}, {%d, %d}\n",
        // left_down.x, left_down.y,
        // left_up.x, left_up.y,
        // right_down.x, right_down.y,
        // right_up.x, right_up.y
    );

    // Vector2D
    std::pair<uint32_t, uint32_t> xaxis = { right_down.x - left_down.x, right_down.y - left_down.y };
    std::pair<uint32_t, uint32_t> yaxis = { left_up.x - left_down.x, left_up.y - left_down.y };

    // cv::Mat T(2, 2, CV_32F) ;
    // T << xaxis.first, yaxis.first, xaxis.second, yaxis.second;
    // cv::Mat T = cv::Mat(2, 2, CV_32F) << xaxis.first, yaxis.fir
    cv::Mat T = (cv::Mat_<double>(2, 2) << xaxis.first, yaxis.first, xaxis.second, yaxis.second);

    if (cv::determinant(T) == 0) {
        alert::info_message("no determinent matrix based on corner idx matrix\n");
        return;
    }

    double candi_x_len = sqrt(pow(xaxis.first, 2) + pow(xaxis.second, 2));
    double candi_y_len = sqrt(pow(yaxis.first, 2) + pow(yaxis.second, 2));

    corner_map_coord_mx.lock();
    T_change_coord = T.inv();
    scaling_factor_X = (idx_x_len) / (candi_x_len);
    scaling_factor_Y = (idx_y_len) / (candi_y_len);
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

    cv::Point2f d_point(image_coord->x, image_coord->y);
    std::vector<cv::Point2f> d_points({ d_point });
    std::vector<cv::Point2f> ud_points;
    cv::undistortPoints(d_points, ud_points, config_fs->cameraMat_left(), config_fs->distCoeff_left());

    cv::Mat point = (cv::Mat_<double>(2, 1) << ud_points[0].x, ud_points[0].y);

    cv::Mat transformed = T_change_coord * point;

    auto x_coord = transformed.at<double>(0, 0);
    auto y_coord = transformed.at<double>(1, 0);

    x_coord *= scaling_factor_X;
    y_coord *= scaling_factor_Y;

    alert::info_message( "Axis - ( %lf ,%lf )\n", x_coord, y_coord );
}
