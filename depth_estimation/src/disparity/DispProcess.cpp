#include "disparity/DispProcess.h"

#include "depth_estimation/corner_infos.h"

#include <cv_bridge/cv_bridge.h>
#include <boost/bind.hpp>

#include <fmt/core.h>
#include <fmt/color.h>

#include <opencv2/highgui/highgui.hpp>

#include <cmath>

using namespace cv;

void DispProcess::windowMouseCallback(int event, int x, int y, int flag, void* params) {

    Mat disp = *(static_cast<Mat*>(((void**) params)[0]));
    double camera_focal_len = *(static_cast<double*>(((void**) params)[1]));
    auto corner_info_ = static_cast<std::vector<CORNER_INFO>*>(((void**) params)[2]);

    if (event == EVENT_LBUTTONDOWN) {

        double est_dist = ((camera_focal_len * baseline) / (disp.at<short>(y, x) / 16.0)) * 100;
        est_dist = round(est_dist * 1000) / 1000.0;
        // fmt::print("(coord x, y): {}, {}\n", y, x);

        fmt::print(
            "{}: {} cm\n",
            fmt::format(fg(fmt::color::light_green), "Distance"),
            est_dist
        );

        if (corner_info_->size() < 3) {
            CORNER_INFO info = { (uint32_t) x, (uint32_t) y, est_dist };
            corner_info_->push_back(info);
            fmt::print(
                "{}: ", fmt::format(fg(fmt::color::light_blue), "corner coordinate")
            );
            for (const auto it : *corner_info_) {
                fmt ::print("[ {}, {}, {} ] ", it.x, it.y, it.distance);
            }
            fmt::print("\n");
        }

    }

}

DispProcess::DispProcess()
    : 
    imgTrans(nh),
#if USE_IMAGE_TRANSPORT_SUBSCRIBER_FILTER
    left_image_sub(imgTrans, "stereo/left_image", 1),
    right_image_sub(imgTrans, "stereo/right_image", 1),
#else
    left_image_sub(nh, "stereo/left_image", 1),
    right_image_sub(nh, "stereo/right_image" 1),
#endif /* USE_IMAGE_TRANSPORT_SUBSCRIBER_FILTER */
    sync( SyncPolicy( 1 ), left_image_sub, right_image_sub ),
    config_fs(std::make_shared<ReadStereoFS>(CONFIG_DIR_PATH "calib_storage.yaml")),
    disp(std::make_shared<DispMap>(config_fs.get()))
{
    sync.registerCallback( boost::bind(&DispProcess::processCallback, this, _1, _2) );
}

void DispProcess::processCallback(const sensor_msgs::ImageConstPtr& left_image_, const sensor_msgs::ImageConstPtr& right_image_) {

    left_image = cv_bridge::toCvShare(left_image_, left_image_->encoding)->image;
    right_image = cv_bridge::toCvShare(right_image_, right_image_->encoding)->image;

    static bool undistored_ = false;
    if (!undistored_) {
        disp->initUndistored(left_image.size(), right_image.size());
    }

#if SHOW_IMAGE
    imshow("left_image", left_image);
    imshow("right_image", right_image);
#endif /* SHOW_IMAGE */

    // * ---------------------- START Code related with disparity map START ---------------------- *
    disp->makingDisparityProcess(left_image, right_image);

    Mat wls_filtered_img = disp->wlsFilteredImage();
    Mat disparity_map = disp->leftDisparityMap();
    double cam_focal_len = disp->cameraFocalLength();

    void* param[] = { &disparity_map, &cam_focal_len, &corner_info };
    imshow("disparity map", wls_filtered_img);
    setMouseCallback("disparity map", DispProcess::windowMouseCallback, (void*) param);
    // * ----------------------  END  Code related with disparity map  END  ---------------------- *

// #if SHOW_IMAGE
    char key = waitKey(1); // To display imshow
    if (key == 'c' || key == 'C') { // check
        if (this->corner_info.size() == 3) {
            // TODO get the theta of camera frame and load
            double theta = calculateTheta();
            double degree = theta * (180.0 / 3.141592);
            // if (degree < 45.0) {
            //     degree = 90 - degree;
            // }
            fmt::print("degree: {}\n", degree);
        }
    } else if (key == 'r' || key == 'R') { // reset
        fmt::print("{} corner coordinate\n", fmt::format(fg(fmt::color::pink), "RESET"));
        this->corner_info.clear();
    }
// #endif
}

const double DispProcess::calculateTheta() const {

    CORNER_INFO vec_1 = { 
        corner_info[1].x - corner_info[0].x,
        corner_info[1].y - corner_info[0].y,
        corner_info[1].distance - corner_info[0].distance
     };

    CORNER_INFO vec_2 = {
        corner_info[2].x - corner_info[0].x,
        corner_info[2].y - corner_info[0].y,
        corner_info[2].distance - corner_info[0].distance
    };

    double vec_1_dist_square_origin = pow(vec_1.x, 2) + pow(vec_1.y, 2);
    double vec_2_dist_square_origin = pow(vec_2.x, 2) + pow(vec_2.y, 2);

    double vec_1_dist_square = vec_1_dist_square_origin + pow(vec_1.distance, 2);
    double vec_2_dist_square = vec_2_dist_square_origin + pow(vec_2.distance, 2);

    double inner_product_origin =  vec_1.x * vec_2.x + vec_1.y * vec_2.y;
    double inner_product = inner_product_origin + vec_1.distance * vec_2.distance;

    double area = (0.5) * sqrt(vec_1_dist_square * vec_2_dist_square - pow(inner_product, 2));
    double area_origin =
        (0.5) * sqrt(vec_1_dist_square_origin * vec_2_dist_square_origin - pow(inner_product_origin, 2));

    double theda_radian = acos(area_origin / area);

    return theda_radian;
}