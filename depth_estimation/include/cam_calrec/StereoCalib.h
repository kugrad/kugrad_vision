#ifndef __STEREO_CALIB_H__
#define __STEREO_CALIB_H__

#include <ros/ros.h>

#if USE_IMAGE_TRANSPORT_SUBSCRIBER_FILTER
#include <image_transport/subscriber_filter.h>
#else
#include <sensor_msgs/Image.h>
#include <message_filters/subscriber.h>
#endif /* USE_IMAGE_TRANSPORT_SUBSCRIBER_FILTER */

#if EXACT_TIME_SYNC
#include <message_filters/sync_policies/exact_time.h>
#else
#include <message_filters/sync_policies/approximate_time.h>
#endif /* EXACT_TIME_SYNC */

#include "opencv2/core/core.hpp"
#include "opencv2/calib3d/calib3d.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"

#include <iostream>
#include <stdio.h>

// using namespace cv;
// using namespace std;

class StereoCalib {

public:
    // StereoCalib(cv::Mat& actualOne, cv::Mat& actualTwo);
    StereoCalib(
        // std::string left_cam_path,
        // std::string right_cam_path,
        int chessboard_horizontal_corner_num,
        int chessboard_vertical_corner_num,
        int chessboard_square_size
    );
    ~StereoCalib();

    void stereoCalibrationProcessCallback(
        const sensor_msgs::ImageConstPtr& left_image_,
        const sensor_msgs::ImageConstPtr& right_iamge_
    );
    
private:

    ros::NodeHandle nh;
    image_transport::ImageTransport imgTrans;

#if USE_IMAGE_TRANSPORT_SUBSCRIBER_FILTER
    typedef image_transport::SubscriberFilter ImageSubscriber;
#else
    typedef message_filters::Subscriber< sensor_msgs::Image > ImageSubscriber;
#endif /* USE_IMAGE_TRANSPORT_SUBSCRIBER_FILTER */
    ImageSubscriber left_image_sub;
    ImageSubscriber right_image_sub;

#if EXACT_TIME_SYNC
    typedef message_filters::sync_policies::ExactTime<sensor_msgs::Image, sensor_msgs::Image> SyncPolicy;
#else
    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image> SyncPolicy;
#endif /* EXACT_TIME_SYNC */
    message_filters::Synchronizer<SyncPolicy> sync;

    cv::Mat left_image; // left image
    cv::Mat right_image; // right image

    std::vector<cv::Point3f> objp;

    int hor_corner_n;  //Horizontal corners
    int ver_corner_n; //Vertical corners
    float square_size;

    // calibration parameter both intrinsic and extrinsic
    std::vector<std::vector<cv::Point3f>> object_points; //Represents the 3D corners actual location
    std::vector<std::vector<cv::Point2f>> img_points_l, img_points_r; //Represent the location of corners detected in 3D
    std::vector<cv::Point2f> corner_pts_l, corner_pts_r;
    // vector<Point3f> obj;
    
    
    /**
     * @brief calibration parameter both intrinsic and extrinsic
     * 
     */
    // camera_mat's are 3x3 floating point arrays of each camera
    // dist_coeff's are distortion coefficients vectors of each camera
    // dist_coeff's Matrix of distortion coefficient of camera left and right
    cv::Mat camera_mat_left, camera_mat_right;
    cv::Mat dist_coeff_left, dist_coeff_right;
    
    // rotation_mat     - Rotation Matrix between the first and second camera coordinate systems
    // translation_mat  - Translation vector between the cameras coordinate systems
    // essential_mat    - Essential Matrix
    // fundamental_mat  - Fundamental matrix
    cv::Mat rotation_mat;
    cv::Mat translation_mat;
    cv::Mat essential_mat;
    cv::Mat fundamental_mat;
    
    /**
     * @brief rectification parameter
     * 
     */
    // rectify_left - 3x3 Rectification Transformation (Rotation Matrix) for the left Camera
    // rectify_right - 3x3 Rectification Transformation (Rotation Matrix) for the right Camera
    // projection_left - Projection matrix 3x4 in the new and rectified coordinate system of the left camera
    // projection_right - Projection matrix 3x4 in the new and rectified coordinate system of the right camera
    // disparity - Disparity matrix by depth 4x4
    cv::Mat rectify_left;
    cv::Mat rectify_right;
    cv::Mat projection_left;
    cv::Mat projection_right;
    cv::Mat disparity;
    
};

#endif /* __STEREO_CALIB_H */
