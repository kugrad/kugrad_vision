#include "cam_calrec/StereoCalib.h"
#include "utils.h"

#include <opencv2/calib3d/calib3d_c.h>

#include <cv_bridge/cv_bridge.h>
#include <boost/bind.hpp>

#include <fmt/core.h>
#include <fmt/color.h>

#include <chrono>
#include <thread>
#include <mutex>

using namespace std::chrono;

StereoCalib::StereoCalib(
    int chessboard_horizontal_corner_num,
    int chessboard_vertical_corner_num,
    int chessboard_square_size
) :
    imgTrans(nh),
#if USE_IMAGE_TRANSPORT_SUBSCRIBER_FILTER
    left_image_sub(imgTrans, "stereo/left_image", 1),
    right_image_sub(imgTrans, "stereo/right_image", 1),
#else
    left_image_sub(nh, "stereo/left_image", 1),
    right_image_sub(nh, "stereo/right_image" 1),
#endif /* USE_IMAGE_TRANSPORT_SUBSCRIBER_FILTER */
    sync( SyncPolicy( 1 ), left_image_sub, right_image_sub ),
    hor_corner_n(chessboard_horizontal_corner_num),
    ver_corner_n(chessboard_vertical_corner_num),
    square_size(chessboard_square_size)
{ 
    sync.registerCallback( boost::bind(&StereoCalib::stereoCalibrationProcessCallback, this, _1, _2) );

    // ready for calibration
    for (int i = 0; i < hor_corner_n; i++) {
        for (int j = 0; j < ver_corner_n; j++) {
            objp.push_back(Point3f(j * square_size, i * square_size, 0.0f));
        }
    }
}

StereoCalib::~StereoCalib() {
    camera_mat_left.release();
    camera_mat_right.release();
    dist_coeff_left.release();
    dist_coeff_right.release();
    rotation_mat.release();
    translation_mat.release();
    essential_mat.release();
    fundamental_mat.release();
}

/// @brief callback alert when new camera image received
/// @param left_image_  left  camera sensor_msgs pointer
/// @param right_image_ right camera seonsor_msgs pointer
void StereoCalib::stereoCalibrationProcessCallback(const sensor_msgs::ImageConstPtr& left_image_, const sensor_msgs::ImageConstPtr& right_image_) {
    left_image = cv_bridge::toCvShare(left_image_, left_image_->encoding)->image;
    right_image = cv_bridge::toCvShare(right_image_, right_image_->encoding)->image;

    static bool first_calib = false; 
    static int stereo_cnt = 0;

    Mat undis_left_image, undis_right_image;
    Mat left_image_gray, right_image_gray;
    if (first_calib) {
        Mat left_image_ = left_image.clone();
        Mat right_image_ = right_image.clone();
        undistort(left_image_, undis_left_image, camera_mat_left, dist_coeff_left);
        undistort(right_image_, undis_right_image, camera_mat_right, dist_coeff_right);
    } else {
        undis_left_image = left_image.clone();
        undis_right_image = right_image.clone();
    }

#if (CV_VERSION_MAJOR >= 4)
    cvtColor(undis_left_image, left_image_gray, COLOR_RGB2GRAY);
    cvtColor(undis_right_image, right_image_gray, COLOR_RGB2GRAY);
#else
    cvtColor(undis_left_image, left_image_gray, CV_RGB2GRAY);
    cvtColor(undis_right_image, right_image_gray, CV_RGB2GRAY);
#endif 

    bool success_l = findChessboardCorners(
        undis_left_image,
        Size(ver_corner_n, hor_corner_n),
        corner_pts_l,
        CV_CALIB_CB_ADAPTIVE_THRESH | CV_CALIB_CB_FAST_CHECK | CV_CALIB_CB_NORMALIZE_IMAGE
        // CV_CALIB_CB_ADAPTIVE_THRESH | CV_CALIB_CB_FILTER_QUADS 
    );
        
    bool success_r = findChessboardCorners(
        undis_right_image,
        Size(ver_corner_n, hor_corner_n),
        corner_pts_r,
        CV_CALIB_CB_ADAPTIVE_THRESH | CV_CALIB_CB_FAST_CHECK | CV_CALIB_CB_NORMALIZE_IMAGE
        // CV_CALIB_CB_ADAPTIVE_THRESH | CV_CALIB_CB_FILTER_QUADS 
    );

    if (success_l && success_r) {
        TermCriteria criteria(CV_TERMCRIT_EPS | CV_TERMCRIT_ITER, 30, 0.001);

        cornerSubPix(left_image_gray, corner_pts_l, Size(11, 11), Size(-1, -1), criteria);
        cornerSubPix(right_image_gray, corner_pts_r, Size(11, 11), Size(-1, -1), criteria);

        drawChessboardCorners(left_image_gray, Size(ver_corner_n, hor_corner_n), corner_pts_l, success_l);
        drawChessboardCorners(right_image_gray, Size(ver_corner_n, hor_corner_n), corner_pts_r, success_r);

        char check = static_cast<char>(waitKey(1) & 0xFF);
        if (check == 'C' || check == 'c') { // if keyboard press 'C' or 'c'
            object_points.push_back(objp); 

            img_points_l.push_back(corner_pts_l);
            img_points_r.push_back(corner_pts_r);

            alert::info_message("Start Stereo calibration");

            stereoCalibrate(
                object_points,
                img_points_l, img_points_r,
                camera_mat_left, dist_coeff_left,
                camera_mat_right, dist_coeff_right,
                Size(left_image_gray.rows, left_image_gray.cols),
                rotation_mat,
                translation_mat,
                essential_mat,
                fundamental_mat,
                CV_CALIB_SAME_FOCAL_LENGTH | CV_CALIB_ZERO_TANGENT_DIST,
                cvTermCriteria(CV_TERMCRIT_ITER + CV_TERMCRIT_EPS, 100, 1e-5)
            );

            stereo_cnt += 1;

            alert::info_message("Stereo calibration count: %d", stereo_cnt);

            first_calib = true;
        } else if (check == 27) {
            // store camera parameter
            alert::info_message("Getting Rectification matrix...");

            stereoRectify(
                camera_mat_left, dist_coeff_left,
                camera_mat_right, dist_coeff_right,
                left_image.size(),
                rotation_mat,
                translation_mat,
                rectify_left,
                rectify_right,
                projection_left,
                projection_right,
                disparity
            );

            alert::info_message("Getting Rectification matrix complete...");

            FileStorage calib_info_storage(CONFIG_DIR_PATH "calib_storage.yaml", FileStorage::WRITE);

            calib_info_storage << "camera_matrix_left" << camera_mat_left;
            calib_info_storage << "camera_matrix_right" << camera_mat_right;
            calib_info_storage << "dist_coefficient_left" << dist_coeff_left;
            calib_info_storage << "dist_coefficient_right" << dist_coeff_right;
            calib_info_storage << "rotation" << rotation_mat;
            calib_info_storage << "translation" << translation_mat;
            calib_info_storage << "essential" << essential_mat;
            calib_info_storage << "fundamental" << fundamental_mat;
            calib_info_storage << "rectification_left" << rectify_left;
            calib_info_storage << "rectification_right" << rectify_right;
            calib_info_storage << "projection_left" << projection_left;
            calib_info_storage << "projection_right" << projection_right;
            calib_info_storage << "disparity" << disparity;

            calib_info_storage.release();

        }
    }

#if SHOW_IMAGE
    imshow("image_left", left_image_gray);
    imshow("image_right", right_image_gray);
#endif

    // cv::imshow("left_image", left_image);
    // cv::imshow("right_image", right_image);

#if SHOW_IMAGE
    waitKey(1);
#endif
}

void StereoCalib::startStereoCalibNRect() {

    vector<Point3f> objp;
    for(size_t i = 0; i < hor_corner_n; i++) {
        for(size_t j = 0; j < ver_corner_n; j++) {
            objp.push_back(Point3f(j * square_size, i * square_size, 0.0f));
        }
    }

    while (true) {
        // capture left and right image synchronously
        static bool first_calib = false;
        static int stereo_cnt = 0;

        l_image_mx.lock();
        Mat img_l = this->left_image.clone();
        l_image_mx.unlock();
        r_image_mx.lock();
        Mat img_r = this->right_image.clone();
        r_image_mx.unlock();


        /* ---------- while running as 30 fps START ---------- */
        /**
         * @brief calculate calib in this section
         * 
         */
        Mat left_image_gray, right_image_gray; 
        if (first_calib) {
            Mat left_image_ = img_l.clone();
            Mat right_image_ = img_r.clone();
            undistort(left_image_, img_l, camera_mat_left, dist_coeff_left);
            undistort(right_image_, img_r, camera_mat_right, dist_coeff_right);
        }

        if (img_l.empty() || img_r.empty()) continue;
#if (CV_VERSION_MAJOR >= 4)
        cvtColor(img_l, left_image_gray, COLOR_RGB2GRAY);
        cvtColor(img_r, right_image_gray, COLOR_RGB2GRAY);
#else
        cvtColor(left_image_, left_image_gray, CV_RGB2GRAY);
        cvtColor(right_image_, right_image_gray, CV_RGB2GRAY);
#endif 

        bool success_l = findChessboardCorners(
            img_l,
            Size(ver_corner_n, hor_corner_n),
            corner_pts_l,
            CV_CALIB_CB_ADAPTIVE_THRESH | CV_CALIB_CB_FAST_CHECK | CV_CALIB_CB_NORMALIZE_IMAGE
            // CV_CALIB_CB_ADAPTIVE_THRESH | CV_CALIB_CB_FILTER_QUADS 
        );
        
        bool success_r = findChessboardCorners(
            img_r,
            Size(ver_corner_n, hor_corner_n),
            corner_pts_r,
            CV_CALIB_CB_ADAPTIVE_THRESH | CV_CALIB_CB_FAST_CHECK | CV_CALIB_CB_NORMALIZE_IMAGE
            // CV_CALIB_CB_ADAPTIVE_THRESH | CV_CALIB_CB_FILTER_QUADS 
        );

        if (success_l && success_r) {
            TermCriteria criteria(CV_TERMCRIT_EPS | CV_TERMCRIT_ITER, 30, 0.001);

            cornerSubPix(left_image_gray, corner_pts_l, Size(11, 11), Size(-1, -1), criteria);
            cornerSubPix(right_image_gray, corner_pts_r, Size(11, 11), Size(-1, -1), criteria);

            drawChessboardCorners(left_image_gray, Size(ver_corner_n, hor_corner_n), corner_pts_l, success_l);
            drawChessboardCorners(right_image_gray, Size(ver_corner_n, hor_corner_n), corner_pts_r, success_r);

            char check = static_cast<char>(waitKey(1) & 0xFF);
            if (check == 'C' || check == 'c') { // if keyboard press 'C' or 'c'
                object_points.push_back(objp); 

                img_points_l.push_back(corner_pts_l);
                img_points_r.push_back(corner_pts_r);

                alert::info_message("Start Stereo calibration");

                stereoCalibrate(
                    object_points,
                    img_points_l, img_points_r,
                    camera_mat_left, dist_coeff_left,
                    camera_mat_right, dist_coeff_right,
                    Size(left_image_gray.rows, left_image_gray.cols),
                    rotation_mat,
                    translation_mat,
                    essential_mat,
                    fundamental_mat,
                    CV_CALIB_SAME_FOCAL_LENGTH | CV_CALIB_ZERO_TANGENT_DIST,
                    cvTermCriteria(CV_TERMCRIT_ITER + CV_TERMCRIT_EPS, 100, 1e-5)
                );

                stereo_cnt += 1;

                alert::info_message("Stereo calibration count: %d", stereo_cnt);

                first_calib = true;
            }

        }

        imshow("image_left", left_image_gray);
        imshow("image_right", right_image_gray);

        /* ------------ while running as 30 fps END ----------- */

        if (waitKey(1) == 27) {
            break;
        }
    }

    destroyAllWindows();

    alert::info_message("Getting Rectification matrix...");

    stereoRectify(
        camera_mat_left, dist_coeff_left,
        camera_mat_right, dist_coeff_right,
        left_image.size(),
        rotation_mat,
        translation_mat,
        rectify_left,
        rectify_right,
        projection_left,
        projection_right,
        disparity
    );

    alert::info_message("Getting Rectification matrix complete...");

    FileStorage calib_info_storage(CONFIG_DIR_PATH "calib_storage.yaml", FileStorage::WRITE);

    calib_info_storage << "camera_matrix_left" << camera_mat_left;
    calib_info_storage << "camera_matrix_right" << camera_mat_right;
    calib_info_storage << "dist_coefficient_left" << dist_coeff_left;
    calib_info_storage << "dist_coefficient_right" << dist_coeff_right;
    calib_info_storage << "rotation" << rotation_mat;
    calib_info_storage << "translation" << translation_mat;
    calib_info_storage << "essential" << essential_mat;
    calib_info_storage << "fundamental" << fundamental_mat;
    calib_info_storage << "rectification_left" << rectify_left;
    calib_info_storage << "rectification_right" << rectify_right;
    calib_info_storage << "projection_left" << projection_left;
    calib_info_storage << "projection_right" << projection_right;
    calib_info_storage << "disparity" << disparity;

    calib_info_storage.release();
    
    // cout << Q << endl;
    
    // fs1.release();
    // cout << "Rectification completed" << endl;
    
    // cout << "Applying Undistort..." << endl;
    
    // Mat map1x, map1y, map2x, map2y;
    // Mat imgU1, imgU2, imgRectify, img1Teste, img2Teste;
    
    // initUndistortRectifyMap(CM1, D1, R1, P1, img1.size(), CV_32FC1, map1x, map1y);
    // initUndistortRectifyMap(CM2, D2, R2, P2, img2.size(), CV_32FC1, map2x, map2y);
    
    // cout << "Undistort completed" << endl;
    
    // while(1){
    //     img1Teste = m_imageOne.clone();
    //     img2Teste = m_imageTwo.clone(); 
        
    //     cvtColor(img1Teste, img1Teste, cv::COLOR_BGR2RGB);
    //     cvtColor(img2Teste, img2Teste, cv::COLOR_BGR2RGB);
       
    //     remap(img1Teste, imgU1, map1x, map1y, INTER_LINEAR, BORDER_CONSTANT, Scalar());
    //     remap(img2Teste, imgU2, map2x, map2y, INTER_LINEAR, BORDER_CONSTANT, Scalar());
        
    //     //imshow("image1", img1Teste);
    //     //imshow("image2", img2Teste);
        
    //     //To display the rectified images
    //     imgRectify = Mat::zeros(imgU1.rows, imgU1.cols*2+10, imgU1.type());

    //     imgU1.copyTo(imgRectify(Range::all(), Range(0, imgU2.cols)));
    //     imgU2.copyTo(imgRectify(Range::all(), Range(imgU2.cols+10, imgU2.cols*2+10)));

    //     //If it is too large to fit on the screen, scale down by 2, it should fit.
    //     if(imgRectify.cols > 1920){
    //         resize(imgRectify, imgRectify, Size(imgRectify.cols/2, imgRectify.rows/2));
    //     }
        
    //     //To draw the lines in the rectified image
    //     for(int j = 0; j < imgRectify.rows; j += 16){
    //         Point p1 = Point(0,j);
    //         Point p2 = Point(imgRectify.cols*2,j);
    //         line(imgRectify, p1, p2, CV_RGB(255,0,0));
    //     }

    //     imshow("Rectified", imgRectify);
        
    //     k = waitKey(5);
    //     key = (char) waitKey(5);
        
    //     if(key==27){
    //         break;
    //     }
    // }
}
