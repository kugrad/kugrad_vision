#include <ros/ros.h>
#include <image_transport/image_transport.h>

#include "utils.h"

#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>

#include <thread>
#include <mutex>
#include <chrono>

// #include <boost/date_time/posix_time/posix_time.pp>

#if !TEST
#include "stereo_stream/CamConfig.h"
#endif /* TEST */

int main(int argc, char* argv[]) {

    ros::init(argc, argv, "image_capture_node");

    std::string left_camera_fd, right_camera_fd;
    int fps = 30; // default fps 30hz
    int cam_width = 0;
    int cam_height = 0;

#if !TEST
    {
        std::unique_ptr<CamConfig> cam_config(new CamConfig(CONFIG_DIR_PATH "cam_config.json"));

        fps = cam_config->camFps();
        cam_width = cam_config->camWidth();
        cam_height = cam_config->camHeight();

        left_camera_fd = cam_config->leftCameraFdIdx();
        right_camera_fd = cam_config->rightCameraFdIdx();

        // release cam_config here
    }
#endif /* !TEST */

    /**
     * #brief   ros initialization for publisher
     * 
    */
    ros::NodeHandle nh;


    // create publisher left right image publisher
    image_transport::ImageTransport it(nh);
    image_transport::Publisher left_img_pub = it.advertise("stereo/left_image", 10);
    image_transport::Publisher right_img_pub = it.advertise("stereo/right_image", 10);

#if TEST
    cv::VideoCapture stream_l(DATA_DIR_PATH "left.mp4");
    cv::VideoCapture stream_r(DATA_DIR_PATH "right.mp4");

    // cam_width = stream_l.get(cv::CAP_PROP_FRAME_WIDTH);
    // cam_height = stream_l.get(cv::CAP_PROP_FRAME_HEIGHT);
    // fps = stream_l.get(cv::CAP_PROP_FPS);
    // fps = 30;
    cam_width = 640;
    cam_height = 480;
    fps = 30;

    stream_l.set(cv::CAP_PROP_FRAME_WIDTH, cam_width);
    stream_l.set(cv::CAP_PROP_FRAME_HEIGHT, cam_height);
    stream_l.set(cv::CAP_PROP_FPS, fps);
    stream_r.set(cv::CAP_PROP_FRAME_WIDTH, cam_width);
    stream_r.set(cv::CAP_PROP_FRAME_HEIGHT, cam_height);
    stream_r.set(cv::CAP_PROP_FPS, fps);
#else
    cv::VideoCapture stream_l(left_camera_fd, cv::CAP_V4L2);
    cv::VideoCapture stream_r(right_camera_fd, cv::CAP_V4L2);

    stream_l.set(cv::CAP_PROP_FRAME_WIDTH, cam_width);
    stream_l.set(cv::CAP_PROP_FRAME_HEIGHT, cam_height);
    stream_l.set(cv::CAP_PROP_FPS, fps);
    stream_r.set(cv::CAP_PROP_FRAME_WIDTH, cam_width);
    stream_r.set(cv::CAP_PROP_FRAME_HEIGHT, cam_height);
    stream_r.set(cv::CAP_PROP_FPS, fps);
#endif /* TEST */

    if (!stream_l.isOpened() || !stream_r.isOpened()) {
        alert::critic_runtime_error("VideoCapture is not opened! left or right - image_cap_node");
    }

    // boost::posix_time::ptime epoch(boost::gregorian::date(1970, 1, 1)); // The epoch time

    sensor_msgs::ImagePtr left_msg;
    sensor_msgs::ImagePtr right_msg;;

    ros::Rate fps_rate(fps); // 30 fps => 30hz

    while (ros::ok()) {

        cv::Mat image_left, image_right;
        stream_l >> image_left;
        stream_r >> image_right;

        if (image_left.empty()) {
            alert::critic_runtime_error("LEFT Image is not readed");
        }

        if (image_right.empty()) {
            alert::critic_runtime_error("RIGHT Image is not readed");
        }

        left_msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", image_left).toImageMsg();
        right_msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", image_right).toImageMsg();

        left_img_pub.publish(left_msg);
        right_img_pub.publish(right_msg);

        ros::spinOnce();
        fps_rate.sleep();
    }

    // std::thread left_image_stream_thread([&]() {

    //     cv::Mat image_left;

    //     while (ros::ok()) {
    //         // ROS_INFO("stream left_continue\n");
    //         stream_l >> image_left;

    //         // cv::imshow("image_left", image_left);

    //         if (image_left.empty()) {
    //             alert::critic_runtime_error("LEFT Image is not readed");
    //         }

    //         left_msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", image_left).toImageMsg();
    //         left_img_pub.publish(left_msg);

    //         // cv::waitKey(1);
    //         ros::spinOnce();
    //         fps_rate.sleep();
    //     }

    // });

    // std::thread right_image_stream_thread([&]() {

    //     cv::Mat image_right;

    //     while (ros::ok()) {
    //         // ROS_INFO("stream right_continue\n");
    //         stream_r >> image_right;

    //         // cv::imshow("image_right", image_right);

    //         if (image_right.empty()) {
    //             alert::critic_runtime_error("LEFT Image is not readed");
    //         }

    //         right_msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", image_right).toImageMsg();
    //         right_img_pub.publish(right_msg);

    //         // cv::waitKey(1);
    //         ros::spinOnce();
    //         fps_rate.sleep();
    //     }

    // });

    // ros::spin();

    return 0;
}