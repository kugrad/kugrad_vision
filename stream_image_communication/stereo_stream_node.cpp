#include "ros/ros.h"

#include "stream_image_communication/Stereo_image_msg.h"
#include "stream_image_communication/CamConfig.h"
#include "utils.h"

#include <opencv2/videoio/videoio.hpp>
#include <boost/date_time/posix_time/posix_time.hpp>


using namespace cv;

int main(int argc, char* argv[]) {

    ros::init(argc, argv, "image_capture_node");

    std::string left_camera_fd, right_camera_fd;
    int fps = 30; // default fps 30hz
    int cam_width = 0;
    int cam_height = 0;

    {
        std::unique_ptr<CamConfig> cam_config(new CamConfig(CONFIG_DIR_PATH "cam_config.json"));

        fps = cam_config->camFps();
        cam_width = cam_config->camWidth();
        cam_height = cam_config->camHeight();

        left_camera_fd = cam_config->leftCameraFdIdx();
        right_camera_fd = cam_config->rightCameraFdIdx();

        // release cam_config here
    }

    /**
     * #brief   ros initialization for publisher
     * 
    */
    ros::NodeHandle nh;

    // publisher stero image
    ros::Publisher stream_stereo =
        nh.advertise<stream_image_communication::Stereo_image_msg>("stereo_image", 1000);

    ros::Rate fps_rate(fps); // 30 fps => 30hz

    VideoCapture stream_l(left_camera_fd, CAP_V4L2);
    VideoCapture stream_r(right_camera_fd, CAP_V4L2);

    if (!stream_l.isOpened() || !stream_r.isOpened()) {
        alert::critic_runtime_error("VideoCapture is not opened! left or right - image_cap_node");
    }

    boost::posix_time::ptime epoch(boost::gregorian::date(1970, 1, 1)); // The epoch time

    Mat image_left;
    Mat image_right;

    while (ros::ok()) {

        stream_image_communication::Stereo_image_msg msg;

        /* read image */
        stream_l.read(image_left);
        stream_r.read(image_right);

        auto now = boost::posix_time::microsec_clock::universal_time();

        uint64_t cur_time_in_posix = (now - epoch).total_milliseconds();

        msg.left_image_string << image_left;
        msg.right_image_string << image_right;
        msg.time_info = cur_time_in_posix;

        stream_stereo.publish(msg);

        ros::spinOnce();
        fps_rate.sleep();
    }

    return 0;
}