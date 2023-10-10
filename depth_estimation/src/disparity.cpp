#include "ros/ros.h"

#include "disparity/ReadStereoFS.h"
#include "disparity/DispMap.h"
#include "cam_calrec/CalibConfig.h"

#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/highgui.hpp>

#include <vector>

// #include "stream_image_communication/Stereo_image_msg.h"

// int main(int argc, char* argv[]) {

//     ros::init(argc, argv, "disparity_map");

//     /**
//      * @brief   disparity map initialization
//      * 
//      */
//     ReadStereoFS config_fs(CONFIG_DIR_PATH "caminfo_storage.yaml");
//     // CamConfig config_cam(CONFIG_DIR_PATH "cam_configuration.json");
//     // DispMap process(config_cam, config_fs);

//     /**
//      * #brief   ros initialization for publisher
//      * 
//     */
//     ros::NodeHandle nh;
//     ros::Publisher disparity_map_pub = nh.advertise<std_msgs::String>("disparity_map", 1000);
//     // ros::Rate fps_rate(config_cam.camFps()); // 30 fps => 30hz

//     while (ros::ok()) {
//         std_msgs::String disparity_map_bin;

//         /* ---------- START make disparity map process ---------- */

//         // process.readImageFromStream();

//         // TODO process disparity map 


//         /* ----------  END  make disparity map process ---------- */

//         // disparity_map_bin.data =  // insert data

//         disparity_map_pub.publish(disparity_map_bin);

//         ros::spinOnce();
//         // fps_rate.sleep();
//     }

//     return 0;
// }

// void stereoImageCallback(const stream_image_communication::Stereo_image_msg::ConstPtr& msg) {

//     int width = msg->width;
//     int height = msg->height;
    // std::string encoding_type = msg->encoding_type;

    // std::vector<uint8_t> l_buffer(msg->left_image_string.begin(), msg->left_image_string.end());
    // std::vector<uint8_t> r_buffer(msg->right_image_string.begin(), msg->right_image_string.end());

    // cv::Mat l_raw(1, msg->left_image_string.size(), msg->left_image_string.data());
    // cv::Mat r_raw(1, msg->right_image_string.size(), msg->right_image_string.data());

    // cv::Mat l_img = cv::imdecode(l_buffer, cv::IMREAD_COLOR);
    // cv::Mat r_img = cv::imdecode(r_buffer, cv::IMREAD_COLOR);

    // cv::Mat l_img(cv::Size(width, height), CV_8UC3, (void*) msg->left_image_string.data());
    // cv::Mat r_img(cv::Size(width, height), CV_8UC3, (void*) msg->right_image_string.data());
    // cv::Mat l_img = (cv::Mat(width, height, CV_8UC3) << msg->left_image_string.data());
    // cv::Mat l_img(width, height, CV_8UC3);
    // l_img << msg->left_image_string.data();


    // cv::imshow("left", l_img);
    // cv::imshow("right", l_img);

// }

int main(int argc, char **argv)
{
  ros::init(argc, argv, "disparity_map_generator");

  ros::NodeHandle nh;

  // ros::Subscriber sub = nh.subscribe("stereo_image", 1000, stereoImageCallback);

  /**
   * ros::spin() will enter a loop, pumping callbacks.  With this version, all
   * callbacks will be called from within this thread (the main one).  ros::spin()
   * will exit when Ctrl-C is pressed, or the node is shutdown by the master.
   */
  ros::spin();

  return 0;
}