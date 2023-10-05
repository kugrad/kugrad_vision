#include "ros/ros.h"
#include "std_msgs/String.h"

#include "disparity/ReadStereoFS.h"
#include "disparity/DispMap.h"
#include "cam_calrec/CalibConfig.h"

#include "stream_image_communication/Stereo_image_msg.h"

int main(int argc, char* argv[]) {

    ros::init(argc, argv, "disparity_map");

    /**
     * @brief   disparity map initialization
     * 
     */
    ReadStereoFS config_fs(CONFIG_DIR_PATH "caminfo_storage.yaml");
    CamConfig config_cam(CONFIG_DIR_PATH "cam_configuration.json");
    DispMap process(config_cam, config_fs);

    /**
     * #brief   ros initialization for publisher
     * 
    */
    ros::NodeHandle nh;
    ros::Publisher disparity_map_pub = nh.advertise<std_msgs::String>("disparity_map", 1000);
    ros::Rate fps_rate(config_cam.camFps()); // 30 fps => 30hz

    while (ros::ok()) {
        std_msgs::String disparity_map_bin;

        /* ---------- START make disparity map process ---------- */

        process.readImageFromStream();

        // TODO process disparity map 


        /* ----------  END  make disparity map process ---------- */

        // disparity_map_bin.data =  // insert data

        disparity_map_pub.publish(disparity_map_bin);

        ros::spinOnce();
        fps_rate.sleep();
    }

    return 0;
}