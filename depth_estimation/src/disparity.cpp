#include "disparity/ReadStereoFS.h"

#include "ros/ros.h"
#include "std_msgs/String.h"

int main(int argc, char* argv[]) {

    ros::init(argc, argv, "disparity_map");

    /**
     * #brief   ros initialization for publisher
     * 
    */
    ros::NodeHandle nh;
    ros::Publisher disparity_map_pub = nh.advertise<std::string>("disparity_map", 1000);
    ros::Rate fps_rate(30); // 30 fps => 30hz

    /**
     * @brief   disparity map initialization
     * 
     */
    ReadStereoFS config_fs(CONFIG_DIR_PATH "caminfo_storage.yaml");



    while (ros::ok()) {
        std_msgs::String disparity_map_bin;

        /* ---------- START make disparity map process ---------- */



        /* ----------  END  make disparity map process ---------- */

        // disparity_map_bin.data =  // insert data

        disparity_map_pub.publish(disparity_map_bin);

        ros::spinOnce();
        fps_rate.sleep();
    }

    return 0;
}