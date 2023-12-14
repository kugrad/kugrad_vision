#include "ros/ros.h"

#include "estimation_coord/estCoord.h"

int main(int argc, char* argv[]) {

    ros::init(argc, argv, "coord_estmation_node");

    EstimationCoord est;

    ros::spin();

    return 0;
}