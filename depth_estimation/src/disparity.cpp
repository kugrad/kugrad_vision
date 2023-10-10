#include <ros/ros.h>

#include "disparity/DispProcess.h"

int main(int argc, char* argv[]) {

  ros::init(argc, argv, "disparity_node");

  DispProcess dp;

  ros::spin();

  return 0;
}
