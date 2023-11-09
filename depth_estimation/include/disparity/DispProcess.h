#ifndef __DISP_PROCESS_H__
#define __DISP_PROCESS_H__

#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <message_filters/synchronizer.h>
#include "geometry_msgs/Pose2D.h"

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

#include <opencv2/core/mat.hpp>

#include "disparity/DispMap.h"
#include "disparity/ReadStereoFS.h"

#include <vector>

class DispProcess {

public:
    DispProcess();
    ~DispProcess() = default;

    // * Implemented what we do
    void processCallback(
        const sensor_msgs::ImageConstPtr& left_image_,
        const sensor_msgs::ImageConstPtr& right_image_
    );

    void imageCoordCallback(
        const geometry_msgs::Pose2D::ConstPtr& image_coord
    );

private:

    typedef struct {
        uint32_t x;
        uint32_t y;
        double distance;
    } CORNER_INFO;

    static void windowMouseCallback(int event, int x, int y, int flag, void* params);
    
    const double calculateTheta() const;

    ros::NodeHandle nh;
    image_transport::ImageTransport imgTrans;

#if USE_IMAGE_TRANSPORT_SUBSCRIBER_FILTER
    typedef image_transport::SubscriberFilter ImageSubscriber;
#else
    typedef message_filters::Subscriber< sensor_msgs::Image > ImageSubscriber;
#endif /* USE_IMAGE_TRANSPORT_SUBSCRIBER_FILTER */
    ImageSubscriber left_image_sub;
    ImageSubscriber right_image_sub;
    ros::Subscriber image_coordinate_sub;

#if EXACT_TIME_SYNC
    typedef message_filters::sync_policies::ExactTime<sensor_msgs::Image, sensor_msgs::Image> SyncPolicy;
#else
    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image> SyncPolicy;
#endif /* EXACT_TIME_SYNC */
    message_filters::Synchronizer<SyncPolicy> sync;

    static constexpr double baseline = 0.0375;

    cv::Mat left_image; // left image
    cv::Mat right_image; // right image

    // ReadStereoFS* config_fs;
    // DispMap* disp;
    std::shared_ptr<ReadStereoFS> config_fs;
    std::shared_ptr<DispMap> disp;

    std::vector<CORNER_INFO> corner_info;

};

#endif