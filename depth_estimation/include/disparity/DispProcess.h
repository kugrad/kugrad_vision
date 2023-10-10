#ifndef __DISP_PROCESS_H__
#define __DISP_PROCESS_H__

#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <message_filters/synchronizer.h>

#define USE_IMAGE_TRANSPORT_SUBSCRIBER_FILTER 1
#define EXACT_TIME_SYNC 1

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

class DispProcess {

public:
    DispProcess();
    ~DispProcess() = default;

    // * Implemented what we do
    void processCallback(
        const sensor_msgs::ImageConstPtr& left_image_,
        const sensor_msgs::ImageConstPtr& right_image_
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

};

#endif