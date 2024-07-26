#ifndef __RGBD_SLAM_NODE_HPP__
#define __RGBD_SLAM_NODE_HPP__

#include <iostream>
#include <algorithm>
#include <fstream>
#include <chrono>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/msg/imu.hpp"

#include "message_filters/subscriber.h"
#include "message_filters/synchronizer.h"
#include "message_filters/sync_policies/approximate_time.h"

#include <cv_bridge/cv_bridge.h>

#include "System.h"
#include "Frame.h"
#include "Map.h"
#include "Tracking.h"

#include "utility.hpp"

#include "nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp" 

class RgbdInertialNode : public rclcpp::Node
{
public:
    RgbdInertialNode(ORB_SLAM3::System* pSLAM);
    

    ~RgbdInertialNode();

private:
    using ImuMsg = sensor_msgs::msg::Imu;
    using ImageMsg = sensor_msgs::msg::Image;
    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::msg::Image, sensor_msgs::msg::Image> approximate_sync_policy;
    int img_queue_size_max = 1000; 
    void GrabRGBD(const sensor_msgs::msg::Image::SharedPtr msgRGB, const sensor_msgs::msg::Image::SharedPtr msgD);
    cv::Mat GetImage(const ImageMsg::SharedPtr msg);
    void GrabImu(const ImuMsg::SharedPtr msg);
    void SyncWithImu();
    void Track(double tTrack);

    ORB_SLAM3::System* m_SLAM;
    std::thread *syncThread_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub;

    cv_bridge::CvImageConstPtr cv_ptrRGB;
    cv_bridge::CvImageConstPtr cv_ptrD;

    

    // IMU
    rclcpp::Subscription<ImuMsg>::SharedPtr   subImu_;
    queue<ImuMsg::SharedPtr> imuBuf_;
    std::mutex bufMutex_;

    // RGBD
    std::shared_ptr<message_filters::Subscriber<sensor_msgs::msg::Image> > rgb_sub;
    std::shared_ptr<message_filters::Subscriber<sensor_msgs::msg::Image> > depth_sub;
    std::shared_ptr<message_filters::Synchronizer<approximate_sync_policy> > syncApproximate;
    queue<ImageMsg::SharedPtr> imgRgbBuf_, imgDepthBuf_;
    std::mutex bufMutexRgb_, bufMutexDepth_;
};

#endif
