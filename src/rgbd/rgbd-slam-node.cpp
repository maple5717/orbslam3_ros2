#include "rgbd-slam-node.hpp"
#include "sophus/se3.hpp"
#include <opencv2/core/core.hpp>


using std::placeholders::_1;

RgbdSlamNode::RgbdSlamNode(ORB_SLAM3::System* pSLAM)
:   Node("ORB_SLAM3_ROS2"),
    m_SLAM(pSLAM)
{
    // rgb_sub = std::make_shared<message_filters::Subscriber<ImageMsg> >(shared_ptr<rclcpp::Node>(this), "camera/rgb");
    // depth_sub = std::make_shared<message_filters::Subscriber<ImageMsg> >(shared_ptr<rclcpp::Node>(this), "camera/depth");
    odom_pub = this->create_publisher<nav_msgs::msg::Odometry>("odometry", 10);
    message_filters::Subscriber<ImageMsg>* rgb_sub = new message_filters::Subscriber<ImageMsg>(this, "/camera/camera/color/image_raw");;
    message_filters::Subscriber<ImageMsg>* depth_sub = new message_filters::Subscriber<ImageMsg>(this, "/camera/camera/depth/image_rect_raw");

    syncApproximate = std::make_shared<message_filters::Synchronizer<approximate_sync_policy> >(approximate_sync_policy(10), *rgb_sub, *depth_sub);
    syncApproximate->registerCallback(&RgbdSlamNode::GrabRGBD, this);

}

RgbdSlamNode::~RgbdSlamNode()
{   
    // delete rgb_sub, depth_sub;
    // Stop all threads
    m_SLAM->Shutdown();

    // Save camera trajectory
    m_SLAM->SaveKeyFrameTrajectoryTUM("KeyFrameTrajectory.txt");
}

void RgbdSlamNode::GrabRGBD(const ImageMsg::SharedPtr msgRGB, const ImageMsg::SharedPtr msgD)
{
    // Copy the ros rgb image message to cv::Mat.
    try
    {
        cv_ptrRGB = cv_bridge::toCvShare(msgRGB);
    }
    catch (cv_bridge::Exception& e)
    {
        RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
        return;
    }

    // Copy the ros depth image message to cv::Mat.
    try
    {
        cv_ptrD = cv_bridge::toCvShare(msgD);
    }
    catch (cv_bridge::Exception& e)
    {
        RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
        return;
    }

    Sophus::SE3f pose = m_SLAM->TrackRGBD(cv_ptrRGB->image, cv_ptrD->image, Utility::StampToSec(msgRGB->header.stamp));
    
    geometry_msgs::msg::PoseStamped pose_stamped;
    nav_msgs::msg::Odometry odometry;
    
    // Set position (translation)
    pose_stamped.pose.position.x = pose.translation()[0];
    pose_stamped.pose.position.y = pose.translation()[1];
    pose_stamped.pose.position.z = pose.translation()[2];
    
    // Set orientation (rotation)
    Eigen::Quaternionf quaternion(pose.unit_quaternion());
    pose_stamped.pose.orientation.x = quaternion.x();
    pose_stamped.pose.orientation.y = quaternion.y();
    pose_stamped.pose.orientation.z = quaternion.z();
    pose_stamped.pose.orientation.w = quaternion.w();

    // Header
    odometry.header = pose_stamped.header;
    odometry.pose.pose = pose_stamped.pose;
    
    // Twist (optional, set if velocity information is available)
    // odometry.twist.twist = ...;
    
    odom_pub->publish(odometry);
}
