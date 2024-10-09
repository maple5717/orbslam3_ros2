#include "rgbd-inertial-node.hpp"
#include "sophus/se3.hpp"
#include <opencv2/core/core.hpp>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <cmath>

using std::placeholders::_1;

RgbdInertialNode::RgbdInertialNode(ORB_SLAM3::System* pSLAM)
:   Node("ORB_SLAM3_ROS2"),
    m_SLAM(pSLAM)
{
    // rgb_sub = std::make_shared<message_filters::Subscriber<ImageMsg> >(shared_ptr<rclcpp::Node>(this), "camera/rgb");
    // depth_sub = std::make_shared<message_filters::Subscriber<ImageMsg> >(shared_ptr<rclcpp::Node>(this), "camera/depth");
    odom_pub = this->create_publisher<nav_msgs::msg::Odometry>("odometry", 10);
    message_filters::Subscriber<ImageMsg>* rgb_sub = new message_filters::Subscriber<ImageMsg>(this, "/camera/camera/color/image_raw");;
    message_filters::Subscriber<ImageMsg>* depth_sub = new message_filters::Subscriber<ImageMsg>(this, "/camera/camera/aligned_depth_to_color/image_raw");

    syncApproximate = std::make_shared<message_filters::Synchronizer<approximate_sync_policy> >(approximate_sync_policy(10), *rgb_sub, *depth_sub);
    syncApproximate->registerCallback(&RgbdInertialNode::GrabRGBD, this);

    subImu_ = this->create_subscription<ImuMsg>("/camera/camera/imu", 1000, std::bind(&RgbdInertialNode::GrabImu, this, _1));

    syncThread_ = new std::thread(&RgbdInertialNode::SyncWithImu, this);
}

RgbdInertialNode::~RgbdInertialNode()
{   
    // delete rgb_sub, depth_sub;
    // Stop all threads
    m_SLAM->Shutdown();

    // Save camera trajectory
    m_SLAM->SaveKeyFrameTrajectoryTUM("KeyFrameTrajectory.txt");
}

cv::Mat RgbdInertialNode::GetImage(const ImageMsg::SharedPtr msg)
{
    // Copy the ros image message to cv::Mat.
    cv_bridge::CvImageConstPtr cv_ptr;

    try
    {
        cv_ptr = cv_bridge::toCvShare(msg);
    }
    catch (cv_bridge::Exception &e)
    {
        RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
    }

    // if (cv_ptr->image.type() == 0)
    // {
    //     return cv_ptr->image.clone();
    // }
    // else
    // {
    //     std::cerr << "Error image type" << std::endl;
    //     return cv_ptr->image.clone();
    // }
    return cv_ptr->image.clone();
}

void RgbdInertialNode::GrabRGBD(const ImageMsg::SharedPtr msgRGB, const ImageMsg::SharedPtr msgD)
{
    // save the ros rgb image message to queue.
    
    // std::cout  << "rgb " << imgRgbBuf_.size() << std::endl;
    // std::cout  << "depth " << imgDepthBuf_.size() << std::endl;
    bufMutexRgb_.lock();
    if (imgRgbBuf_.size() > img_queue_size_max)
    {
        imgRgbBuf_.pop();
        imgDepthBuf_.pop();
    }
    imgRgbBuf_.push(msgRGB);

    bufMutexRgb_.unlock();

    // save the ros depth image message to queue.
    bufMutexDepth_.lock();

    // if (!imgDepthBuf_.empty())
    //     imgDepthBuf_.pop();
    imgDepthBuf_.push(msgD);

    bufMutexDepth_.unlock();

    
}

void RgbdInertialNode::GrabImu(const ImuMsg::SharedPtr msg)
{   
    bufMutex_.lock();
    imuBuf_.push(msg);
    bufMutex_.unlock();
}

void RgbdInertialNode::Track(double tTrack)
{   
    // load rgb img from buffer
    cv::Mat imRgb, imDepth;
    
    bufMutexRgb_.lock();
    imRgb = GetImage(imgRgbBuf_.front());
    imgRgbBuf_.pop();
    bufMutexRgb_.unlock();

    bufMutexDepth_.lock();
    imDepth = GetImage(imgDepthBuf_.front());
    imgDepthBuf_.pop();
    bufMutexDepth_.unlock();

    vector<ORB_SLAM3::IMU::Point> vImuMeas;
    bufMutex_.lock();
    if (!imuBuf_.empty())
    {
        // Load imu measurements from buffer
        vImuMeas.clear();
        while (!imuBuf_.empty() && Utility::StampToSec(imuBuf_.front()->header.stamp) <= tTrack)
        {
            double t = Utility::StampToSec(imuBuf_.front()->header.stamp);
            cv::Point3f acc(imuBuf_.front()->linear_acceleration.x, imuBuf_.front()->linear_acceleration.y, imuBuf_.front()->linear_acceleration.z);
            cv::Point3f gyr(imuBuf_.front()->angular_velocity.x, imuBuf_.front()->angular_velocity.y, imuBuf_.front()->angular_velocity.z);
            vImuMeas.push_back(ORB_SLAM3::IMU::Point(acc, gyr, t));
            imuBuf_.pop();
        }
    }
    bufMutex_.unlock();
    
    // if (bClahe_)
    // {
    //     clahe_->apply(imLeft, imLeft);
    //     clahe_->apply(imRight, imRight);
    // }

    // if (doRectify_)
    // {
    //     cv::remap(imLeft, imLeft, M1l_, M2l_, cv::INTER_LINEAR);
    //     cv::remap(imRight, imRight, M1r_, M2r_, cv::INTER_LINEAR);
    // }


    Sophus::SE3f pose = m_SLAM->TrackRGBD(imRgb, imDepth, tTrack, vImuMeas);
    rclcpp::Time current_time = this->now();
    geometry_msgs::msg::PoseStamped pose_stamped;
    nav_msgs::msg::Odometry odometry;

    // Transformation
    // TODO: put this into a func
    Eigen::Matrix3d R_body_to_camera; // Rotation matrix (body to camera)
    Eigen::Vector3d t_body_to_camera; // Translation vector (body to camera)

    // // Rotation matrix (from body frame to camera frame)
    // R_body_to_camera << 0., 0., 1.,
    //                     -1., 0., 0.,
    //                      0., -1., 0.;

    // // Translation vector (from body frame to camera frame) 
    // t_body_to_camera << 0., 0., 1.4;

    // Sophus::SE3f T_body_to_camera(Sophus::SO3f(R_body_to_camera.cast<float>()), t_body_to_camera.cast<float>());

    // // Compute the pose in the body frame
    // pose = T_body_to_camera.inverse() * pose;

    Eigen::Vector3f translation = pose.translation();
    Eigen::Quaternionf quaternion_p(pose.unit_quaternion());

    // Convert quaternion to a rotation matrix (SO(3))
    Eigen::Matrix3f rotation_matrix = quaternion_p.toRotationMatrix();

    // Compute the inverse of the rotation matrix
    Eigen::Matrix3f inverse_rotation_matrix = rotation_matrix.inverse();

    // Transform the translation using the inverse rotation matrix
    Eigen::Vector3f transformed_translation = inverse_rotation_matrix * translation;


    double dt = (current_time - prev_time).seconds();
    Eigen::Vector3f vel_linear = (translation - prev_translation) / dt; 

    Eigen::Quaternionf delta_quat = quaternion_p * prev_quaternion.inverse();
    Eigen::Vector3f vel_rot = delta_quat.toRotationMatrix().eulerAngles(0, 1, 2)/ dt;
    
    // Note: The euler angle only covers y component
    double y_rot = atan2(delta_quat.y(), delta_quat.w());
    // std::cout << delta_quat.toRotationMatrix() << std::endl << std::endl;
    // std::cout << delta_quat.toRotationMatrix().eulerAngles(0, 1, 2) << std::endl << std::endl;


    prev_time = current_time; 
    prev_translation = translation;
    prev_quaternion = quaternion_p; 



    
    // Set position (translation)
    pose_stamped.pose.position.x = -transformed_translation[0]; // -pose.translation()[0];
    pose_stamped.pose.position.y = transformed_translation[1]; // pose.translation()[1];
    pose_stamped.pose.position.z = -transformed_translation[2]; // -pose.translation()[2];
    
    // Set orientation (rotation)
    Eigen::Quaternionf quaternion(pose.unit_quaternion());
    pose_stamped.pose.orientation.x = quaternion.x();
    pose_stamped.pose.orientation.y = quaternion.y();
    pose_stamped.pose.orientation.z = quaternion.z();
    pose_stamped.pose.orientation.w = -quaternion.w();

    odometry.twist.twist.angular.x = 0;// -vel_rot[0];
    odometry.twist.twist.angular.y = -y_rot / dt;// -vel_rot[1];
    odometry.twist.twist.angular.z = 0;// -vel_rot[2];

    odometry.twist.twist.linear.x = -vel_linear[0];
    odometry.twist.twist.linear.y = vel_linear[1];
    odometry.twist.twist.linear.z = -vel_linear[2];

    // Header
    odometry.header = pose_stamped.header;
    odometry.pose.pose = pose_stamped.pose;
    odometry.header.frame_id = "camera_color_optical_frame";
    // odometry.child_frame_id = "base_link";
    
    // Twist (optional, set if velocity information is available)
    // odometry.twist.twist = ...;
    odometry.header.stamp = this->now();
    odom_pub->publish(odometry);
}

void RgbdInertialNode::SyncWithImu()
{
    const double maxTimeDiff = 0.01;

    while (1)
    {
        double tRgbd = 0, tDepth = 0; 
        // std::cout << imgRgbBuf_.empty() << imgDepthBuf_.empty() << imuBuf_.empty() << std::endl;
        if (!imgRgbBuf_.empty() && !imgDepthBuf_.empty() && !imuBuf_.empty())
        {   

            tRgbd = Utility::StampToSec(imgRgbBuf_.front()->header.stamp);
            tDepth = Utility::StampToSec(imgDepthBuf_.front()->header.stamp);

            // double tMax = std::max(tRgbd, tDepth);
            double tMin = std::min(tRgbd, tDepth);
            
            // wait until enough Imu data is collected
            // if (tMin > Utility::StampToSec(imuBuf_.back()->header.stamp))
            //     continue;

            Track(tMin);

            std::chrono::milliseconds tSleep(1);
            std::this_thread::sleep_for(tSleep);

           

        }
    }
}


