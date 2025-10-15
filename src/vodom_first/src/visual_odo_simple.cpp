#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/path.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <cv_bridge/cv_bridge.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/features2d.hpp>
#include <opencv2/calib3d.hpp>
#include <iostream>
#include <vector>
#include <chrono>

class VisualOdometrySimple : public rclcpp::Node
{
public:
    VisualOdometrySimple() : Node("visual_odo_simple"), frame_count_(0), initialized_(false)
    {
        // ============================
        // CAMERA INTRINSICS AND DISTORTION
        // PLACE YOUR CAMERA PARAMETERS HERE:
        // ============================
        
        // Camera intrinsic matrix (3x3)
        // K = [fx  0  cx]
        //     [ 0 fy  cy]
        //     [ 0  0   1]
        double fx = 718.856;  // YOUR fx VALUE HERE
        double fy = 718.856;  // YOUR fy VALUE HERE  
        double cx = 607.1928; // YOUR cx VALUE HERE
        double cy = 185.2157; // YOUR cy VALUE HERE
        
        K_ = (cv::Mat_<double>(3, 3) << 
            fx, 0.0, cx,
            0.0, fy, cy,
            0.0, 0.0, 1.0);
        
        // Distortion coefficients [k1, k2, p1, p2, k3]
        distortion_coeffs_ = (cv::Mat_<double>(5, 1) << 
            0.0,  // k1 - YOUR k1 VALUE HERE
            0.0,  // k2 - YOUR k2 VALUE HERE  
            0.0,  // p1 - YOUR p1 VALUE HERE
            0.0,  // p2 - YOUR p2 VALUE HERE
            0.0); // k3 - YOUR k3 VALUE HERE
        
        // ============================
        // END CAMERA PARAMETERS
        // ============================
        
        // Initialize ORB detector and BFMatcher
        orb_ = cv::ORB::create(8000);
        matcher_ = cv::BFMatcher::create(cv::NORM_HAMMING, true);
        
        // SOLO CAMARA - sin IMU
        image_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
            "/camera/image_raw", 10,
            std::bind(&VisualOdometrySimple::imageCallback, this, std::placeholders::_1));
        
        // Publishers para VO: odometry + path
        vo_odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>("/odometry/visual", 10);
        vo_path_pub_ = this->create_publisher<nav_msgs::msg::Path>("/vo_path", 10);
        
        // Initialize pose
        current_x_ = 0.0;
        current_y_ = 0.0;
        current_yaw_ = 0.0;
        
        // Initialize path for visualization
        vo_path_.header.frame_id = "odom";
        
        RCLCPP_INFO(this->get_logger(), "Visual Odometry Simple - Camera only");
        RCLCPP_INFO(this->get_logger(), "Camera: fx=%.2f, fy=%.2f, cx=%.2f, cy=%.2f", fx, fy, cx, cy);
        RCLCPP_INFO(this->get_logger(), "🔍 Subscribed to: /camera/image_raw");
        RCLCPP_INFO(this->get_logger(), "📡 Publishing odometry to: /odometry/visual");
        RCLCPP_INFO(this->get_logger(), "🛤️  Publishing path to: /vo_path");
        RCLCPP_INFO(this->get_logger(), "⏳ Waiting for camera data...");
        
        // Log every 5 seconds if no data received
        timer_ = this->create_wall_timer(
            std::chrono::seconds(5),
            std::bind(&VisualOdometrySimple::checkStatus, this));
    }

private:
    // ROS2 subscribers and publishers
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr vo_odom_pub_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr vo_path_pub_;
    rclcpp::TimerBase::SharedPtr timer_;
    
    // Path for visualization
    nav_msgs::msg::Path vo_path_;
    
    // OpenCV objects
    cv::Ptr<cv::ORB> orb_;
    cv::Ptr<cv::BFMatcher> matcher_;
    
    // Camera parameters
    cv::Mat K_;  // Camera intrinsic matrix
    cv::Mat distortion_coeffs_;  // Distortion coefficients
    
    // Current state
    double current_x_, current_y_, current_yaw_;
    cv::Mat previous_frame_;
    std::vector<cv::KeyPoint> previous_keypoints_;
    cv::Mat previous_descriptors_;
    bool initialized_;
    int frame_count_;
    
    // Image callback - processes each incoming camera frame
    void imageCallback(const sensor_msgs::msg::Image::SharedPtr msg)
    {
        RCLCPP_INFO(this->get_logger(), "🎥 Received image %d: %dx%d", frame_count_, msg->width, msg->height);
        
        // Convert ROS image to OpenCV
        cv_bridge::CvImagePtr cv_ptr;
        try {
            cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::MONO8);
            RCLCPP_INFO(this->get_logger(), "✅ Image converted to OpenCV successfully");
        } catch (cv_bridge::Exception& e) {
            RCLCPP_ERROR(this->get_logger(), "❌ cv_bridge exception: %s", e.what());
            return;
        }
        
        cv::Mat current_frame = cv_ptr->image;
        
        // Undistort the image if distortion coefficients are provided
        cv::Mat undistorted_frame;
        cv::undistort(current_frame, undistorted_frame, K_, distortion_coeffs_);
        current_frame = undistorted_frame;
        
        // Process visual odometry
        processVisualOdometry(current_frame, msg->header.stamp);
        
        frame_count_++;
    }
    
    void checkStatus()
    {
        if (frame_count_ == 0) {
            RCLCPP_WARN(this->get_logger(), "⚠️  No camera data received yet. Check:");
            RCLCPP_WARN(this->get_logger(), "   1. Is rosbag playing? ros2 bag play your_bag.bag");
            RCLCPP_WARN(this->get_logger(), "   2. Is topic correct? ros2 topic list | grep camera");
            RCLCPP_WARN(this->get_logger(), "   3. Current topic: /camera/image_raw");
        } else {
            RCLCPP_INFO(this->get_logger(), "✅ System working! Processed %d frames", frame_count_);
        }
    }
    
    // Main visual odometry processing function
    void processVisualOdometry(const cv::Mat& current_frame, const builtin_interfaces::msg::Time& timestamp)
    {
        if (!initialized_) {
            // Initialize with first frame
            previous_frame_ = current_frame.clone();
            
            // Detect features in first frame
            orb_->detectAndCompute(previous_frame_, cv::noArray(), previous_keypoints_, previous_descriptors_);
            
            initialized_ = true;
            RCLCPP_INFO(this->get_logger(), "Visual odometry initialized with first frame");
            return;
        }
        
        // Detect features in current frame
        std::vector<cv::KeyPoint> current_keypoints;
        cv::Mat current_descriptors;
        orb_->detectAndCompute(current_frame, cv::noArray(), current_keypoints, current_descriptors);
        
        if (current_descriptors.empty() || previous_descriptors_.empty()) {
            RCLCPP_WARN(this->get_logger(), "No descriptors found in frame %d", frame_count_);
            updatePreviousFrame(current_frame, current_keypoints, current_descriptors);
            return;
        }
        
        // Match features
        std::vector<cv::DMatch> matches;
        matcher_->match(previous_descriptors_, current_descriptors, matches);
        
        if (matches.size() < 8) {
            RCLCPP_WARN(this->get_logger(), "Insufficient matches (%zu) in frame %d", matches.size(), frame_count_);
            updatePreviousFrame(current_frame, current_keypoints, current_descriptors);
            return;
        }
        
        // Extract matched points
        std::vector<cv::Point2f> prev_pts, curr_pts;
        for (const auto& match : matches) {
            prev_pts.push_back(previous_keypoints_[match.queryIdx].pt);
            curr_pts.push_back(current_keypoints[match.trainIdx].pt);
        }
        
        // Calculate Essential Matrix and recover pose
        cv::Mat E, R, t, mask;
        E = cv::findEssentialMat(prev_pts, curr_pts, K_, cv::RANSAC, 0.999, 1.0, mask);
        
        if (E.empty()) {
            RCLCPP_WARN(this->get_logger(), "Failed to compute essential matrix for frame %d", frame_count_);
            updatePreviousFrame(current_frame, current_keypoints, current_descriptors);
            return;
        }
        
        int inliers = cv::recoverPose(E, prev_pts, curr_pts, K_, R, t, mask);
        
        if (inliers < 8) {
            RCLCPP_WARN(this->get_logger(), "Insufficient inliers (%d) for frame %d", inliers, frame_count_);
            updatePreviousFrame(current_frame, current_keypoints, current_descriptors);
            return;
        }
        
        // Update pose (simple integration)
        double scale = 1.0;
        
        cv::Mat rotation_update = R.t();
        cv::Mat translation_update = -rotation_update * t * scale;
        
        // Update current position
        current_x_ += translation_update.at<double>(0);
        current_y_ += translation_update.at<double>(2);
        
        // Simple yaw from rotation matrix
        double dyaw = atan2(R.at<double>(1, 0), R.at<double>(0, 0));
        current_yaw_ += dyaw;
        
        // Publish odometry
        publishOdometry(timestamp);
        
        // Update previous frame data
        updatePreviousFrame(current_frame, current_keypoints, current_descriptors);
        
        RCLCPP_INFO(this->get_logger(), "🚗 Frame %d: Position (%.2f, %.2f) | Inliers: %d/%zu | Path: %zu poses!", 
                   frame_count_, current_x_, current_y_, inliers, matches.size(), vo_path_.poses.size());
    }
    
    void updatePreviousFrame(const cv::Mat& frame, const std::vector<cv::KeyPoint>& keypoints, const cv::Mat& descriptors)
    {
        previous_frame_ = frame.clone();
        previous_keypoints_ = keypoints;
        previous_descriptors_ = descriptors.clone();
    }
    
    void publishOdometry(const builtin_interfaces::msg::Time& timestamp)
    {
        // Publish Odometry for EKF
        nav_msgs::msg::Odometry odom_msg;
        odom_msg.header.stamp = timestamp;
        odom_msg.header.frame_id = "odom";
        odom_msg.child_frame_id = "base_link";
        
        // Set position
        odom_msg.pose.pose.position.x = current_x_;
        odom_msg.pose.pose.position.y = current_y_;
        odom_msg.pose.pose.position.z = 0.0;
        
        // Set orientation
        tf2::Quaternion q;
        q.setRPY(0, 0, current_yaw_);
        odom_msg.pose.pose.orientation = tf2::toMsg(q);
        
        // Covariance for EKF
        odom_msg.pose.covariance[0] = 0.1;   // x
        odom_msg.pose.covariance[7] = 0.1;   // y
        odom_msg.pose.covariance[35] = 0.1;  // yaw
        
        vo_odom_pub_->publish(odom_msg);
        
        // Publish Path for visualization
        geometry_msgs::msg::PoseStamped pose_stamped;
        pose_stamped.header.stamp = timestamp;
        pose_stamped.header.frame_id = "odom";
        pose_stamped.pose = odom_msg.pose.pose;
        
        vo_path_.poses.push_back(pose_stamped);
        vo_path_.header.stamp = timestamp;
        vo_path_pub_->publish(vo_path_);
    }
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<VisualOdometrySimple>();
    
    RCLCPP_INFO(node->get_logger(), "==============================================");
    RCLCPP_INFO(node->get_logger(), "🎥 Visual Odometry + 🛤️  Path Publisher");
    RCLCPP_INFO(node->get_logger(), "Input: /camera/image_raw");
    RCLCPP_INFO(node->get_logger(), "Output: /odometry/visual (for EKF)");
    RCLCPP_INFO(node->get_logger(), "Output: /vo_path (for visualization)");
    RCLCPP_INFO(node->get_logger(), "==============================================");
    
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}