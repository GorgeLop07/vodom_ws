#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/path.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <cv_bridge/cv_bridge.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/features2d.hpp>
#include <opencv2/calib3d.hpp>
#include <iostream>
#include <fstream>
#include <vector>
#include <chrono>
#include <iomanip>

class VisualOdometrySimple : public rclcpp::Node
{
public:
    VisualOdometrySimple() : Node("visual_odo_simple"), current_frame_(0), initialized_(false)
    {
        // Initialize ORB detector and BFMatcher (EXACTLY like KITTI version)
        orb = cv::ORB::create(8000);
        matcher = cv::BFMatcher::create(cv::NORM_HAMMING, true);
        
        // Load camera calibration (your calibration)
        loadCameraCalibration();
        
        // Subscribe to camera topic
        image_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
            "/camera/image_raw", 10,
            std::bind(&VisualOdometrySimple::imageCallback, this, std::placeholders::_1));
        
        // Publishers for Visual Odometry (SAME as KITTI version)
        vo_odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>("/odometry/visual", 10);
        vo_path_pub_ = this->create_publisher<nav_msgs::msg::Path>("/vo_path", 10);
        
        // Initialize transform broadcaster for RViz
        tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);
        
        // Initialize pose and path
        current_x_ = 0.0;
        current_y_ = 0.0;
        current_yaw_ = 0.0;
        current_frame_ = 0;
        
        // Initialize path message
        vo_path_msg_.header.frame_id = "odom";
        
        RCLCPP_INFO(this->get_logger(), "🎥 Visual Odometry node initialized");
        RCLCPP_INFO(this->get_logger(), "📥 Subscribed to: /camera/image_raw");
        RCLCPP_INFO(this->get_logger(), "📤 Publishing VO to: /odometry/visual");
        RCLCPP_INFO(this->get_logger(), "🛤️  Publishing path to: /vo_path");
        RCLCPP_INFO(this->get_logger(), "🔗 Publishing transforms: odom->base_link");
        RCLCPP_INFO(this->get_logger(), "⏳ Waiting for camera data...");
    }

private:
    // ROS2 components
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr vo_odom_pub_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr vo_path_pub_;
    std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    
    // OpenCV objects (SAME as KITTI)
    cv::Ptr<cv::ORB> orb;
    cv::Ptr<cv::BFMatcher> matcher;
    cv::Mat K;  // Camera intrinsic matrix
    
    // Current state
    double current_x_, current_y_, current_yaw_;
    size_t current_frame_;
    nav_msgs::msg::Path vo_path_msg_;
    bool initialized_;
    
    // Previous frame data for VO
    cv::Mat prev_image_;
    
    void loadCameraCalibration() {
        // Load YOUR camera calibration
        K = (cv::Mat_<double>(3, 3) << 
            1092.83795872, 0.0, 631.80024334,
            0.0, 1091.39567552, 349.32490162,
            0.0, 0.0, 1.0);
        
        RCLCPP_INFO(this->get_logger(), "✅ Camera calibration loaded");
        RCLCPP_INFO(this->get_logger(), "📷 fx=%.3f, fy=%.3f, cx=%.3f, cy=%.3f", 
                   K.at<double>(0,0), K.at<double>(1,1), K.at<double>(0,2), K.at<double>(1,2));
    }
    
    void imageCallback(const sensor_msgs::msg::Image::SharedPtr msg) {
        RCLCPP_INFO(this->get_logger(), "🎥 Frame %zu: %dx%d", current_frame_, msg->width, msg->height);
        
        // Convert ROS image to OpenCV
        cv_bridge::CvImagePtr cv_ptr;
        try {
            cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
        } catch (cv_bridge::Exception& e) {
            RCLCPP_ERROR(this->get_logger(), "❌ cv_bridge exception: %s", e.what());
            return;
        }
        
        // Convert to grayscale
        cv::Mat current_image;
        cv::cvtColor(cv_ptr->image, current_image, cv::COLOR_BGR2GRAY);
        
        if (!initialized_) {
            // Initialize with first frame
            prev_image_ = current_image.clone();
            initialized_ = true;
            current_frame_++;
            RCLCPP_INFO(this->get_logger(), "✅ Visual odometry initialized with first frame");
            return;
        }
        
        // Process visual odometry using KITTI pipeline
        processFrame(current_image, msg->header.stamp);
        
        // Update previous frame
        prev_image_ = current_image.clone();
        current_frame_++;
    }
    
    void processFrame(const cv::Mat& img_curr, const builtin_interfaces::msg::Time& timestamp) {
        // Skip first few frames for stable initialization (SAME as KITTI)
        if (current_frame_ < 5) {
            publish_odometry(timestamp);
            return;
        }
        
        // Feature detection and matching (EXACT KITTI pipeline)
        std::vector<cv::KeyPoint> kp_prev, kp_curr;
        cv::Mat desc_prev, desc_curr;
        
        orb->detectAndCompute(prev_image_, cv::noArray(), kp_prev, desc_prev);
        orb->detectAndCompute(img_curr, cv::noArray(), kp_curr, desc_curr);
        
        if (desc_prev.empty() || desc_curr.empty()) {
            RCLCPP_WARN(this->get_logger(), "⚠️  No descriptors found in frame %zu", current_frame_);
            return;
        }
        
        // Match features (EXACT KITTI method)
        std::vector<cv::DMatch> matches;
        if (desc_prev.rows >= 2 && desc_curr.rows >= 2) {
            matcher->match(desc_prev, desc_curr, matches);
        }
        
        // Filter matches (EXACT KITTI algorithm)
        std::vector<cv::DMatch> good_matches;
        if (!matches.empty()) {
            auto min_element = std::min_element(matches.begin(), matches.end(),
                [](const cv::DMatch& a, const cv::DMatch& b) {
                    return a.distance < b.distance;
                });
            double min_dist = min_element->distance;
            
            for (const auto& match : matches) {
                if (match.distance <= std::max(2 * min_dist, 30.0)) {
                    good_matches.push_back(match);
                }
            }
        }
        
        // Estimate motion if we have enough matches (EXACT KITTI pipeline)
        if (good_matches.size() >= 8) {
            std::vector<cv::Point2f> pts_prev, pts_curr;
            for (const auto& match : good_matches) {
                pts_prev.push_back(kp_prev[match.queryIdx].pt);
                pts_curr.push_back(kp_curr[match.trainIdx].pt);
            }
            
            // Find essential matrix (EXACT KITTI method)
            cv::Mat mask;
            cv::Mat E = cv::findEssentialMat(pts_prev, pts_curr, K, cv::RANSAC, 0.999, 1.0, mask);
            
            if (!E.empty()) {
                cv::Mat R, t;
                int inliers = cv::recoverPose(E, pts_prev, pts_curr, K, R, t, mask);
                
                if (inliers > 10) {
                    // Use fixed scale instead of ground truth (simple but stable)
                    double scale = 0.1; // Conservative scale for real-time
                    
                    // Apply scale and transform to world coordinates (EXACT KITTI)
                    double dx = scale * t.at<double>(0);
                    double dz = scale * t.at<double>(2);
                    
                    // Extract yaw from rotation matrix (EXACT KITTI method)
                    double dyaw = -atan2(R.at<double>(2, 0), R.at<double>(0, 0));
                    
                    // Filter out excessive rotations (EXACT KITTI approach)
                    if (abs(dyaw) > 0.05) { // Limit to ~3 degrees per frame
                        dyaw = dyaw > 0 ? 0.05 : -0.05; // Clamp instead of zero
                        RCLCPP_WARN(this->get_logger(), "Frame %zu: Clamped excessive rotation", current_frame_);
                    }
                    
                    // Transform relative motion to world coordinates (EXACT KITTI)
                    // current_x_ += dx * cos(current_yaw_) - dz * sin(current_yaw_); //Esto puede no ser correcto xd
                    // current_y_ += dx * sin(current_yaw_) + dz * cos(current_yaw_);
                    current_x_ += -dz * cos(current_yaw_) + dx * sin(current_yaw_);  // Forward = dz
                    current_y_ += -dz * sin(current_yaw_) - dx * cos(current_yaw_);  // Lateral = dx
                    current_yaw_ += dyaw;
                    
                    RCLCPP_INFO(this->get_logger(), "🚗 Frame %zu: matches=%zu, inliers=%d, scale=%.3f, dx=%.3f, dz=%.3f, dyaw=%.3f°", 
                                current_frame_, good_matches.size(), inliers, scale, dx, dz, dyaw*57.3);
                }
            }
        }
        
        // Publish odometry
        publish_odometry(timestamp);
    }
    
    void publish_odometry(const builtin_interfaces::msg::Time& timestamp) {
        // Create odometry message (EXACT KITTI version)
        auto odom_msg = nav_msgs::msg::Odometry();
        odom_msg.header.stamp = timestamp;
        odom_msg.header.frame_id = "odom";
        odom_msg.child_frame_id = "base_link";
        
        // Set position
        odom_msg.pose.pose.position.x = current_x_;
        odom_msg.pose.pose.position.y = current_y_;
        odom_msg.pose.pose.position.z = 0.0;
        
        // Set orientation (from yaw)
        tf2::Quaternion q;
        q.setRPY(0, 0, current_yaw_);
        odom_msg.pose.pose.orientation = tf2::toMsg(q);
        
        // Set covariance (EXACT KITTI values)
        for (int i = 0; i < 36; i++) {
            odom_msg.pose.covariance[i] = 0.0;
        }
        odom_msg.pose.covariance[0] = 0.1;   // x
        odom_msg.pose.covariance[7] = 0.1;   // y
        odom_msg.pose.covariance[35] = 0.1;  // yaw
        
        vo_odom_pub_->publish(odom_msg);
        
        // Publish transform for RViz visualization
        geometry_msgs::msg::TransformStamped transform;
        transform.header.stamp = timestamp;
        transform.header.frame_id = "odom";
        transform.child_frame_id = "base_link";
        transform.transform.translation.x = current_x_;
        transform.transform.translation.y = current_y_;
        transform.transform.translation.z = 0.0;
        transform.transform.rotation = tf2::toMsg(q);
        tf_broadcaster_->sendTransform(transform);
        
        // Add to path and publish (EXACT KITTI method)
        geometry_msgs::msg::PoseStamped pose_stamped;
        pose_stamped.header = odom_msg.header;
        pose_stamped.pose = odom_msg.pose.pose;
        vo_path_msg_.poses.push_back(pose_stamped);
        vo_path_msg_.header.stamp = timestamp;
        
        vo_path_pub_->publish(vo_path_msg_);
        
        RCLCPP_INFO(this->get_logger(), "📡 Position: (%.2f, %.2f) Yaw: %.1f° | Path: %zu poses", 
                   current_x_, current_y_, current_yaw_*57.3, vo_path_msg_.poses.size());
    }
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<VisualOdometrySimple>();
    
    RCLCPP_INFO(node->get_logger(), "==============================================");
    RCLCPP_INFO(node->get_logger(), "🎥 KITTI Visual Odometry Pipeline - Real Time");
    RCLCPP_INFO(node->get_logger(), "📥 Input: /camera/image_raw");
    RCLCPP_INFO(node->get_logger(), "📤 Output: /odometry/visual");
    RCLCPP_INFO(node->get_logger(), "🛤️  Output: /vo_path");
    RCLCPP_INFO(node->get_logger(), "🔧 Using EXACT KITTI vision pipeline");
    RCLCPP_INFO(node->get_logger(), "==============================================");
    
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
