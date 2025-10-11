#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/path.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <cv_bridge/cv_bridge.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/features2d.hpp>
#include <opencv2/calib3d.hpp>
#include <iostream>
#include <fstream>
#include <vector>
#include <string>
#include <cmath>

class PureVisualOdometry : public rclcpp::Node {
public:
    PureVisualOdometry() : Node("pure_visual_odometry") {
        // Load camera calibration
        if (!loadCameraCalibration()) {
            RCLCPP_ERROR(this->get_logger(), "Failed to load camera calibration");
            return;
        }
        
        // Initialize ORB feature detector and BFMatcher (more robust for ORB)
        orb_ = cv::ORB::create(8000);
        bf_matcher_ = cv::BFMatcher::create(cv::NORM_HAMMING, false); // crossCheck=false for knnMatch
        
        // Publisher for visual odometry path
        path_pub_ = this->create_publisher<nav_msgs::msg::Path>("/vo_path", 10);
        
        // Subscriber for camera images
        image_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
            "/camera/image_raw", 10,
            std::bind(&PureVisualOdometry::imageCallback, this, std::placeholders::_1)
        );
        
        // Initialize path message
        path_msg_.header.frame_id = "map";
        
        // Initialize pose variables
        current_x_ = 0.0;
        current_y_ = 0.0;
        current_yaw_ = 0.0;
        
        // Flags
        first_frame_ = true;
        scale_factor_ = 0.05; // Even smaller scale factor for smoother motion
        
        RCLCPP_INFO(this->get_logger(), "Pure Visual Odometry Node initialized (ORB + BFMatcher)");
        RCLCPP_INFO(this->get_logger(), "Camera parameters - fx: %.3f, fy: %.3f, cx: %.3f, cy: %.3f", 
                   camera_matrix_.at<double>(0,0), camera_matrix_.at<double>(1,1),
                   camera_matrix_.at<double>(0,2), camera_matrix_.at<double>(1,2));
    }

private:
    // ROS2 components
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
    
    // OpenCV components
    cv::Ptr<cv::ORB> orb_;
    cv::Ptr<cv::BFMatcher> bf_matcher_;
    cv::Mat camera_matrix_;
    cv::Mat distortion_coeffs_;
    cv::Mat prev_image_;
    
    // State variables
    nav_msgs::msg::Path path_msg_;
    double current_x_, current_y_, current_yaw_;
    bool first_frame_;
    double scale_factor_;
    
    // Smoothing variables
    std::vector<double> recent_dx_, recent_dy_, recent_dyaw_;
    static constexpr size_t SMOOTHING_WINDOW = 3;
    
    bool loadCameraCalibration() {
        // Load calibration from config file using absolute path
        std::string calib_file = "/home/jorgelop/Documents/VantTec_SDV_SWARM/SDV_Proyect/SDV_Software_Workspaces/vo_ws/src/vodom_first/config/camera_calib.txt";
        std::ifstream file(calib_file);
        
        if (!file.is_open()) {
            RCLCPP_ERROR(this->get_logger(), "Cannot open calibration file: %s", calib_file.c_str());
            return false;
        }
        
        std::vector<double> params;
        std::string line;
        std::getline(file, line);
        
        // Skip comment lines
        while (!line.empty() && line[0] == '#') {
            std::getline(file, line);
        }
        
        std::istringstream iss(line);
        double val;
        while (iss >> val) {
            params.push_back(val);
        }
        
        if (params.size() != 12) {
            RCLCPP_ERROR(this->get_logger(), "Calibration file must have 12 parameters, found %zu", params.size());
            return false;
        }
        
        // Extract camera matrix from calibration matrix (first 3x3)
        camera_matrix_ = cv::Mat::zeros(3, 3, CV_64F);
        camera_matrix_.at<double>(0, 0) = params[0]; // fx
        camera_matrix_.at<double>(0, 1) = params[1]; // 0
        camera_matrix_.at<double>(0, 2) = params[2]; // cx
        camera_matrix_.at<double>(1, 0) = params[4]; // 0
        camera_matrix_.at<double>(1, 1) = params[5]; // fy
        camera_matrix_.at<double>(1, 2) = params[6]; // cy
        camera_matrix_.at<double>(2, 0) = params[8]; // 0
        camera_matrix_.at<double>(2, 1) = params[9]; // 0
        camera_matrix_.at<double>(2, 2) = params[10]; // 1
        
        // Initialize distortion coefficients as zero (assuming calibrated images)
        distortion_coeffs_ = cv::Mat::zeros(5, 1, CV_64F);
        
        return true;
    }
    
    void imageCallback(const sensor_msgs::msg::Image::SharedPtr msg) {
        try {
            // Convert ROS image to OpenCV
            cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
            cv::Mat current_image;
            
            // Convert to grayscale for ORB processing
            cv::cvtColor(cv_ptr->image, current_image, cv::COLOR_BGR2GRAY);
            
            if (first_frame_) {
                // Store first frame
                prev_image_ = current_image.clone();
                first_frame_ = false;
                RCLCPP_INFO(this->get_logger(), "First frame received, starting visual odometry");
                return;
            }
            
            // Process visual odometry
            processVisualOdometry(current_image);
            
            // Update previous frame
            prev_image_ = current_image.clone();
            
        } catch (cv_bridge::Exception& e) {
            RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
        }
    }
    
    void processVisualOdometry(const cv::Mat& current_image) {
        // Detect and match features
        std::vector<cv::Point2f> prev_points, curr_points;
        if (!detectAndMatchFeatures(prev_image_, current_image, prev_points, curr_points)) {
            RCLCPP_DEBUG(this->get_logger(), "Feature matching failed");
            return;
        }
        
        if (prev_points.size() < 15) { // Increased minimum matches for better stability
            RCLCPP_DEBUG(this->get_logger(), "Insufficient feature matches: %zu", prev_points.size());
            return;
        }
        
        // Estimate motion using essential matrix
        cv::Mat motion = estimateMotion(prev_points, curr_points);
        if (motion.empty()) {
            RCLCPP_DEBUG(this->get_logger(), "Motion estimation failed");
            return;
        }
        
        // Extract 2D pose change
        auto [dx, dy, dyaw] = extractPose2D(motion);
        
        // Filter out unrealistic movements and apply smoothing
        if (std::abs(dx) > 0.5 || std::abs(dy) > 0.5 || std::abs(dyaw) > 0.2) { // More strict limits
            RCLCPP_DEBUG(this->get_logger(), "Unrealistic motion detected, skipping: dx=%.3f, dy=%.3f, dyaw=%.3f", dx, dy, dyaw);
            return;
        }
        
        // Apply temporal smoothing
        auto [smooth_dx, smooth_dy, smooth_dyaw] = applySmoothing(dx, dy, dyaw);
        
        // Update current pose
        updatePose(smooth_dx, smooth_dy, smooth_dyaw);
        
        // Publish path
        publishPath();
        
        RCLCPP_INFO(this->get_logger(), "VO Update - Features: %zu, Raw: (%.3f,%.3f,%.1f°) Smooth: (%.3f,%.3f,%.1f°)", 
                    prev_points.size(), dx, dy, dyaw*57.3, smooth_dx, smooth_dy, smooth_dyaw*57.3);
    }
    
    bool detectAndMatchFeatures(const cv::Mat& img1, const cv::Mat& img2, 
                               std::vector<cv::Point2f>& points1, std::vector<cv::Point2f>& points2) {
        // Detect ORB features
        std::vector<cv::KeyPoint> kp1, kp2;
        cv::Mat desc1, desc2;
        
        orb_->detectAndCompute(img1, cv::noArray(), kp1, desc1);
        orb_->detectAndCompute(img2, cv::noArray(), kp2, desc2);
        
        if (desc1.empty() || desc2.empty()) {
            return false;
        }
        
        // Match features using BFMatcher with k=2 for ratio test
        std::vector<std::vector<cv::DMatch>> knn_matches;
        bf_matcher_->knnMatch(desc1, desc2, knn_matches, 2);
        
        // Apply Lowe's ratio test (more robust than distance threshold)
        const float ratio_thresh = 0.75f; // Slightly more lenient
        for (const auto& match : knn_matches) {
            if (match.size() == 2 && match[0].distance < ratio_thresh * match[1].distance) {
                points1.push_back(kp1[match[0].queryIdx].pt);
                points2.push_back(kp2[match[0].trainIdx].pt);
            }
        }
        
        return points1.size() >= 15; // Ensure minimum matches
    }
    
    std::tuple<double, double, double> applySmoothing(double dx, double dy, double dyaw) {
        // Add current measurements to history
        recent_dx_.push_back(dx);
        recent_dy_.push_back(dy);
        recent_dyaw_.push_back(dyaw);
        
        // Limit window size
        if (recent_dx_.size() > SMOOTHING_WINDOW) {
            recent_dx_.erase(recent_dx_.begin());
            recent_dy_.erase(recent_dy_.begin());
            recent_dyaw_.erase(recent_dyaw_.begin());
        }
        
        // Calculate smoothed values (median filter for rotation, average for translation)
        double smooth_dx = 0.0, smooth_dy = 0.0;
        for (double val : recent_dx_) smooth_dx += val;
        for (double val : recent_dy_) smooth_dy += val;
        smooth_dx /= recent_dx_.size();
        smooth_dy /= recent_dy_.size();
        
        // Use median for rotation (more robust against outliers)
        std::vector<double> yaw_copy = recent_dyaw_;
        std::sort(yaw_copy.begin(), yaw_copy.end());
        double smooth_dyaw = yaw_copy[yaw_copy.size() / 2];
        
        // Additional damping for rotation (reduce crazy spins)
        smooth_dyaw *= 0.5; // Reduce rotation by 50%
        
        return {smooth_dx, smooth_dy, smooth_dyaw};
    }
    
    cv::Mat estimateMotion(const std::vector<cv::Point2f>& points1, const std::vector<cv::Point2f>& points2) {
        // Find essential matrix using RANSAC with relaxed parameters
        cv::Mat mask;
        cv::Mat E = cv::findEssentialMat(points1, points2, camera_matrix_, 
                                        cv::RANSAC, 0.99, 2.0, mask); // More lenient threshold
        
        if (E.empty()) {
            return cv::Mat();
        }
        
        // Filter inliers
        std::vector<cv::Point2f> inlier_points1, inlier_points2;
        for (int i = 0; i < mask.rows; ++i) {
            if (mask.at<uchar>(i)) {
                inlier_points1.push_back(points1[i]);
                inlier_points2.push_back(points2[i]);
            }
        }
        
        if (inlier_points1.size() < 10) { // Reduced minimum for more flexibility
            return cv::Mat();
        }
        
        // Recover pose from essential matrix
        cv::Mat R, t;
        int inliers = cv::recoverPose(E, inlier_points1, inlier_points2, camera_matrix_, R, t);
        
        if (inliers < 10) { // Reduced minimum for more flexibility
            return cv::Mat();
        }
        
        // Create transformation matrix
        cv::Mat T = cv::Mat::eye(4, 4, CV_64F);
        R.copyTo(T(cv::Rect(0, 0, 3, 3)));
        t.copyTo(T(cv::Rect(3, 0, 1, 3)));
        
        return T;
    }
    
    std::tuple<double, double, double> extractPose2D(const cv::Mat& transformation) {
        if (transformation.empty()) {
            return {0.0, 0.0, 0.0};
        }
        
        // Extract translation (scaled for ground vehicle)
        double dx = transformation.at<double>(0, 3) * scale_factor_;
        double dz = transformation.at<double>(2, 3) * scale_factor_;
        
        // Extract rotation matrix
        cv::Mat R = transformation(cv::Rect(0, 0, 3, 3));
        
        // Multiple methods to extract yaw and use the most conservative
        double dyaw1 = atan2(R.at<double>(2, 0), R.at<double>(0, 0)); // Original method
        double dyaw2 = atan2(R.at<double>(1, 0), R.at<double>(1, 1)); // Alternative method
        
        // Use the smaller rotation (more conservative)
        double dyaw = (std::abs(dyaw1) < std::abs(dyaw2)) ? dyaw1 : dyaw2;
        
        // Clamp rotation to reasonable limits
        dyaw = std::max(-0.1, std::min(0.1, dyaw)); // Max ±5.7 degrees per frame
        
        // For ground vehicles, we typically use:
        // - X-axis: right/left motion
        // - Z-axis: forward/backward motion  
        // - Y-axis rotation: yaw
        return {dx, dz, dyaw};
    }
    
    void updatePose(double dx, double dy, double dyaw) {
        // Transform relative motion to global coordinates
        double global_dx = dx * cos(current_yaw_) - dy * sin(current_yaw_);
        double global_dy = dx * sin(current_yaw_) + dy * cos(current_yaw_);
        
        // Update pose
        current_x_ += global_dx;
        current_y_ += global_dy;
        current_yaw_ += dyaw;
        
        // Normalize yaw angle
        while (current_yaw_ > M_PI) current_yaw_ -= 2.0 * M_PI;
        while (current_yaw_ < -M_PI) current_yaw_ += 2.0 * M_PI;
    }
    
    void publishPath() {
        auto now = this->now();
        
        // Create pose stamped
        geometry_msgs::msg::PoseStamped pose_stamped;
        pose_stamped.header.stamp = now;
        pose_stamped.header.frame_id = "map";
        pose_stamped.pose.position.x = current_x_;
        pose_stamped.pose.position.y = current_y_;
        pose_stamped.pose.position.z = 0.0;
        
        // Convert yaw to quaternion
        tf2::Quaternion q;
        q.setRPY(0, 0, current_yaw_);
        pose_stamped.pose.orientation = tf2::toMsg(q);
        
        // Add to path and publish
        path_msg_.poses.push_back(pose_stamped);
        path_msg_.header.stamp = now;
        path_pub_->publish(path_msg_);
    }
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<PureVisualOdometry>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
