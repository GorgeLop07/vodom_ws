#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/path.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
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
#include <deque>
//Lol
class VORobustOdometry : public rclcpp::Node {
public:
    VORobustOdometry() : Node("vo_robust_odometry") {
        // Load camera calibration
        if (!loadCameraCalibration()) {
            RCLCPP_ERROR(this->get_logger(), "Failed to load camera calibration");
            return;
        }
        
        // Initialize ORB feature detector and BFMatcher
        orb_ = cv::ORB::create(8000);
        bf_matcher_ = cv::BFMatcher::create(cv::NORM_HAMMING, false);
        
        // Publishers
        vo_path_pub_ = this->create_publisher<nav_msgs::msg::Path>("/vo_path", 10);
        fused_path_pub_ = this->create_publisher<nav_msgs::msg::Path>("/fused_path", 10);
        vo_odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>("/vo_odometry", 10);
        fused_odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>("/fused_odometry", 10);
        
        // Subscribers
        image_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
            "/camera/image_raw", 10,
            std::bind(&VORobustOdometry::imageCallback, this, std::placeholders::_1)
        );
        
        gnss_sub_ = this->create_subscription<sensor_msgs::msg::NavSatFix>(
            "/vectornav/gnss", 10,
            std::bind(&VORobustOdometry::gnssCallback, this, std::placeholders::_1)
        );
        
        imu_sub_ = this->create_subscription<sensor_msgs::msg::Imu>(
            "/vectornav/imu", 10,
            std::bind(&VORobustOdometry::imuCallback, this, std::placeholders::_1)
        );
        
        // Initialize path messages
        vo_path_msg_.header.frame_id = "map";
        fused_path_msg_.header.frame_id = "map";
        
        // Initialize state variables
        initializeState();
        
        RCLCPP_INFO(this->get_logger(), "VORobust Odometry Node initialized");
        RCLCPP_INFO(this->get_logger(), "Camera params - fx: %.3f, fy: %.3f, cx: %.3f, cy: %.3f", 
                   camera_matrix_.at<double>(0,0), camera_matrix_.at<double>(1,1),
                   camera_matrix_.at<double>(0,2), camera_matrix_.at<double>(1,2));
    }

private:
    // ROS2 components
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr vo_path_pub_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr fused_path_pub_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr vo_odom_pub_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr fused_odom_pub_;
    
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
    rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr gnss_sub_;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
    
    // OpenCV components
    cv::Ptr<cv::ORB> orb_;
    cv::Ptr<cv::BFMatcher> bf_matcher_;
    cv::Mat camera_matrix_;
    cv::Mat distortion_coeffs_;
    cv::Mat prev_image_;
    
    // State variables
    nav_msgs::msg::Path vo_path_msg_;
    nav_msgs::msg::Path fused_path_msg_;
    
    // Visual Odometry state
    double vo_x_, vo_y_, vo_yaw_;
    bool first_frame_;
    double scale_factor_;
    
    // Fused state (VO + GPS + IMU)
    double fused_x_, fused_y_, fused_yaw_;
    
    // GPS state
    double gps_x_, gps_y_;
    bool gps_initialized_;
    double gps_origin_lat_, gps_origin_lon_;
    rclcpp::Time last_gps_time_;
    
    // IMU state
    double imu_yaw_;
    bool imu_initialized_;
    rclcpp::Time last_imu_time_;
    
    // Filtering parameters
    double gps_weight_;
    double vo_weight_;
    double imu_weight_;
    std::deque<std::pair<double, double>> vo_history_; // For smoothing
    
    void initializeState() {
        // VO state
        vo_x_ = 0.0;
        vo_y_ = 0.0;
        vo_yaw_ = 0.0;
        first_frame_ = true;
        scale_factor_ = 0.1;
        
        // Fused state
        fused_x_ = 0.0;
        fused_y_ = 0.0;
        fused_yaw_ = 0.0;
        
        // GPS state
        gps_x_ = 0.0;
        gps_y_ = 0.0;
        gps_initialized_ = false;
        
        // IMU state
        imu_yaw_ = 0.0;
        imu_initialized_ = false;
        
        // Fusion weights (can be tuned)
        gps_weight_ = 0.7;   // High trust in GPS for position
        vo_weight_ = 0.3;    // Lower trust in VO for position
        imu_weight_ = 0.8;   // High trust in IMU for orientation
    }
    
    bool loadCameraCalibration() {
        std::string calib_file = "/home/jorgelop/Documents/VantTec_SDV_SWARM/SDV_Proyect/SDV_Software_Workspaces/vo_ws/src/vodom_first/config/calib(1).txt";
        std::ifstream file(calib_file);
        
        if (!file.is_open()) {
            RCLCPP_ERROR(this->get_logger(), "Cannot open calibration file: %s", calib_file.c_str());
            return false;
        }
        
        // Initialize matrices
        camera_matrix_ = cv::Mat::zeros(3, 3, CV_64F);
        distortion_coeffs_ = cv::Mat::zeros(5, 1, CV_64F);
        
        std::string line;
        bool reading_intrinsics = false;
        bool reading_distortion = false;
        int matrix_row = 0;
        
        while (std::getline(file, line)) {
            if (line.find("Matriz Intrinseca") != std::string::npos) {
                reading_intrinsics = true;
                reading_distortion = false;
                matrix_row = 0;
                continue;
            }
            if (line.find("Coeficientes de Distorsion") != std::string::npos) {
                reading_intrinsics = false;
                reading_distortion = true;
                continue;
            }
            if (line.find("Extrinseca") != std::string::npos) {
                break;
            }
            
            // Read intrinsic matrix (3x3)
            if (reading_intrinsics && !line.empty() && matrix_row < 3) {
                std::istringstream iss(line);
                double val;
                int col = 0;
                while (iss >> val && col < 3) {
                    camera_matrix_.at<double>(matrix_row, col) = val;
                    col++;
                }
                matrix_row++;
            }
            
            // Read distortion coefficients (5 values)
            if (reading_distortion && !line.empty()) {
                std::istringstream iss(line);
                double val;
                int idx = 0;
                while (iss >> val && idx < 5) {
                    distortion_coeffs_.at<double>(idx, 0) = val;
                    idx++;
                }
                reading_distortion = false;
            }
        }
        
        return true;
    }
    
    void imageCallback(const sensor_msgs::msg::Image::SharedPtr msg) {
        try {
            // Convert ROS image to OpenCV
            cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
            cv::Mat current_image;
            
            // Apply undistortion to correct lens distortion
            cv::Mat undistorted_image;
            cv::undistort(cv_ptr->image, undistorted_image, camera_matrix_, distortion_coeffs_);
            
            // Convert to grayscale for ORB processing
            cv::cvtColor(undistorted_image, current_image, cv::COLOR_BGR2GRAY);
            
            if (first_frame_) {
                prev_image_ = current_image.clone();
                first_frame_ = false;
                RCLCPP_INFO(this->get_logger(), "First frame received, starting visual odometry");
                return;
            }
            
            // Process visual odometry
            processVisualOdometry(current_image);
            
            // Fuse with GPS/IMU data
            fuseWithSensors();
            
            // Publish results
            publishOdometry();
            
            // Update previous frame
            prev_image_ = current_image.clone();
            
        } catch (cv_bridge::Exception& e) {
            RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
        }
    }
    
    void gnssCallback(const sensor_msgs::msg::NavSatFix::SharedPtr msg) {
        // Check for valid GPS fix
        if (msg->status.status < 0) {
            RCLCPP_WARN(this->get_logger(), "Invalid GPS fix, status: %d", msg->status.status);
            return;
        }
        
        if (!gps_initialized_) {
            // Initialize GPS origin (first valid fix)
            gps_origin_lat_ = msg->latitude;
            gps_origin_lon_ = msg->longitude;
            gps_initialized_ = true;
            
            RCLCPP_INFO(this->get_logger(), "GPS initialized at lat: %.8f, lon: %.8f", 
                       gps_origin_lat_, gps_origin_lon_);
            return;
        }
        
        // Convert lat/lon to local XY coordinates (simple UTM approximation)
        convertGpsToXY(msg->latitude, msg->longitude, gps_x_, gps_y_);
        last_gps_time_ = this->get_clock()->now();
        
        RCLCPP_INFO(this->get_logger(), "GPS update: (%.2f, %.2f) from lat: %.8f, lon: %.8f", 
                   gps_x_, gps_y_, msg->latitude, msg->longitude);
    }
    
    void imuCallback(const sensor_msgs::msg::Imu::SharedPtr msg) {
        // Extract yaw from IMU quaternion
        tf2::Quaternion q(
            msg->orientation.x,
            msg->orientation.y, 
            msg->orientation.z,
            msg->orientation.w
        );
        
        double roll, pitch, yaw;
        tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);
        
        imu_yaw_ = yaw;
        imu_initialized_ = true;
        last_imu_time_ = this->get_clock()->now();
        
        RCLCPP_DEBUG(this->get_logger(), "IMU yaw: %.3f rad (%.1f deg)", yaw, yaw * 57.3);
    }
    
    void convertGpsToXY(double lat, double lon, double& x, double& y) {
        // Simple conversion from lat/lon to local XY (meters)
        // This is a basic approximation - for more accuracy use proper UTM conversion
        const double lat_rad = lat * M_PI / 180.0;
        const double lon_rad = lon * M_PI / 180.0;
        const double origin_lat_rad = gps_origin_lat_ * M_PI / 180.0;
        const double origin_lon_rad = gps_origin_lon_ * M_PI / 180.0;
        
        const double earth_radius = 6378137.0; // meters
        
        x = earth_radius * (lon_rad - origin_lon_rad) * cos(origin_lat_rad);
        y = earth_radius * (lat_rad - origin_lat_rad);
    }
    
    void processVisualOdometry(const cv::Mat& current_image) {
        // Detect and match features
        std::vector<cv::Point2f> prev_points, curr_points;
        if (!detectAndMatchFeatures(prev_image_, current_image, prev_points, curr_points)) {
            RCLCPP_DEBUG(this->get_logger(), "Feature matching failed");
            return;
        }
        
        if (prev_points.size() < 10) {
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
        
        // Basic filter for unrealistic movements
        if (std::abs(dx) > 1.0 || std::abs(dy) > 1.0 || std::abs(dyaw) > 0.3) {
            RCLCPP_DEBUG(this->get_logger(), "Unrealistic motion detected, skipping");
            return;
        }
        
        // Update VO pose
        updateVOPose(dx, dy, dyaw);
        
        // Add to history for smoothing
        vo_history_.push_back({dx, dy});
        if (vo_history_.size() > 5) {
            vo_history_.pop_front();
        }
        
        RCLCPP_INFO(this->get_logger(), "VO: features=%zu, motion=(%.3f,%.3f,%.1f°)", 
                    prev_points.size(), dx, dy, dyaw * 57.3);
    }
    
    void fuseWithSensors() {
        // Simple weighted fusion of VO + GPS + IMU
        
        // Position fusion (VO + GPS)
        if (gps_initialized_) {
            // Weighted average of VO and GPS positions
            fused_x_ = vo_weight_ * vo_x_ + gps_weight_ * gps_x_;
            fused_y_ = vo_weight_ * vo_y_ + gps_weight_ * gps_y_;
        } else {
            // Use pure VO if no GPS
            fused_x_ = vo_x_;
            fused_y_ = vo_y_;
        }
        
        // Orientation fusion (VO + IMU)
        if (imu_initialized_) {
            // Weighted average of VO and IMU yaw
            fused_yaw_ = (1.0 - imu_weight_) * vo_yaw_ + imu_weight_ * imu_yaw_;
        } else {
            // Use pure VO if no IMU
            fused_yaw_ = vo_yaw_;
        }
        
        // Normalize yaw
        while (fused_yaw_ > M_PI) fused_yaw_ -= 2.0 * M_PI;
        while (fused_yaw_ < -M_PI) fused_yaw_ += 2.0 * M_PI;
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
        
        // Match features using BFMatcher with ratio test
        std::vector<std::vector<cv::DMatch>> knn_matches;
        bf_matcher_->knnMatch(desc1, desc2, knn_matches, 2);
        
        // Apply Lowe's ratio test
        const float ratio_thresh = 0.75f;
        for (const auto& match : knn_matches) {
            if (match.size() == 2 && match[0].distance < ratio_thresh * match[1].distance) {
                points1.push_back(kp1[match[0].queryIdx].pt);
                points2.push_back(kp2[match[0].trainIdx].pt);
            }
        }
        
        return points1.size() >= 10;
    }
    
    cv::Mat estimateMotion(const std::vector<cv::Point2f>& points1, const std::vector<cv::Point2f>& points2) {
        // Find essential matrix using RANSAC
        cv::Mat mask;
        cv::Mat E = cv::findEssentialMat(points1, points2, camera_matrix_, 
                                        cv::RANSAC, 0.99, 2.0, mask);
        
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
        
        if (inlier_points1.size() < 10) {
            return cv::Mat();
        }
        
        // Recover pose from essential matrix
        cv::Mat R, t;
        int inliers = cv::recoverPose(E, inlier_points1, inlier_points2, camera_matrix_, R, t);
        
        if (inliers < 10) {
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
        
        // Extract translation (scaled)
        double dx = transformation.at<double>(0, 3) * scale_factor_;
        double dz = transformation.at<double>(2, 3) * scale_factor_;
        
        // Extract rotation matrix and yaw
        cv::Mat R = transformation(cv::Rect(0, 0, 3, 3));
        double dyaw = atan2(R.at<double>(2, 0), R.at<double>(0, 0));
        
        return {dx, dz, dyaw};
    }
    
    void updateVOPose(double dx, double dy, double dyaw) {
        // Transform relative motion to global coordinates
        double global_dx = dx * cos(vo_yaw_) - dy * sin(vo_yaw_);
        double global_dy = dx * sin(vo_yaw_) + dy * cos(vo_yaw_);
        
        // Update VO pose
        vo_x_ += global_dx;
        vo_y_ += global_dy;
        vo_yaw_ += dyaw;
        
        // Normalize yaw
        while (vo_yaw_ > M_PI) vo_yaw_ -= 2.0 * M_PI;
        while (vo_yaw_ < -M_PI) vo_yaw_ += 2.0 * M_PI;
    }
    
    void publishOdometry() {
        auto now = this->now();
        
        // Publish VO-only odometry and path
        publishSingleOdometry(vo_x_, vo_y_, vo_yaw_, "/vo_odometry", "/vo_path", 
                             vo_odom_pub_, vo_path_pub_, vo_path_msg_);
        
        // Publish fused odometry and path
        publishSingleOdometry(fused_x_, fused_y_, fused_yaw_, "/fused_odometry", "/fused_path",
                             fused_odom_pub_, fused_path_pub_, fused_path_msg_);
        
        RCLCPP_INFO(this->get_logger(), "State - VO:(%.2f,%.2f,%.1f°) Fused:(%.2f,%.2f,%.1f°) GPS:%s IMU:%s",
                   vo_x_, vo_y_, vo_yaw_*57.3, fused_x_, fused_y_, fused_yaw_*57.3,
                   gps_initialized_ ? "OK" : "NO", imu_initialized_ ? "OK" : "NO");
    }
    
    void publishSingleOdometry(double x, double y, double yaw, 
                              const std::string& odom_topic, const std::string& path_topic,
                              rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub,
                              rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub,
                              nav_msgs::msg::Path& path_msg) {
        auto now = this->now();
        
        // Create odometry message
        auto odom_msg = nav_msgs::msg::Odometry();
        odom_msg.header.stamp = now;
        odom_msg.header.frame_id = "map";
        odom_msg.child_frame_id = "base_link";
        
        // Set position
        odom_msg.pose.pose.position.x = x;
        odom_msg.pose.pose.position.y = y;
        odom_msg.pose.pose.position.z = 0.0;
        
        // Set orientation
        tf2::Quaternion q;
        q.setRPY(0, 0, yaw);
        odom_msg.pose.pose.orientation = tf2::toMsg(q);
        
        // Set covariance
        for (int i = 0; i < 36; i++) {
            odom_msg.pose.covariance[i] = 0.0;
        }
        odom_msg.pose.covariance[0] = 0.1;   // x
        odom_msg.pose.covariance[7] = 0.1;   // y  
        odom_msg.pose.covariance[35] = 0.1;  // yaw
        
        odom_pub->publish(odom_msg);
        
        // Create and publish path
        geometry_msgs::msg::PoseStamped pose_stamped;
        pose_stamped.header = odom_msg.header;
        pose_stamped.pose = odom_msg.pose.pose;
        path_msg.poses.push_back(pose_stamped);
        path_msg.header.stamp = now;
        
        path_pub->publish(path_msg);
    }
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<VORobustOdometry>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}