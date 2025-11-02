#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/path.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/pose2_d.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
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
    VisualOdometrySimple() : Node("visual_odo_simple"), current_frame_(0)
    {
        // Initialize ORB detector and BFMatcher (better for ORB descriptors)
        orb = cv::ORB::create(8000);
        matcher = cv::BFMatcher::create(cv::NORM_HAMMING, true);
        
        // Get parameters
        std::string image_directory = this->declare_parameter<std::string>("image_directory", 
            "/home/jorgelop/Documents/VantTec_SDV_SWARM/SDV_Proyect/SDV_Software_Workspaces/vo_ws/src/vodom_first/KITTI_sequence_2/image_l/");
        
        // Extract base directory (remove image_0/ or image_l/ from path)
        std::string base_dir = image_directory;
        if (base_dir.back() == '/') base_dir.pop_back(); // Remove trailing /
        size_t last_slash = base_dir.find_last_of('/');
        if (last_slash != std::string::npos) {
            base_dir = base_dir.substr(0, last_slash); // Remove image_0 or image_l
        }
        
        loadCalib(base_dir + "/calib.txt");
        loadPoses(base_dir + "/poses.txt");
        loadImages(image_directory);
        
        // Publishers for Visual Odometry
        vo_odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>("/odometry/visual", 10);
        vo_path_pub_ = this->create_publisher<nav_msgs::msg::Path>("/vo_path", 10);
        
        // Publisher for GPS simulation (from ground truth)
        gps_pub_ = this->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>("/gps/pose", 10);
        
        // Initialize pose and path
        current_x_ = 0.0;
        current_y_ = 0.0;
        current_yaw_ = 0.0;
        current_frame_ = 0;
        
        // Initialize path message
        vo_path_msg_.header.frame_id = "map";
        
        // Create timers
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(100),
            std::bind(&VisualOdometrySimple::process_frame, this)
        );
        
        gps_timer_ = this->create_wall_timer(
            std::chrono::seconds(2),  // GPS every 2 seconds
            std::bind(&VisualOdometrySimple::publish_gps, this)
        );
        
        RCLCPP_INFO(this->get_logger(), "Visual Odometry Simple node initialized with %zu images", images.size());
    }

private:
    // ROS2 publishers and timers
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr vo_odom_pub_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr vo_path_pub_;
    rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr gps_pub_;
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::TimerBase::SharedPtr gps_timer_;
    
    // OpenCV objects
    cv::Ptr<cv::ORB> orb;
    cv::Ptr<cv::BFMatcher> matcher;
    
    // Data storage
    std::vector<cv::Mat> images;
    std::vector<cv::Mat> gt_poses;
    cv::Mat K;  // Camera intrinsic matrix
    
    // Current state
    double current_x_, current_y_, current_yaw_;
    size_t current_frame_;
    nav_msgs::msg::Path vo_path_msg_;
    
    void loadCalib(const std::string& file_path) {
        std::ifstream file(file_path);
        if (!file.is_open()) {
            RCLCPP_ERROR(this->get_logger(), "Failed to open calibration file: %s", file_path.c_str());
            return;
        }
        
        std::string line;
        if (std::getline(file, line)) {
            // First line contains P0 calibration matrix
            std::istringstream iss(line);
            std::vector<double> values;
            double val;
            while (iss >> val) {
                values.push_back(val);
            }
            
            if (values.size() >= 12) {
                K = (cv::Mat_<double>(3, 3) <<
                    values[0], values[1], values[2],
                    values[4], values[5], values[6],
                    values[8], values[9], values[10]);
                RCLCPP_INFO(this->get_logger(), "Loaded camera calibration matrix");
            } else {
                RCLCPP_ERROR(this->get_logger(), "Invalid calibration format");
                // Set default calibration for KITTI sequence 00
                K = (cv::Mat_<double>(3, 3) <<
                    718.8560, 0.0, 607.1928,
                    0.0, 718.8560, 185.2157,
                    0.0, 0.0, 1.0);
                RCLCPP_WARN(this->get_logger(), "Using default KITTI calibration");
            }
        } else {
            RCLCPP_ERROR(this->get_logger(), "Empty calibration file");
            // Set default calibration for KITTI sequence 00
            K = (cv::Mat_<double>(3, 3) <<
                718.8560, 0.0, 607.1928,
                0.0, 718.8560, 185.2157,
                0.0, 0.0, 1.0);
            RCLCPP_WARN(this->get_logger(), "Using default KITTI calibration");
        }
    }
    
    void loadPoses(const std::string& file_path) {
        std::ifstream file(file_path);
        if (!file.is_open()) {
            RCLCPP_ERROR(this->get_logger(), "Failed to open poses file: %s", file_path.c_str());
            return;
        }
        
        std::string line;
        while (std::getline(file, line)) {
            std::istringstream iss(line);
            cv::Mat pose = cv::Mat::eye(4, 4, CV_64F);
            
            for (int i = 0; i < 3; i++) {
                for (int j = 0; j < 4; j++) {
                    iss >> pose.at<double>(i, j);
                }
            }
            gt_poses.push_back(pose);
        }
        RCLCPP_INFO(this->get_logger(), "Loaded %zu ground truth poses", gt_poses.size());
    }
    
    void loadImages(const std::string& dir_path) {
        for (int i = 0; i < 1000; i++) {
            // KITTI format: 6-digit zero-padded numbers
            std::stringstream ss;
            ss << std::setfill('0') << std::setw(6) << i;
            
            // Handle directory path (may or may not end with /)
            std::string base_path = dir_path;
            if (base_path.back() != '/') base_path += "/";
            
            std::string img_path = base_path + ss.str() + ".png";
            cv::Mat img = cv::imread(img_path, cv::IMREAD_GRAYSCALE);
            if (!img.empty()) {
                images.push_back(img);
            } else {
                // Stop if we can't find the next consecutive image
                break;
            }
        }
        RCLCPP_INFO(this->get_logger(), "Loaded %zu images", images.size());
    }
    
    void process_frame() {
        if (current_frame_ >= images.size() - 1) {
            RCLCPP_INFO(this->get_logger(), "All frames processed. Visual odometry complete.");
            return;
        }
        
        if (current_frame_ == 0) {
            // Initialize from ground truth for stable start
            if (!gt_poses.empty()) {
                current_x_ = gt_poses[0].at<double>(0, 3);
                current_y_ = gt_poses[0].at<double>(2, 3); // KITTI uses z as forward
                
                // Extract initial yaw from ground truth rotation matrix
                cv::Mat R_gt = gt_poses[0](cv::Rect(0, 0, 3, 3));
                current_yaw_ = atan2(R_gt.at<double>(1, 0), R_gt.at<double>(0, 0));
                
                RCLCPP_INFO(this->get_logger(), "Initialized: x=%.2f, y=%.2f, yaw=%.3f", current_x_, current_y_, current_yaw_);
            }
            current_frame_++;
            return;
        }
        
        // Skip first few frames for stable initialization
        if (current_frame_ < 5) {
            current_frame_++;
            // Publish initial pose during skip frames
            publish_odometry();
            return;
        }
        
        // Get current and previous frames
        cv::Mat img_prev = images[current_frame_ - 1];
        cv::Mat img_curr = images[current_frame_];
        
        // Feature detection and matching
        std::vector<cv::KeyPoint> kp_prev, kp_curr;
        cv::Mat desc_prev, desc_curr;
        
        orb->detectAndCompute(img_prev, cv::noArray(), kp_prev, desc_prev);
        orb->detectAndCompute(img_curr, cv::noArray(), kp_curr, desc_curr);
        
        if (desc_prev.empty() || desc_curr.empty()) {
            current_frame_++;
            return;
        }
        
        // Match features (no need to convert for BFMatcher with ORB)
        std::vector<cv::DMatch> matches;
        if (desc_prev.rows >= 2 && desc_curr.rows >= 2) {
            matcher->match(desc_prev, desc_curr, matches);
        }
        
        // Filter matches
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
        
        // Estimate motion if we have enough matches
        if (good_matches.size() >= 8) {
            std::vector<cv::Point2f> pts_prev, pts_curr;
            for (const auto& match : good_matches) {
                pts_prev.push_back(kp_prev[match.queryIdx].pt);
                pts_curr.push_back(kp_curr[match.trainIdx].pt);
            }
            
            // Find essential matrix
            cv::Mat mask;
            cv::Mat E = cv::findEssentialMat(pts_prev, pts_curr, K, cv::RANSAC, 0.999, 1.0, mask);
            
            if (!E.empty()) {
                cv::Mat R, t;
                int inliers = cv::recoverPose(E, pts_prev, pts_curr, K, R, t, mask);
                
                if (inliers > 10) {
                    // For KITTI, we can use ground truth for scale estimation
                    double scale = 1.0;
                    if (current_frame_ < gt_poses.size() - 1) {
                        cv::Mat prev_pose = gt_poses[current_frame_ - 1];
                        cv::Mat curr_pose = gt_poses[current_frame_];
                        
                        double gt_dx = curr_pose.at<double>(0, 3) - prev_pose.at<double>(0, 3);
                        double gt_dz = curr_pose.at<double>(2, 3) - prev_pose.at<double>(2, 3);
                        double gt_dist = sqrt(gt_dx * gt_dx + gt_dz * gt_dz);
                        
                        double est_dist = sqrt(t.at<double>(0) * t.at<double>(0) + t.at<double>(2) * t.at<double>(2));
                        if (est_dist > 0.01) {
                            scale = gt_dist / est_dist;
                        }
                    }
                    
                    // Apply scale and transform to world coordinates
                    double dx = scale * t.at<double>(0);
                    double dz = scale * t.at<double>(2);
                    
                    // Extract yaw from rotation matrix - more conservative approach
                    // Use the rotation around Y axis (vehicle yaw)
                    double dyaw = atan2(R.at<double>(2, 0), R.at<double>(0, 0));
                    
                    // Alternative: Use smaller rotation estimates
                    // dyaw = atan2(R.at<double>(1, 0), R.at<double>(0, 0)) * 0.5; // Reduce by half
                    
                    // Filter out excessive rotations (likely noise)
                    if (abs(dyaw) > 0.05) { // Limit to ~3 degrees per frame
                        dyaw = dyaw > 0 ? 0.05 : -0.05; // Clamp instead of zero
                        RCLCPP_WARN(this->get_logger(), "Frame %zu: Clamped excessive rotation", current_frame_);
                    }
                    
                    // Transform relative motion to world coordinates
                    current_x_ += dx * cos(current_yaw_) - dz * sin(current_yaw_);
                    current_y_ += dx * sin(current_yaw_) + dz * cos(current_yaw_);
                    current_yaw_ += dyaw;
                    
                    RCLCPP_INFO(this->get_logger(), "Frame %zu: matches=%zu, inliers=%d, scale=%.3f, dx=%.3f, dz=%.3f, dyaw=%.3f", 
                                current_frame_, good_matches.size(), inliers, scale, dx, dz, dyaw);
                }
            }
        }
        
        // Publish odometry
        publish_odometry();
        
        current_frame_++;
    }
    
    void publish_odometry() {
        auto now = this->get_clock()->now();
        
        // Create odometry message
        auto odom_msg = nav_msgs::msg::Odometry();
        odom_msg.header.stamp = now;
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
        
        // Set covariance (simple diagonal)
        for (int i = 0; i < 36; i++) {
            odom_msg.pose.covariance[i] = 0.0;
        }
        odom_msg.pose.covariance[0] = 10.0;   // x - Incertidumbre muy alta para dar máximo peso al GPS
        odom_msg.pose.covariance[7] = 10.0;   // y - Incertidumbre muy alta para dar máximo peso al GPS
        odom_msg.pose.covariance[35] = 5.0;  // yaw - Incertidumbre muy alta para dar máximo peso al GPS
        
        vo_odom_pub_->publish(odom_msg);
        
        // Add to path and publish
        geometry_msgs::msg::PoseStamped pose_stamped;
        pose_stamped.header = odom_msg.header;
        pose_stamped.pose = odom_msg.pose.pose;
        vo_path_msg_.poses.push_back(pose_stamped);
        vo_path_msg_.header.stamp = now;
        
        vo_path_pub_->publish(vo_path_msg_);
    }
    
    void publish_gps() {
        if (current_frame_ >= gt_poses.size()) {
            return;
        }
        
        // Get ground truth pose
        cv::Mat pose = gt_poses[current_frame_];
        double gt_x = pose.at<double>(0, 3);
        double gt_y = pose.at<double>(2, 3); // Note: KITTI uses z as forward
        
        // Create GPS message (PoseWithCovarianceStamped)
        auto gps_msg = geometry_msgs::msg::PoseWithCovarianceStamped();
        gps_msg.header.stamp = this->get_clock()->now();
        gps_msg.header.frame_id = "map";
        
        gps_msg.pose.pose.position.x = gt_x;
        gps_msg.pose.pose.position.y = gt_y;
        gps_msg.pose.pose.position.z = 0.0;
        
        // Set orientation to identity
        gps_msg.pose.pose.orientation.w = 1.0;
        
        // Set GPS covariance (higher uncertainty than odometry)
        for (int i = 0; i < 36; i++) {
            gps_msg.pose.covariance[i] = 0.0;
        }
        gps_msg.pose.covariance[0] = 1.0;   // x variance
        gps_msg.pose.covariance[7] = 1.0;   // y variance
        gps_msg.pose.covariance[35] = 999999.0; // yaw (GPS doesn't provide orientation)
        
        gps_pub_->publish(gps_msg);
        
        RCLCPP_INFO(this->get_logger(), "GPS update: (%.2f, %.2f)", gt_x, gt_y);
    }
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<VisualOdometrySimple>());
    rclcpp::shutdown();
    return 0;
}