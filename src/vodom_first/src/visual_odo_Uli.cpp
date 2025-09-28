#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/path.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/pose2_d.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/features2d.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/highgui.hpp>
#include <iostream>
#include <fstream>
#include <vector>
#include <string>
#include <algorithm>
#include <filesystem>
#include <cmath>
#include <eigen3/Eigen/Dense>
#include <chrono>

namespace fs = std::filesystem;

class VisualOdometryUli : public rclcpp::Node {
public:
    VisualOdometryUli(const std::string& data_dir)
    : Node("visual_odo_Uli") {
        loadCalib(data_dir + "/calib.txt");
        loadPoses(data_dir + "/poses.txt");
        loadImages(data_dir + "/image_0"); //Cambiar a l en caso de primer dataset
        orb = cv::ORB::create(8000);//Intente subir el numero de features de 5000 a 8000 a ver que pasa XD 
        flann = cv::Ptr<cv::FlannBasedMatcher>(new cv::FlannBasedMatcher(new cv::flann::LshIndexParams(6, 12, 1)));
        
        // Publishers for VO pure (red in RViz)
        vo_path_pub_ = this->create_publisher<nav_msgs::msg::Path>("/vo_path_2d", 10);
        vo_odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>("/vo_odom_2d", 10);
        
        // Publishers for EKF fused (blue in RViz)
        ekf_path_pub_ = this->create_publisher<nav_msgs::msg::Path>("/ekf_path_2d", 10);
        ekf_odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>("/ekf_odom_2d", 10);
        
        // Legacy publishers (keep for compatibility)
        path_pub_ = this->create_publisher<nav_msgs::msg::Path>("vo_path_uliXD", 10);
        odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>("vo_odom_2d", 10);
        pose2d_pub_ = this->create_publisher<geometry_msgs::msg::Pose2D>("vo_pose_2d", 10);
        
        // EKF Fusion: Subscribe to ground truth (simulating GPS)
        gt_sub_ = this->create_subscription<geometry_msgs::msg::Pose2D>(
            "/gt_pose_2d", 10,
            std::bind(&VisualOdometryUli::ground_truth_callback, this, std::placeholders::_1)
        );
        
        // Initialize EKF
        init_ekf();
        
        // Initialize VO pure pose (red path)
        current_x_ = 0.0;
        current_y_ = 0.0;
        current_yaw_ = 0.0;
        
        // Initialize EKF pose (blue path)  
        ekf_x_ = 0.0;
        ekf_y_ = 0.0;
        ekf_yaw_ = 0.0;
        
        current_frame_ = 0;
        
        // Create timer for processing frames
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(100), // 10 Hz
            std::bind(&VisualOdometryUli::process_frame, this)
        );
        
        // Initialize path message
        // Initialize path messages
        vo_path_msg_.header.frame_id = "map";
        ekf_path_msg_.header.frame_id = "map"; 
        path_msg_.header.frame_id = "map";  // Legacy
        
        RCLCPP_INFO(this->get_logger(), "Visual Odometry 2D node initialized with %zu images", images.size());
    }

private:
    cv::Mat K, P;
    std::vector<cv::Mat> gt_poses;
    std::vector<cv::Mat> images;
    cv::Ptr<cv::ORB> orb;
    cv::Ptr<cv::FlannBasedMatcher> flann;
    // VO Pure publishers (red in RViz)
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr vo_path_pub_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr vo_odom_pub_;
    
    // EKF Fused publishers (blue in RViz)
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr ekf_path_pub_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr ekf_odom_pub_;
    
    // Legacy publishers (keep for compatibility)
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
    rclcpp::Publisher<geometry_msgs::msg::Pose2D>::SharedPtr pose2d_pub_;
    rclcpp::Subscription<geometry_msgs::msg::Pose2D>::SharedPtr gt_sub_;
    rclcpp::TimerBase::SharedPtr timer_;
    
    // 2D pose tracking - VO PURE (for red path)
    double current_x_, current_y_, current_yaw_;
    
    // EKF pose tracking (for blue path)
    double ekf_x_, ekf_y_, ekf_yaw_;
    
    size_t current_frame_;
    // Separate path messages for VO and EKF
    nav_msgs::msg::Path vo_path_msg_;    // Red path (VO pure)
    nav_msgs::msg::Path ekf_path_msg_;   // Blue path (EKF fused)
    nav_msgs::msg::Path path_msg_;       // Legacy path
    
    // EKF State: [x, y, yaw, vx, vy, vyaw]
    Eigen::VectorXd state_;           // State vector (6x1)
    Eigen::MatrixXd covariance_;      // Covariance matrix (6x6)
    Eigen::MatrixXd process_noise_;   // Process noise Q (6x6)
    Eigen::MatrixXd vo_noise_;        // VO measurement noise (3x3)
    Eigen::MatrixXd gps_noise_;       // GPS measurement noise (3x3)
    
    std::chrono::steady_clock::time_point last_time_;
    bool ekf_initialized_;
    double gps_correction_interval_;  // Seconds between GPS corrections
    double last_gps_time_;

    void process_frame() {
        // Check if we've processed all frames
        if (current_frame_ >= images.size()) {
            RCLCPP_INFO(this->get_logger(), "All frames processed. Visual odometry complete.");
            timer_->cancel();
            return;
        }
        
        if (current_frame_ == 0) {
            // Initialize VO and EKF from ground truth or keep zeros
            if (!gt_poses.empty()) {
                current_x_ = gt_poses[0].at<double>(0, 3);
                current_y_ = gt_poses[0].at<double>(1, 3);
                current_yaw_ = 0.0;
                
                // Initialize EKF with same starting point
                ekf_x_ = current_x_;
                ekf_y_ = current_y_;
                ekf_yaw_ = current_yaw_;
            }
            RCLCPP_INFO(this->get_logger(), "Processing frame %zu/%zu", current_frame_ + 1, images.size());
        } else {
            // Calculate time step for EKF
            auto current_time = std::chrono::steady_clock::now();
            double dt = std::chrono::duration<double>(current_time - last_time_).count();
            last_time_ = current_time;
            
            std::vector<cv::Point2f> q1, q2;
            getMatches(current_frame_, q1, q2);
            auto [dx, dy, dyaw] = getPose2D(q1, q2);
            
            // EKF FUSION: Predict + Update with VO
            if (ekf_initialized_) {
                predict_step(dt);              // Predict state forward
                update_with_vo(dx, dy, dyaw);  // Update with VO measurement
            } else {
                // Fallback to simple integration if EKF not ready
                current_x_ += dx * cos(current_yaw_) - dy * sin(current_yaw_);
                current_y_ += dx * sin(current_yaw_) + dy * cos(current_yaw_);
                current_yaw_ += dyaw;
                
                while (current_yaw_ > M_PI) current_yaw_ -= 2.0 * M_PI;
                while (current_yaw_ < -M_PI) current_yaw_ += 2.0 * M_PI;
            }
            
            if (current_frame_ % 10 == 0) {
                RCLCPP_INFO(this->get_logger(), "Frame %zu/%zu - EKF Pose: (%.2f, %.2f, %.2f°) | VO: (%.3f, %.3f, %.3f°)", 
                           current_frame_ + 1, images.size(), 
                           current_x_, current_y_, current_yaw_ * 180.0 / M_PI,
                           dx, dy, dyaw * 180.0 / M_PI);
            }
        }
        
        // Create and publish pose messages
        publish_current_pose();
        
        current_frame_++;
    }

    void publish_current_pose() {
        auto now = this->now();
        
        // Create pose stamped for path
        geometry_msgs::msg::PoseStamped pose_stamped;
        pose_stamped.header.stamp = now;
        pose_stamped.header.frame_id = "map";
        pose_stamped.pose.position.x = current_x_;
        pose_stamped.pose.position.y = current_y_;
        pose_stamped.pose.position.z = 0.0; // Always zero for 2D
        
        // Convert yaw to quaternion
        tf2::Quaternion q;
        q.setRPY(0, 0, current_yaw_);
        pose_stamped.pose.orientation = tf2::toMsg(q);
        
        // Publish VO Pure Path (RED)
        vo_path_msg_.poses.push_back(pose_stamped);
        vo_path_msg_.header.stamp = now;
        vo_path_pub_->publish(vo_path_msg_);
        
        // Create EKF pose
        geometry_msgs::msg::PoseStamped ekf_pose_stamped;
        ekf_pose_stamped.header.frame_id = "map";
        ekf_pose_stamped.header.stamp = now;
        ekf_pose_stamped.pose.position.x = ekf_x_;  // Use EKF variables
        ekf_pose_stamped.pose.position.y = ekf_y_;  // Use EKF variables
        ekf_pose_stamped.pose.position.z = 0.0;
        
        tf2::Quaternion ekf_q;
        ekf_q.setRPY(0, 0, ekf_yaw_);  // Use EKF yaw
        ekf_pose_stamped.pose.orientation = tf2::toMsg(ekf_q);
        
        // Publish EKF Path (BLUE)
        ekf_path_msg_.poses.push_back(ekf_pose_stamped);
        ekf_path_msg_.header.stamp = now;
        ekf_path_pub_->publish(ekf_path_msg_);
        
        // Legacy: Add to path and publish
        path_msg_.poses.push_back(pose_stamped);
        path_msg_.header.stamp = now;
        path_pub_->publish(path_msg_);
        
        // Publish 2D pose
        geometry_msgs::msg::Pose2D pose2d_msg;
        pose2d_msg.x = current_x_;
        pose2d_msg.y = current_y_;
        pose2d_msg.theta = current_yaw_;
        pose2d_pub_->publish(pose2d_msg);
        
        // Publish odometry
        nav_msgs::msg::Odometry odom_msg;
        odom_msg.header.stamp = now;
        odom_msg.header.frame_id = "map";
        odom_msg.child_frame_id = "base_link";
        odom_msg.pose.pose = pose_stamped.pose;
        odom_pub_->publish(odom_msg);
    }

    void loadCalib(const std::string& filepath) {
        std::ifstream f(filepath);
        std::string line;
        std::getline(f, line);
        std::vector<double> params;
        std::istringstream iss(line);
        double val;
        while (iss >> val) params.push_back(val);
        if (params.size() != 12) {
            RCLCPP_ERROR(this->get_logger(), "Calib file does not have 12 elements, found %zu", params.size());
            throw std::runtime_error("Calib file format error");
        }
        cv::Mat P_ = cv::Mat(params).reshape(1, 3);
        P = cv::Mat(3, 4, CV_64F);
        for (int i = 0; i < 3; ++i)
            for (int j = 0; j < 4; ++j)
                P.at<double>(i, j) = P_.at<double>(i, j);
        K = P(cv::Rect(0, 0, 3, 3)).clone();
    }

    void loadPoses(const std::string& filepath) {
        std::ifstream f(filepath);
        std::string line;
        while (std::getline(f, line)) {
            std::istringstream iss(line);
            std::vector<double> vals;
            double v;
            while (iss >> v) vals.push_back(v);
            if (vals.size() != 12) {
                RCLCPP_WARN(this->get_logger(), "Pose line does not have 12 elements, found %zu. Skipping.", vals.size());
                continue;
            }
            cv::Mat T = cv::Mat(vals).reshape(1, 3);
            cv::Mat pose = cv::Mat::eye(4, 4, CV_64F);
            for (int i = 0; i < 3; ++i)
                for (int j = 0; j < 4; ++j)
                    pose.at<double>(i, j) = T.at<double>(i, j);
            gt_poses.push_back(pose);
        }
    }

    void loadImages(const std::string& dirpath) {
        std::vector<std::string> files;
        for (const auto& entry : fs::directory_iterator(dirpath)) {
            if (entry.path().extension() == ".png" || entry.path().extension() == ".jpg") {
                files.push_back(entry.path().string());
            }
        }
        std::sort(files.begin(), files.end());
        for (const auto& file : files) {
            cv::Mat img = cv::imread(file, cv::IMREAD_GRAYSCALE);
            if (!img.empty()) images.push_back(img);
        }
    }

    void getMatches(int i, std::vector<cv::Point2f>& q1, std::vector<cv::Point2f>& q2) {
        std::vector<cv::KeyPoint> kp1, kp2;
        cv::Mat des1, des2;
        orb->detectAndCompute(images[i - 1], cv::noArray(), kp1, des1);
        orb->detectAndCompute(images[i], cv::noArray(), kp2, des2);
        if (des1.empty() || des2.empty()) return;
        std::vector<std::vector<cv::DMatch>> matches;
        flann->knnMatch(des1, des2, matches, 2);
        for (auto& m : matches) {
            if (m.size() == 2 && m[0].distance < 0.7 * m[1].distance) {
                q1.push_back(kp1[m[0].queryIdx].pt);
                q2.push_back(kp2[m[0].trainIdx].pt);
            }
        }
    }

    // Original 3D pose estimation (kept for reference)
    cv::Mat getPose(const std::vector<cv::Point2f>& q1, const std::vector<cv::Point2f>& q2) {
        if (q1.size() < 8 || q2.size() < 8) return cv::Mat::eye(4, 4, CV_64F);
        cv::Mat mask;
        cv::Mat E = cv::findEssentialMat(q1, q2, K, cv::RANSAC, 0.999, 0.5, mask);
        if (E.empty()) return cv::Mat::eye(4, 4, CV_64F);
        std::vector<cv::Point2f> q1f, q2f;
        if (!mask.empty() && mask.rows == (int)q1.size()) {
            for (int i = 0; i < mask.rows; ++i) {
                if (mask.at<uchar>(i)) {
                    q1f.push_back(q1[i]);
                    q2f.push_back(q2[i]);
                }
            }
        } else {
            q1f = q1;
            q2f = q2;
        }
        cv::Mat R, t;
        cv::recoverPose(E, q1f, q2f, K, R, t);
        cv::Mat T = cv::Mat::eye(4, 4, CV_64F);
        R.copyTo(T(cv::Rect(0, 0, 3, 3)));
        t = t * 1.0;
        for (int i = 0; i < 3; ++i) T.at<double>(i, 3) = t.at<double>(i);
        return T;
    }

    // New 2D pose estimation for ground vehicles
    std::tuple<double, double, double> getPose2D(const std::vector<cv::Point2f>& q1, const std::vector<cv::Point2f>& q2) {
        if (q1.size() < 8 || q2.size() < 8) return {0.0, 0.0, 0.0};
        
        // Use essential matrix approach but extract only 2D motion
        cv::Mat mask;
        cv::Mat E = cv::findEssentialMat(q1, q2, K, cv::RANSAC, 0.999, 0.5, mask);
        if (E.empty()) return {0.0, 0.0, 0.0};
        
        std::vector<cv::Point2f> q1f, q2f;
        if (!mask.empty() && mask.rows == (int)q1.size()) {
            for (int i = 0; i < mask.rows; ++i) {
                if (mask.at<uchar>(i)) {
                    q1f.push_back(q1[i]);
                    q2f.push_back(q2[i]);
                }
            }
        } else {
            q1f = q1;
            q2f = q2;
        }
        
        cv::Mat R, t;
        cv::recoverPose(E, q1f, q2f, K, R, t);
        
        // Extract 2D motion from 3D transformation
        double dx = t.at<double>(0) * 1.0; // Scale factor - you might need to tune this
        double dy = t.at<double>(2) * 1.0; // Using Z as forward motion for vehicle
        
        // Extract yaw rotation from rotation matrix
        // For small rotations around Y axis (typical for ground vehicles)
        double dyaw = atan2(R.at<double>(2, 0), R.at<double>(0, 0));
        
        return {dx, dy, dyaw};
    }

    bool isIdentity(const cv::Mat& mat) {
        return cv::countNonZero(cv::abs(mat - cv::Mat::eye(4, 4, CV_64F)) > 1e-6) == 0;
    }
    
    // ========== EKF IMPLEMENTATION ==========
    
    void init_ekf() {
        // State: [x, y, yaw, vx, vy, vyaw]
        state_ = Eigen::VectorXd::Zero(6);
        
        // Initial covariance (high uncertainty)
        covariance_ = Eigen::MatrixXd::Identity(6, 6) * 10.0;
        covariance_(0,0) = 1.0;    // x position uncertainty
        covariance_(1,1) = 1.0;    // y position uncertainty  
        covariance_(2,2) = 0.1;    // yaw uncertainty
        covariance_(3,3) = 5.0;    // vx uncertainty
        covariance_(4,4) = 5.0;    // vy uncertainty
        covariance_(5,5) = 1.0;    // vyaw uncertainty
        
        // Process noise Q
        process_noise_ = Eigen::MatrixXd::Identity(6, 6);
        process_noise_ *= 0.1;     // Base process noise
        process_noise_(0,0) = 0.01; // x process noise
        process_noise_(1,1) = 0.01; // y process noise  
        process_noise_(2,2) = 0.005; // yaw process noise
        process_noise_(3,3) = 0.5;  // vx process noise
        process_noise_(4,4) = 0.5;  // vy process noise
        process_noise_(5,5) = 0.1;  // vyaw process noise
        
        // VO measurement noise R (for [dx, dy, dyaw] measurements)
        vo_noise_ = Eigen::MatrixXd::Identity(3, 3);
        vo_noise_(0,0) = 0.5;  // dx noise (VO has high uncertainty)
        vo_noise_(1,1) = 0.5;  // dy noise
        vo_noise_(2,2) = 0.1;  // dyaw noise
        
        // GPS measurement noise R (for [x, y, yaw] absolute measurements)  
        gps_noise_ = Eigen::MatrixXd::Identity(3, 3);
        gps_noise_(0,0) = 0.1;  // x noise (GPS more accurate)
        gps_noise_(1,1) = 0.1;  // y noise
        gps_noise_(2,2) = 0.05; // yaw noise
        
        ekf_initialized_ = true;
        gps_correction_interval_ = 2.0; // GPS correction every 2 seconds
        last_gps_time_ = 0.0;
        last_time_ = std::chrono::steady_clock::now();
        
        RCLCPP_INFO(this->get_logger(), "EKF initialized for VO-GPS fusion");
    }
    
    void predict_step(double dt) {
        if (!ekf_initialized_) return;
        
        // State transition model: x_k = F * x_{k-1}
        // [x, y, yaw, vx, vy, vyaw] with constant velocity model
        Eigen::MatrixXd F = Eigen::MatrixXd::Identity(6, 6);
        F(0, 3) = dt;  // x += vx * dt
        F(1, 4) = dt;  // y += vy * dt  
        F(2, 5) = dt;  // yaw += vyaw * dt
        
        // Predict state
        Eigen::VectorXd predicted_state = F * state_;
        
        // Predict covariance: P = F * P * F^T + Q
        covariance_ = F * covariance_ * F.transpose() + process_noise_ * dt;
        
        // Update state
        state_ = predicted_state;
        
        // Normalize yaw angle to [-π, π]
        while (state_(2) > M_PI) state_(2) -= 2.0 * M_PI;
        while (state_(2) < -M_PI) state_(2) += 2.0 * M_PI;
    }
    
    void update_with_vo(double dx, double dy, double dyaw) {
        if (!ekf_initialized_) return;
        
        // Measurement model for VO: z = H * x + v
        // We measure velocity-based changes, so H maps state to velocity
        Eigen::MatrixXd H = Eigen::MatrixXd::Zero(3, 6);
        H(0, 3) = 1.0;  // measure vx
        H(1, 4) = 1.0;  // measure vy  
        H(2, 5) = 1.0;  // measure vyaw
        
        // Expected measurement
        Eigen::VectorXd z_pred = H * state_;
        
        // Actual measurement (convert relative motion to velocities)
        Eigen::VectorXd z(3);
        z << dx * 10.0, dy * 10.0, dyaw * 10.0; // Scale factor for velocity estimation
        
        // Innovation
        Eigen::VectorXd y = z - z_pred;
        
        // Innovation covariance
        Eigen::MatrixXd S = H * covariance_ * H.transpose() + vo_noise_;
        
        // Kalman gain
        Eigen::MatrixXd K = covariance_ * H.transpose() * S.inverse();
        
        // Update state and covariance
        state_ = state_ + K * y;
        covariance_ = (Eigen::MatrixXd::Identity(6, 6) - K * H) * covariance_;
        
        // Update EKF pose (do NOT overwrite current_x_, current_y_, current_yaw_!)
        ekf_x_ = state_(0);
        ekf_y_ = state_(1); 
        ekf_yaw_ = state_(2);
    }
    
    void update_with_gps(double gps_x, double gps_y, double gps_yaw) {
        if (!ekf_initialized_) return;
        
        // Measurement model for GPS: direct position measurement
        Eigen::MatrixXd H = Eigen::MatrixXd::Zero(3, 6);
        H(0, 0) = 1.0;  // measure x
        H(1, 1) = 1.0;  // measure y
        H(2, 2) = 1.0;  // measure yaw
        
        // Expected measurement  
        Eigen::VectorXd z_pred = H * state_;
        
        // Actual measurement
        Eigen::VectorXd z(3);
        z << gps_x, gps_y, gps_yaw;
        
        // Innovation
        Eigen::VectorXd y = z - z_pred;
        
        // Normalize yaw innovation
        while (y(2) > M_PI) y(2) -= 2.0 * M_PI;
        while (y(2) < -M_PI) y(2) += 2.0 * M_PI;
        
        // Innovation covariance
        Eigen::MatrixXd S = H * covariance_ * H.transpose() + gps_noise_;
        
        // Kalman gain
        Eigen::MatrixXd K = covariance_ * H.transpose() * S.inverse();
        
        // Update state and covariance
        state_ = state_ + K * y;
        covariance_ = (Eigen::MatrixXd::Identity(6, 6) - K * H) * covariance_;
        
        // Update EKF pose (do NOT overwrite VO variables!)
        ekf_x_ = state_(0);
        ekf_y_ = state_(1);
        ekf_yaw_ = state_(2);
        
        // Normalize EKF yaw
        while (ekf_yaw_ > M_PI) ekf_yaw_ -= 2.0 * M_PI;
        while (ekf_yaw_ < -M_PI) ekf_yaw_ += 2.0 * M_PI;
        while (current_yaw_ < -M_PI) current_yaw_ += 2.0 * M_PI;
        
        RCLCPP_INFO(this->get_logger(), "GPS correction applied: (%.2f, %.2f, %.2f°)", 
                   gps_x, gps_y, gps_yaw * 180.0 / M_PI);
    }
    
    void ground_truth_callback(const geometry_msgs::msg::Pose2D::SharedPtr msg) {
        auto current_time = std::chrono::steady_clock::now();
        double time_since_start = std::chrono::duration<double>(current_time - last_time_).count();
        
        // Apply GPS correction periodically (simulating real GPS)
        if (time_since_start - last_gps_time_ >= gps_correction_interval_) {
            update_with_gps(msg->x, msg->y, msg->theta);
            last_gps_time_ = time_since_start;
        }
    }
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    std::string data_dir = "src/vodom_first/Kitti_Sequence_Larga";
    auto node = std::make_shared<VisualOdometryUli>(data_dir);
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
