#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <nav_msgs/msg/path.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <fstream>
#include <vector>
#include <sstream>
#include <chrono>

struct GTose {
    double timestamp;
    double x, y, z;
    double qx, qy, qz, qw;
    
    GTose(double t, double px, double py, double pz, double rx, double ry, double rz, double rw)
        : timestamp(t), x(px), y(py), z(pz), qx(rx), qy(ry), qz(rz), qw(rw) {}
};

class GTPublisher : public rclcpp::Node
{
public:
    GTPublisher() : Node("gt_pose_publisher"), current_index_(0)
    {
        // Load GT poses from LIO-SAM
        if (!loadGTPoses()) {
            RCLCPP_ERROR(this->get_logger(), "❌ Failed to load GT poses");
            return;
        }
        
        // Publishers
        gt_pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>("/gt_pose", 10);
        gt_path_pub_ = this->create_publisher<nav_msgs::msg::Path>("/gt_path", 10);
        
        // Timer for publishing at 30Hz (same as EKF)
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(33), // ~30Hz
            std::bind(&GTPublisher::publishGTPose, this));
        
        // Initialize path message
        gt_path_msg_.header.frame_id = "odom";
        
        // Calculate total duration for interpolation
        if (!gt_poses_.empty()) {
            start_time_ = this->get_clock()->now();
            total_duration_ = gt_poses_.back().timestamp - gt_poses_[0].timestamp;
            
            RCLCPP_INFO(this->get_logger(), "✅ GT Publisher initialized");
            RCLCPP_INFO(this->get_logger(), "📊 Loaded %zu GT poses from LIO-SAM", gt_poses_.size());
            RCLCPP_INFO(this->get_logger(), "⏱️  Total duration: %.2f seconds", total_duration_);
            RCLCPP_INFO(this->get_logger(), "📤 Publishing: /gt_pose (30Hz interpolated)");
            RCLCPP_INFO(this->get_logger(), "🛤️  Publishing: /gt_path");
        }
    }

private:
    rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr gt_pose_pub_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr gt_path_pub_;
    rclcpp::TimerBase::SharedPtr timer_;
    
    std::vector<GTose> gt_poses_;
    nav_msgs::msg::Path gt_path_msg_;
    size_t current_index_;
    rclcpp::Time start_time_;
    double total_duration_;
    
    bool loadGTPoses() {
        std::string file_path = "/home/jorgelop/Documents/VantTec_SDV_SWARM/SDV_Proyect/SDV_Software_Workspaces/vo_ws/src/vodom_first/trayectories/gt_trajectory.txt";
        std::ifstream file(file_path);
        
        if (!file.is_open()) {
            RCLCPP_ERROR(this->get_logger(), "❌ Cannot open GT file: %s", file_path.c_str());
            return false;
        }
        
        std::string line;
        double base_timestamp = 0.0;
        bool first_line = true;
        
        while (std::getline(file, line)) {
            // Skip empty lines and comments
            if (line.empty() || line[0] == '#') continue;
            
            std::istringstream iss(line);
            double timestamp, x, y, z, qx, qy, qz, qw;
            
            // Parse: timestamp x y z qx qy qz qw
            if (iss >> timestamp >> x >> y >> z >> qx >> qy >> qz >> qw) {
                if (first_line) {
                    base_timestamp = timestamp;
                    first_line = false;
                }
                
                // Normalize timestamp to start from 0
                double norm_timestamp = timestamp - base_timestamp;
                gt_poses_.emplace_back(norm_timestamp, x, y, z, qx, qy, qz, qw);
            }
        }
        
        file.close();
        
        if (gt_poses_.empty()) {
            RCLCPP_ERROR(this->get_logger(), "❌ No valid GT poses loaded");
            return false;
        }
        
        RCLCPP_INFO(this->get_logger(), "✅ Loaded %zu GT poses", gt_poses_.size());
        RCLCPP_INFO(this->get_logger(), "📊 First pose: t=%.3f, pos=(%.2f, %.2f, %.2f)", 
                   gt_poses_[0].timestamp, gt_poses_[0].x, gt_poses_[0].y, gt_poses_[0].z);
        RCLCPP_INFO(this->get_logger(), "📊 Last pose: t=%.3f, pos=(%.2f, %.2f, %.2f)", 
                   gt_poses_.back().timestamp, gt_poses_.back().x, gt_poses_.back().y, gt_poses_.back().z);
        
        return true;
    }
    
    GTose interpolatePose(double current_time) {
        // Find surrounding poses for interpolation
        if (current_time <= gt_poses_[0].timestamp) {
            return gt_poses_[0];
        }
        
        if (current_time >= gt_poses_.back().timestamp) {
            return gt_poses_.back();
        }
        
        // Binary search for efficiency
        size_t left = 0, right = gt_poses_.size() - 1;
        while (right - left > 1) {
            size_t mid = (left + right) / 2;
            if (gt_poses_[mid].timestamp <= current_time) {
                left = mid;
            } else {
                right = mid;
            }
        }
        
        // Linear interpolation between left and right
        const GTose& p1 = gt_poses_[left];
        const GTose& p2 = gt_poses_[right];
        
        double dt = p2.timestamp - p1.timestamp;
        double alpha = (current_time - p1.timestamp) / dt;
        
        // Interpolate position
        double x = p1.x + alpha * (p2.x - p1.x);
        double y = p1.y + alpha * (p2.y - p1.y);
        double z = p1.z + alpha * (p2.z - p1.z);
        
        // SLERP for quaternion interpolation (simplified linear for now)
        double qx = p1.qx + alpha * (p2.qx - p1.qx);
        double qy = p1.qy + alpha * (p2.qy - p1.qy);
        double qz = p1.qz + alpha * (p2.qz - p1.qz);
        double qw = p1.qw + alpha * (p2.qw - p1.qw);
        
        // Normalize quaternion
        double norm = sqrt(qx*qx + qy*qy + qz*qz + qw*qw);
        if (norm > 1e-6) {
            qx /= norm; qy /= norm; qz /= norm; qw /= norm;
        }
        
        return GTose(current_time, x, y, z, qx, qy, qz, qw);
    }
    
    void publishGTPose() {
        if (gt_poses_.empty()) return;
        
        // Calculate current time relative to start
        auto now = this->get_clock()->now();
        double elapsed = (now - start_time_).seconds();
        
        // Scale elapsed time to match GT duration (stretch 600 poses over rosbag duration)
        double scaled_time = elapsed; // Use actual time for now, can be scaled if needed
        
        // Get interpolated pose
        GTose current_pose = interpolatePose(scaled_time);
        
        // Create and publish PoseWithCovarianceStamped
        auto pose_msg = geometry_msgs::msg::PoseWithCovarianceStamped();
        pose_msg.header.stamp = now;
        pose_msg.header.frame_id = "odom";
        
        // Set position
        pose_msg.pose.pose.position.x = current_pose.x;
        pose_msg.pose.pose.position.y = current_pose.y;
        pose_msg.pose.pose.position.z = current_pose.z;
        
        // Set orientation
        pose_msg.pose.pose.orientation.x = current_pose.qx;
        pose_msg.pose.pose.orientation.y = current_pose.qy;
        pose_msg.pose.pose.orientation.z = current_pose.qz;
        pose_msg.pose.pose.orientation.w = current_pose.qw;
        
        // Set covariance - ALTA confianza en GT (domina sobre VO)
        for (int i = 0; i < 36; i++) {
            pose_msg.pose.covariance[i] = 0.0;
        }
        pose_msg.pose.covariance[0] = 0.5;   // x variance - ALTA confianza (GT domina posición)
        pose_msg.pose.covariance[7] = 0.5;   // y variance - ALTA confianza (GT domina posición)
        pose_msg.pose.covariance[14] = 1.0;  // z variance
        pose_msg.pose.covariance[21] = 0.1;  // roll variance (high confidence)
        pose_msg.pose.covariance[28] = 0.1;  // pitch variance
        pose_msg.pose.covariance[35] = 0.1;  // yaw variance
        
        gt_pose_pub_->publish(pose_msg);
        
        // Add to path and publish
        geometry_msgs::msg::PoseStamped pose_stamped;
        pose_stamped.header = pose_msg.header;
        pose_stamped.pose = pose_msg.pose.pose;
        gt_path_msg_.poses.push_back(pose_stamped);
        gt_path_msg_.header.stamp = now;
        
        gt_path_pub_->publish(gt_path_msg_);
        
        // Log periodically
        static int log_counter = 0;
        if (++log_counter % 90 == 0) { // Every 3 seconds at 30Hz
            RCLCPP_INFO(this->get_logger(), 
                       "📍 GT Pose: t=%.2f, pos=(%.2f, %.2f, %.2f), path_length=%zu", 
                       scaled_time, current_pose.x, current_pose.y, current_pose.z, 
                       gt_path_msg_.poses.size());
        }
    }
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<GTPublisher>();
    
    RCLCPP_INFO(node->get_logger(), "==============================================");
    RCLCPP_INFO(node->get_logger(), "📍 GT Pose Publisher (LIO-SAM Integration)");
    RCLCPP_INFO(node->get_logger(), "📤 Output: /gt_pose (PoseWithCovarianceStamped)");
    RCLCPP_INFO(node->get_logger(), "🛤️  Output: /gt_path");
    RCLCPP_INFO(node->get_logger(), "🎯 Purpose: Provide scale reference for EKF");
    RCLCPP_INFO(node->get_logger(), "==============================================");
    
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}