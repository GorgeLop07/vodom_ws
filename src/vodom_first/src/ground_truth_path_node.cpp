#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/path.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/pose2_d.hpp>
#include <fstream>
#include <sstream>
#include <string>
#include <vector>
#include <cmath>
#include <iterator>

class GroundTruthPathNode : public rclcpp::Node {
public:
//Solo para checar que el commit se haya hecho bien
    GroundTruthPathNode()
    : Node("ground_truth_path_node"),
    poses_file_(declare_parameter<std::string>("poses_file", "src/vodom_first/Kitti_Sequence_Larga/poses.txt")),
      frame_id_(declare_parameter<std::string>("frame_id", "map")),
      roll_degrees_(90.0),   // Rotation around X axis
      pitch_degrees_(0.0),  // Rotation around Y axis - CHANGE THIS for pitch!
      yaw_degrees_(0.0)     // Rotation around Z axis
    {
        path_pub_ = this->create_publisher<nav_msgs::msg::Path>("/gt_path", 10);
        pose2d_pub_ = this->create_publisher<geometry_msgs::msg::Pose2D>("/gt_pose_2d", 10);
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(100),
            std::bind(&GroundTruthPathNode::publish_path, this));
        path_msg_.header.frame_id = frame_id_;
        load_poses();
        RCLCPP_INFO(this->get_logger(), "GroundTruthPathNode started. Loaded %zu poses. Rotations: Roll=%.1f° Pitch=%.1f° Yaw=%.1f°", 
                   poses_.size(), roll_degrees_, pitch_degrees_, yaw_degrees_);
    }

private:    
    void load_poses() {
        std::ifstream file(poses_file_);
        if (!file.is_open()) {
            RCLCPP_ERROR(this->get_logger(), "Could not open poses file: %s", poses_file_.c_str());
            return;
        }
        
        // Convert rotation angles to radians
        double roll_rad = roll_degrees_ * M_PI / 180.0;
        double pitch_rad = pitch_degrees_ * M_PI / 180.0;
        double yaw_rad = yaw_degrees_ * M_PI / 180.0;
        
        // Precompute sin/cos for efficiency
        double cos_r = cos(roll_rad), sin_r = sin(roll_rad);
        double cos_p = cos(pitch_rad), sin_p = sin(pitch_rad);
        double cos_y = cos(yaw_rad), sin_y = sin(yaw_rad);
        
        std::string line;
        int line_num = 0;
        while (std::getline(file, line)) {
            line_num++;
            std::istringstream iss(line);
            std::vector<double> values((std::istream_iterator<double>(iss)), std::istream_iterator<double>());
            RCLCPP_INFO(this->get_logger(), "Line %d: %zu values", line_num, values.size());
            if (values.size() == 12) {
                // Original coordinates
                double x = values[3];
                double y = values[7];
                double z = values[11];
                
                // Apply rotations in order: Roll (X) -> Pitch (Y) -> Yaw (Z)
                // Roll around X axis
                double y1 = y * cos_r - z * sin_r;
                double z1 = y * sin_r + z * cos_r;
                
                // Pitch around Y axis  
                double x2 = x * cos_p + z1 * sin_p;
                double z2 = -x * sin_p + z1 * cos_p;
                
                // Yaw around Z axis
                double x_final = x2 * cos_y - y1 * sin_y;
                double y_final = x2 * sin_y + y1 * cos_y;
                double z_final = z2;
                
                geometry_msgs::msg::PoseStamped pose;
                pose.header.frame_id = frame_id_;
                pose.pose.position.x = x_final;
                pose.pose.position.y = y_final;
                pose.pose.position.z = z_final;  // Keep rotated Z or set to 0.0 for 2D
                pose.pose.orientation.w = 1.0;
                poses_.push_back(pose);
            }
        }
    }

    void publish_path() {
        static size_t current_pose_index = 0;
        
        path_msg_.header.stamp = this->now();
        path_msg_.poses = poses_;
        path_pub_->publish(path_msg_);
        
        // Also publish current pose as Pose2D for EKF
        if (!poses_.empty() && current_pose_index < poses_.size()) {
            geometry_msgs::msg::Pose2D pose2d_msg;
            pose2d_msg.x = poses_[current_pose_index].pose.position.x;
            pose2d_msg.y = poses_[current_pose_index].pose.position.y;
            pose2d_msg.theta = 0.0;  // No orientation for now
            pose2d_pub_->publish(pose2d_msg);
            
            current_pose_index = (current_pose_index + 1) % poses_.size();
        }
    }

    std::string poses_file_;
    std::string frame_id_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
    rclcpp::Publisher<geometry_msgs::msg::Pose2D>::SharedPtr pose2d_pub_;
    nav_msgs::msg::Path path_msg_;
    std::vector<geometry_msgs::msg::PoseStamped> poses_;
    rclcpp::TimerBase::SharedPtr timer_;
    double roll_degrees_;   // Rotation around X axis
    double pitch_degrees_;  // Rotation around Y axis - USE THIS for pitch!
    double yaw_degrees_;    // Rotation around Z axis
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<GroundTruthPathNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
