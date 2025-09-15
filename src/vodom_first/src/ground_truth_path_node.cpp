#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/path.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <fstream>
#include <sstream>
#include <string>
#include <vector>

class GroundTruthPathNode : public rclcpp::Node {
public:
    GroundTruthPathNode()
    : Node("ground_truth_path_node"),
    poses_file_(declare_parameter<std::string>("poses_file", "src/vodom_first/KITTI_sequence_2/poses.txt")),
      frame_id_(declare_parameter<std::string>("frame_id", "map"))  
    {
        path_pub_ = this->create_publisher<nav_msgs::msg::Path>("/gt_path", 10);
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(100),
            std::bind(&GroundTruthPathNode::publish_path, this));
        path_msg_.header.frame_id = frame_id_;
        load_poses();
        RCLCPP_INFO(this->get_logger(), "GroundTruthPathNode started. Loaded %zu poses.", poses_.size());
    }

private:    
    void load_poses() {
        std::ifstream file(poses_file_);
        if (!file.is_open()) {
            RCLCPP_ERROR(this->get_logger(), "Could not open poses file: %s", poses_file_.c_str());
            return;
        }
        std::string line;
        int line_num = 0;
        while (std::getline(file, line)) {
            line_num++;
            std::istringstream iss(line);
            std::vector<double> values((std::istream_iterator<double>(iss)), std::istream_iterator<double>());
            RCLCPP_INFO(this->get_logger(), "Line %d: %zu values", line_num, values.size());
            if (values.size() == 12) {
                geometry_msgs::msg::PoseStamped pose;
                pose.header.frame_id = frame_id_;
                pose.pose.position.x = values[3];
                pose.pose.position.y = values[7];
                pose.pose.position.z = values[11];
                // Orientation is not set (identity quaternion)
                pose.pose.orientation.w = 1.0;
                poses_.push_back(pose);
            }
        }
    }

    void publish_path() {
        path_msg_.header.stamp = this->now();
        path_msg_.poses = poses_;
        path_pub_->publish(path_msg_);
    }

    std::string poses_file_;
    std::string frame_id_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
    nav_msgs::msg::Path path_msg_;
    std::vector<geometry_msgs::msg::PoseStamped> poses_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<GroundTruthPathNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
