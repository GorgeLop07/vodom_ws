#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/path.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>

class OdometryToPath : public rclcpp::Node
{
public:
    OdometryToPath() : Node("odometry_to_path")
    {
        // Subscribe to EKF odometry output
        odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "/input_odom", 10,
            std::bind(&OdometryToPath::odomCallback, this, std::placeholders::_1));
        
        // Publish path for visualization
        path_pub_ = this->create_publisher<nav_msgs::msg::Path>("/output_path", 10);
        
        // Initialize path
        ekf_path_.header.frame_id = "odom";
        
        RCLCPP_INFO(this->get_logger(), "🔄 EKF Odometry to Path Converter");
        RCLCPP_INFO(this->get_logger(), "📥 Input: /input_odom (nav_msgs/msg/Odometry)");
        RCLCPP_INFO(this->get_logger(), "📤 Output: /output_path (nav_msgs/msg/Path)");
        RCLCPP_INFO(this->get_logger(), "🔗 Will be remapped via launch file");
    }

private:
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
    nav_msgs::msg::Path ekf_path_;
    
    void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
    {
        // Convert odometry to pose stamped
        geometry_msgs::msg::PoseStamped pose_stamped;
        pose_stamped.header = msg->header;
        pose_stamped.pose = msg->pose.pose;
        
        // Add to path
        ekf_path_.poses.push_back(pose_stamped);
        ekf_path_.header.stamp = msg->header.stamp;
        
        // Publish path
        path_pub_->publish(ekf_path_);
        
        RCLCPP_INFO(this->get_logger(), "🔗 EKF Path updated! Total poses: %zu | Position: (%.2f, %.2f)", 
                   ekf_path_.poses.size(), 
                   pose_stamped.pose.position.x, 
                   pose_stamped.pose.position.y);
    }
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<OdometryToPath>();
    
    RCLCPP_INFO(node->get_logger(), "🛤️  Starting EKF Path Converter...");
    
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}