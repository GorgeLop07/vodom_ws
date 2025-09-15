#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/path.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>


//ESTOS NODOS SOLAMENTE SON PARA DEBUG


class MinimalPathPublisher : public rclcpp::Node {
public:
    MinimalPathPublisher() : Node("minimal_path_publisher") {
        path_pub_ = this->create_publisher<nav_msgs::msg::Path>("/vo_path", 10);
        timer_ = this->create_wall_timer(
            std::chrono::seconds(1),
            std::bind(&MinimalPathPublisher::publish_path, this));
        path_msg_.header.frame_id = "map";
        // Create a simple path with 3 poses
        for (int i = 0; i < 3; ++i) {
            geometry_msgs::msg::PoseStamped pose;
            pose.header.frame_id = "map";
            pose.pose.position.x = i;
            pose.pose.position.y = i * 0.5;
            pose.pose.position.z = 0.0;
            pose.pose.orientation.w = 1.0;
            path_msg_.poses.push_back(pose);
        }
    }
private:
    void publish_path() {
        path_msg_.header.stamp = this->now();
        path_pub_->publish(path_msg_);
        RCLCPP_INFO(this->get_logger(), "Published minimal path with %zu poses", path_msg_.poses.size());
    }
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
    nav_msgs::msg::Path path_msg_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<MinimalPathPublisher>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
