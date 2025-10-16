#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/path.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <fstream>
#include <iomanip>
#include <string>

class PathRecorder : public rclcpp::Node
{
public:
    PathRecorder() : Node("path_recorder")
    {
        // Declare parameters for topic names (CONFIGURABLE)
        // this->declare_parameter<std::string>("vo_topic", "/vo_path");
        this->declare_parameter<std::string>("vo_topic", "/ekf_fused_path");
        // this->declare_parameter<std::string>("vo_output_file", "vo_trajectory.txt");
        this->declare_parameter<std::string>("vo_output_file", "vo_trajectory.txt");
        
        // Get parameters
        //std::string vo_topic = this->get_parameter("vo_topic").as_string();
        std::string vo_topic = this->get_parameter("vo_topic").as_string();
        // vo_output_file_ = this->get_parameter("vo_output_file").as_string();
        vo_output_file_ = this->get_parameter("vo_output_file").as_string();
        
        // Subscribers
        // vo_subscriber_ = this->create_subscription<nav_msgs::msg::Path>(
        //     vo_topic, 10,
        //     std::bind(&PathRecorder::voCallback, this, std::placeholders::_1));
        
        gt_subscriber_ = this->create_subscription<nav_msgs::msg::Path>(
            vo_topic, 10,
            std::bind(&PathRecorder::gtCallback, this, std::placeholders::_1));
        
        // Open files for writing
        // vo_file_.open(vo_output_file_);
        gt_file_.open(vo_output_file_);
        
        // if (!vo_file_.is_open() || !gt_file_.is_open()) {
        //     RCLCPP_ERROR(this->get_logger(), "Failed to open output files!");
        //     rclcpp::shutdown();
        //     return;
        // }
        
        // Write headers (TUM format compatible with evo)
        // vo_file_ << "# timestamp tx ty tz qx qy qz qw\n";
        gt_file_ << "# timestamp tx ty tz qx qy qz qw\n";
        
        RCLCPP_INFO(this->get_logger(), "Path Recorder started!");
        //RCLCPP_INFO(this->get_logger(), "  VO topic: %s -> %s", vo_topic.c_str(), vo_output_file_.c_str());
        RCLCPP_INFO(this->get_logger(), "  GT topic: %s -> %s", vo_topic.c_str(), vo_output_file_.c_str());
        RCLCPP_INFO(this->get_logger(), "Press Ctrl+C to save and exit.");
    }
    
    ~PathRecorder()
    {
        // Close files
        // if (vo_file_.is_open()) {
        //     vo_file_.close();
        //     RCLCPP_INFO(this->get_logger(), "Saved VO trajectory with %zu poses to %s", 
        //                vo_pose_count_, vo_output_file_.c_str());
        //}
        if (gt_file_.is_open()) {
            gt_file_.close();
            RCLCPP_INFO(this->get_logger(), "Saved GT trajectory with %zu poses to %s", 
                       gt_pose_count_, vo_output_file_.c_str());
        }
    }

private:
    // void voCallback(const nav_msgs::msg::Path::SharedPtr msg)
    // {
    //     if (msg->poses.empty()) {
    //         return;
    //     }
        
    //     // Only save new poses (avoid duplicates)
    //     for (size_t i = last_vo_size_; i < msg->poses.size(); ++i) {
    //         const auto& pose = msg->poses[i];
    //         double timestamp = pose.header.stamp.sec + pose.header.stamp.nanosec * 1e-9;
            
    //         vo_file_ << std::fixed << std::setprecision(6) << timestamp << " "
    //                  << pose.pose.position.x << " "
    //                  << pose.pose.position.y << " "
    //                  << pose.pose.position.z << " "
    //                  << pose.pose.orientation.x << " "
    //                  << pose.pose.orientation.y << " "
    //                  << pose.pose.orientation.z << " "
    //                  << pose.pose.orientation.w << "\n";
            
    //         vo_pose_count_++;
    //     }
        
    //     last_vo_size_ = msg->poses.size();
        
    //     RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
    //                         "Recording: VO=%zu poses, GT=%zu poses", 
    //                         vo_pose_count_, gt_pose_count_);
    // }
    
    void gtCallback(const nav_msgs::msg::Path::SharedPtr msg)
    {
        if (msg->poses.empty()) {
            return;
        }
        
        // Only save new poses (avoid duplicates)
        for (size_t i = last_gt_size_; i < msg->poses.size(); ++i) {
            const auto& pose = msg->poses[i];
            double timestamp = pose.header.stamp.sec + pose.header.stamp.nanosec * 1e-9;
            
            gt_file_ << std::fixed << std::setprecision(6) << timestamp << " "
                     << pose.pose.position.x << " "
                     << pose.pose.position.y << " "
                     << pose.pose.position.z << " "
                     << pose.pose.orientation.x << " "
                     << pose.pose.orientation.y << " "
                     << pose.pose.orientation.z << " "
                     << pose.pose.orientation.w << "\n";
            
            gt_pose_count_++;
        }
        
        last_gt_size_ = msg->poses.size();
    }

private:
    //rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr vo_subscriber_;
    rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr gt_subscriber_;
    
    //std::ofstream vo_file_;
    std::ofstream gt_file_;
    
    //std::string vo_output_file_;
    std::string vo_output_file_;
    
    //size_t last_vo_size_ = 0;
    size_t last_gt_size_ = 0;
    //size_t vo_pose_count_ = 0;
    size_t gt_pose_count_ = 0;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<PathRecorder>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
