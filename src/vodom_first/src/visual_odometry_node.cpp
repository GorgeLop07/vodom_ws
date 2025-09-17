#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/path.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <opencv2/opencv.hpp>
#include <filesystem>
#include <string>
#include <vector>

using std::placeholders::_1;
namespace fs = std::filesystem;

class VisualOdometryNode : public rclcpp::Node {
public:
    VisualOdometryNode()
    : Node("visual_odometry_node"),
    image_folder_(declare_parameter<std::string>("image_folder", "/home/jorgelop/vo_ws/src/vodom_first/KITTI_sequence_2/image_l")),
      frame_id_(declare_parameter<std::string>("frame_id", "map"))
    {
        path_pub_ = this->create_publisher<nav_msgs::msg::Path>("/vo_path", 10);
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(100),
            std::bind(&VisualOdometryNode::process_next_image, this));
        path_msg_.header.frame_id = frame_id_;
        load_image_paths();
        RCLCPP_INFO(this->get_logger(), "VisualOdometryNode started. Loaded %zu images.", image_paths_.size());
    }

private:
    void load_image_paths() {
        for (const auto& entry : fs::directory_iterator(image_folder_)) {
            if (entry.path().extension() == ".png") {
                image_paths_.push_back(entry.path().string());
            }
        }
        std::sort(image_paths_.begin(), image_paths_.end());
    }

    void process_next_image() {
        if (current_idx_ >= image_paths_.size()) {
            RCLCPP_INFO(this->get_logger(), "All images processed.");
            // Stop the timer to avoid further publishing
            if (timer_) timer_->cancel();
            return;
        }
        cv::Mat img = cv::imread(image_paths_[current_idx_], cv::IMREAD_GRAYSCALE);
        if (img.empty()) {
            RCLCPP_WARN(this->get_logger(), "Could not read image: %s", image_paths_[current_idx_].c_str());
            current_idx_++;
            return;
        }
        if (current_idx_ == 0) {
            prev_img_ = img;
            prev_R_ = cv::Mat::eye(3,3,CV_64F);
            prev_t_ = cv::Mat::zeros(3,1,CV_64F);
            current_idx_++;
            return;
        }
        // Feature detection and matching
        std::vector<cv::KeyPoint> kp1, kp2;
        cv::Mat desc1, desc2;
        auto orb = cv::ORB::create(5000);
        orb->detectAndCompute(prev_img_, cv::noArray(), kp1, desc1);
        orb->detectAndCompute(img, cv::noArray(), kp2, desc2);
        std::vector<cv::DMatch> matches;
        if (!desc1.empty() && !desc2.empty()) {
            cv::BFMatcher matcher(cv::NORM_HAMMING);
            matcher.match(desc1, desc2, matches);
        }
        // Extract matched points
        std::vector<cv::Point2f> pts1, pts2;
        for (auto& m : matches) {
            pts1.push_back(kp1[m.queryIdx].pt);
            pts2.push_back(kp2[m.trainIdx].pt);
        }
        // Camera intrinsics (KITTI default)
        double focal = 718.856;
        cv::Point2d pp(607.1928, 185.2157);
        if (pts1.size() >= 8) {
            cv::Mat E, R, t, mask;
            E = cv::findEssentialMat(pts2, pts1, focal, pp, cv::RANSAC, 0.999, 1.0, mask);
            if (!E.empty()) {
                cv::recoverPose(E, pts2, pts1, R, t, focal, pp, mask);
                prev_t_ = prev_t_ + prev_R_ * t;
                prev_R_ = R * prev_R_;
                geometry_msgs::msg::PoseStamped pose;
                pose.header.stamp = this->now();
                pose.header.frame_id = frame_id_;
                pose.pose.position.x = prev_t_.at<double>(0);
                pose.pose.position.y = prev_t_.at<double>(1);
                pose.pose.position.z = prev_t_.at<double>(2);
                // Orientation as quaternion
                cv::Mat R_ = prev_R_;
                cv::Mat rvec;
                cv::Rodrigues(R_, rvec);
                double angle = cv::norm(rvec);
                cv::Vec3d axis(0,0,0);
                if (angle > 1e-6) {
                    axis[0] = rvec.at<double>(0) / angle;
                    axis[1] = rvec.at<double>(1) / angle;
                    axis[2] = rvec.at<double>(2) / angle;
                }
                pose.pose.orientation.x = axis[0] * sin(angle/2.0);
                pose.pose.orientation.y = axis[1] * sin(angle/2.0);
                pose.pose.orientation.z = axis[2] * sin(angle/2.0);
                pose.pose.orientation.w = cos(angle/2.0);
                path_msg_.poses.push_back(pose);
                RCLCPP_INFO(this->get_logger(), "Added pose %zu: (%.3f, %.3f, %.3f)", path_msg_.poses.size(), pose.pose.position.x, pose.pose.position.y, pose.pose.position.z);
                path_msg_.header.stamp = this->now();
                // Only publish when a new pose is added
                path_pub_->publish(path_msg_);
                RCLCPP_INFO(this->get_logger(), "Published path with %zu poses", path_msg_.poses.size());
            }
        }
        prev_img_ = img;
        current_idx_++;
    }

    std::string image_folder_;
    std::string frame_id_;
    std::vector<std::string> image_paths_;
    size_t current_idx_ = 0;
    cv::Mat prev_img_;
    cv::Mat prev_R_, prev_t_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
    nav_msgs::msg::Path path_msg_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<VisualOdometryNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
