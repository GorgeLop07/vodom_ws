#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/path.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
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

namespace fs = std::filesystem;

class VisualOdometryUli : public rclcpp::Node {
public:
    VisualOdometryUli(const std::string& data_dir)
    : Node("visual_odo_Uli") {
        loadCalib(data_dir + "/calib.txt");
        loadPoses(data_dir + "/poses.txt");
        loadImages(data_dir + "/image_l"); //Cambiar a l en caso de primer dataset
        orb = cv::ORB::create(8000);//Intente subir el numero de features de 5000 a 8000 a ver que pasa XD 
    flann = cv::Ptr<cv::FlannBasedMatcher>(new cv::FlannBasedMatcher(new cv::flann::LshIndexParams(6, 12, 1)));
        path_pub_ = this->create_publisher<nav_msgs::msg::Path>("vo_path_uliXD", 10);
        process_and_publish();
    }

private:
    cv::Mat K, P;
    std::vector<cv::Mat> gt_poses;
    std::vector<cv::Mat> images;
    cv::Ptr<cv::ORB> orb;
    cv::Ptr<cv::FlannBasedMatcher> flann;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;

    void process_and_publish() {
        nav_msgs::msg::Path path_msg;
        path_msg.header.frame_id = "map";
        cv::Mat cur_pose = gt_poses[0].clone();
        for (size_t i = 0; i < gt_poses.size(); ++i) {
            if (i == 0) {
                cur_pose = gt_poses[i].clone();
            } else {
                std::vector<cv::Point2f> q1, q2;
                getMatches(i, q1, q2);
                cv::Mat transf = getPose(q1, q2);
                if (!isIdentity(transf)) {
                    cur_pose = cur_pose * transf.inv();
                }
            }
            geometry_msgs::msg::PoseStamped pose_stamped;
            pose_stamped.header.stamp = this->now();
            pose_stamped.header.frame_id = "map";
            pose_stamped.pose.position.x = cur_pose.at<double>(0, 3);
            pose_stamped.pose.position.y = cur_pose.at<double>(1, 3);
            pose_stamped.pose.position.z = cur_pose.at<double>(2, 3);
            // No orientation estimation, set to identity
            pose_stamped.pose.orientation.w = 1.0;
            pose_stamped.pose.orientation.x = 0.0;
            pose_stamped.pose.orientation.y = 0.0;
            pose_stamped.pose.orientation.z = 0.0;
            path_msg.poses.push_back(pose_stamped);
        }
        path_msg.header.stamp = this->now();
        path_pub_->publish(path_msg);
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
            // fallback: use all points if mask size doesn't match
            q1f = q1;
            q2f = q2;
        }
        cv::Mat R, t;
        cv::recoverPose(E, q1f, q2f, K, R, t);
        cv::Mat T = cv::Mat::eye(4, 4, CV_64F);
        R.copyTo(T(cv::Rect(0, 0, 3, 3)));
        t = t * 1.0; // No scale estimation
        for (int i = 0; i < 3; ++i) T.at<double>(i, 3) = t.at<double>(i);
        return T;
    }

    bool isIdentity(const cv::Mat& mat) {
        return cv::countNonZero(cv::abs(mat - cv::Mat::eye(4, 4, CV_64F)) > 1e-6) == 0;
    }
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    std::string data_dir = "src/vodom_first/KITTI_sequence_2";
    auto node = std::make_shared<VisualOdometryUli>(data_dir);
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
