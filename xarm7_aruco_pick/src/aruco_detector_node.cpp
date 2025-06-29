#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include "geometry_msgs/msg/transform_stamped.hpp"
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2_eigen/tf2_eigen.hpp>
#include <Eigen/Dense>
#include <opencv2/opencv.hpp>
#include <opencv2/aruco.hpp>

class ArucoDetector : public rclcpp::Node
{
public:
    ArucoDetector()
        : Node("aruco_detector_node"), tf_buffer_(this->get_clock()), tf_listener_(tf_buffer_)
    {
        it_ = std::make_shared<image_transport::ImageTransport>(shared_from_this());
        image_sub_ = it_->subscribe("/color/image_raw", 1, std::bind(&ArucoDetector::image_callback, this, std::placeholders::_1));
        tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

        pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("aruco_pose", 10);

        RCLCPP_INFO(this->get_logger(), "ArucoDetector node initialized.");
    }

private:
    image_transport::Subscriber image_sub_;
    std::shared_ptr<image_transport::ImageTransport> it_;
    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    tf2_ros::Buffer tf_buffer_;
    tf2_ros::TransformListener tf_listener_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_pub_;

    void image_callback(const sensor_msgs::msg::Image::ConstSharedPtr &msg)
    {
        cv::Mat image;
        try {
            image = cv_bridge::toCvShare(msg, "bgr8")->image;
        } catch (cv_bridge::Exception &e) {
            RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
            return;
        }

        cv::Ptr<cv::aruco::Dictionary> dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_4X4_100);
        std::vector<int> ids;
        std::vector<std::vector<cv::Point2f>> corners;
        cv::aruco::detectMarkers(image, dictionary, corners, ids);

        if (ids.empty()) return;

        RCLCPP_INFO(this->get_logger(), "Detected marker ID: %d", ids[0]);

        // Hardcoded camera intrinsics for simulation; replace with actual
        cv::Mat cameraMatrix = (cv::Mat1d(3, 3) << 615.0, 0.0, 320.0,
                                                    0.0, 615.0, 240.0,
                                                    0.0, 0.0, 1.0);
        cv::Mat distCoeffs = cv::Mat::zeros(1, 5, CV_64F);

        std::vector<cv::Vec3d> rvecs, tvecs;
        cv::aruco::estimatePoseSingleMarkers(corners, 1.0, cameraMatrix, distCoeffs, rvecs, tvecs);

        cv::Vec3d rvec = rvecs[0];
        cv::Vec3d tvec = tvecs[0];

        cv::Mat R;
        cv::Rodrigues(rvec, R);
        Eigen::Matrix3d rot;
        for (int i = 0; i < 3; ++i)
            for (int j = 0; j < 3; ++j)
                rot(i, j) = R.at<double>(i, j);
        Eigen::Translation3d trans(tvec[0], tvec[1], tvec[2]);
        Eigen::Affine3d T_camera_aruco = trans * rot;

        geometry_msgs::msg::TransformStamped tf_base_camera;
        try {
            tf_base_camera = tf_buffer_.lookupTransform("link_base", "camera_color_optical_frame", tf2::TimePointZero);
        } catch (const tf2::TransformException &ex) {
            RCLCPP_WARN(this->get_logger(), "TF lookup failed: %s", ex.what());
            return;
        }

        Eigen::Affine3d T_base_camera = tf2::transformToEigen(tf_base_camera.transform);
        Eigen::Affine3d T_base_aruco = T_base_camera * T_camera_aruco;

        geometry_msgs::msg::TransformStamped tf_msg;
        tf_msg.header.stamp = msg->header.stamp;
        tf_msg.header.frame_id = "link_base";
        tf_msg.child_frame_id = "aruco_marker";
        tf_msg.transform = tf2::eigenToTransform(T_base_aruco).transform;

        tf_broadcaster_->sendTransform(tf_msg);
        RCLCPP_INFO(this->get_logger(), "Published TF from link_base to aruco_marker.");

        geometry_msgs::msg::PoseStamped pose_msg;
        pose_msg.header = tf_msg.header;
        pose_msg.pose.position.x = tf_msg.transform.translation.x;
        pose_msg.pose.position.y = tf_msg.transform.translation.y;
        pose_msg.pose.position.z = tf_msg.transform.translation.z;
        pose_msg.pose.orientation = tf_msg.transform.rotation;
        pose_pub_->publish(pose_msg);
        RCLCPP_INFO(this->get_logger(), "Published PoseStamped for aruco_marker.");
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ArucoDetector>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
