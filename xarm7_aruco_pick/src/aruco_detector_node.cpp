#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.hpp>

#include <opencv2/opencv.hpp>
#include <opencv2/aruco.hpp>

#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

class ArucoDetector : public rclcpp::Node {
public:
    ArucoDetector()
    : Node("aruco_detector_node")
    {
        RCLCPP_INFO(this->get_logger(), "Starting ArucoDetector Node");

        image_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
            "/color/image_raw", rclcpp::SensorDataQoS(),
            std::bind(&ArucoDetector::imageCallback, this, std::placeholders::_1)
        );

        camera_info_sub_ = this->create_subscription<sensor_msgs::msg::CameraInfo>(
            "/color/camera_info", rclcpp::SensorDataQoS(),
            std::bind(&ArucoDetector::cameraInfoCallback, this, std::placeholders::_1)
        );

        tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);

        dictionary_ = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_6X6_250);
        RCLCPP_INFO(this->get_logger(), "ArucoDetector ready.");
    }

private:
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
    rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr camera_info_sub_;
    std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

    cv::Mat camera_matrix_;
    cv::Mat dist_coeffs_;
    cv::Ptr<cv::aruco::Dictionary> dictionary_;

    bool camera_info_received_ = false;

    void cameraInfoCallback(const sensor_msgs::msg::CameraInfo::SharedPtr msg) {
        if (camera_info_received_) return;

        camera_matrix_ = cv::Mat(3, 3, CV_64F, const_cast<double*>(msg->k.data())).clone();
        dist_coeffs_ = cv::Mat(msg->d.size(), 1, CV_64F, const_cast<double*>(msg->d.data())).clone();
        camera_info_received_ = true;

        RCLCPP_INFO(this->get_logger(), "Camera info received and camera matrix initialized.");
    }

    void imageCallback(const sensor_msgs::msg::Image::SharedPtr msg) {
        if (!camera_info_received_) {
            RCLCPP_WARN(this->get_logger(), "Waiting for camera info...");
            return;
        }

        RCLCPP_INFO(this->get_logger(), "Image received, converting to OpenCV format");

        cv::Mat image;
        try {
            image = cv_bridge::toCvCopy(msg, "bgr8")->image;
        } catch (cv_bridge::Exception& e) {
            RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
            return;
        }

        std::vector<int> ids;
        std::vector<std::vector<cv::Point2f>> corners;
        cv::Ptr<cv::aruco::DetectorParameters> parameters = cv::aruco::DetectorParameters::create();

        cv::aruco::detectMarkers(image, dictionary_, corners, ids, parameters);

        RCLCPP_INFO(this->get_logger(), "Detected %zu ArUco markers", ids.size());

        if (!ids.empty()) {
            cv::aruco::drawDetectedMarkers(image, corners, ids);

            std::vector<cv::Vec3d> rvecs, tvecs;
            cv::aruco::estimatePoseSingleMarkers(corners, 0.05, camera_matrix_, dist_coeffs_, rvecs, tvecs);

            for (size_t i = 0; i < ids.size(); ++i) {
                cv::aruco::drawAxis(image, camera_matrix_, dist_coeffs_, rvecs[i], tvecs[i], 0.05);

                geometry_msgs::msg::TransformStamped tf_msg;
                tf_msg.header.stamp = msg->header.stamp;
                tf_msg.header.frame_id = "camera_color_optical_frame";
                tf_msg.child_frame_id = "aruco_marker_" + std::to_string(ids[i]);
                tf_msg.transform.translation.x = tvecs[i][0];
                tf_msg.transform.translation.y = tvecs[i][1];
                tf_msg.transform.translation.z = tvecs[i][2];

                tf2::Quaternion q;
                q.setRPY(rvecs[i][0], rvecs[i][1], rvecs[i][2]);
                tf_msg.transform.rotation.x = q.x();
                tf_msg.transform.rotation.y = q.y();
                tf_msg.transform.rotation.z = q.z();
                tf_msg.transform.rotation.w = q.w();

                tf_broadcaster_->sendTransform(tf_msg);
                RCLCPP_INFO(this->get_logger(), "Broadcasted TF for marker %d", ids[i]);
            }
        }

        cv::imshow("Aruco Detection", image);
        cv::waitKey(1);
    }
    
};

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ArucoDetector>());
    rclcpp::shutdown();
    return 0;
}
