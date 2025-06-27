#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <opencv2/aruco.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>


class ArucoDetector : public rclcpp::Node
{
public:
  ArucoDetector()
  : Node("aruco_detector_node")
  {
    RCLCPP_INFO(this->get_logger(), "Starting ArucoDetector Node");

    image_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
      "/color/image_raw", 10,
      std::bind(&ArucoDetector::image_callback, this, std::placeholders::_1));

    camera_info_sub_ = this->create_subscription<sensor_msgs::msg::CameraInfo>(
      "/color/camera_info", 10,
      std::bind(&ArucoDetector::camera_info_callback, this, std::placeholders::_1));

    tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);

    // Use the 4x4_1000 dictionary as requested
    dictionary_ = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_4X4_1000);

    camera_info_received_ = false;

    RCLCPP_INFO(this->get_logger(), "ArucoDetector ready.");
  }

private:
  void camera_info_callback(const sensor_msgs::msg::CameraInfo::SharedPtr msg)
  {
    // Copy camera intrinsic parameters to cv::Mat 3x3
    camera_matrix_ = cv::Mat(3, 3, CV_64F, const_cast<double*>(msg->k.data())).clone();

    // Copy distortion coefficients vector Nx1
    dist_coeffs_ = cv::Mat(msg->d.size(), 1, CV_64F, const_cast<double*>(msg->d.data())).clone();

    camera_info_received_ = true;

    RCLCPP_INFO(this->get_logger(), "Camera info received and initialized.");
  }

  void image_callback(const sensor_msgs::msg::Image::SharedPtr msg)
  {
    if (!camera_info_received_)
    {
      RCLCPP_WARN(this->get_logger(), "Waiting for camera info...");
      return;
    }

    RCLCPP_INFO(this->get_logger(), "Image received, converting to OpenCV format");

    cv_bridge::CvImagePtr cv_ptr;
    try {
      cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    }
    catch (cv_bridge::Exception& e) {
      RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
      return;
    }

    // Detect markers
    std::vector<int> marker_ids;
    std::vector<std::vector<cv::Point2f>> marker_corners, rejected_candidates;

    cv::aruco::DetectorParameters detectorParams;
    auto detectorParamsPtr = cv::aruco::DetectorParameters::create();

    cv::aruco::detectMarkers(
      cv_ptr->image,
      dictionary_,
      marker_corners,
      marker_ids,
      detectorParamsPtr,
      rejected_candidates);

    RCLCPP_INFO(this->get_logger(), "Detected %zu ArUco markers", marker_ids.size());

    if (marker_ids.empty()) {
      return;
    }
    cv::aruco::drawDetectedMarkers(cv_ptr->image, marker_corners, marker_ids);

    // Estimate pose for each detected marker
    // Assuming your marker size (side length) in meters, update this to real size!
    const float marker_length_meters = 0.05f;  

    std::vector<cv::Vec3d> rvecs, tvecs;
    cv::aruco::estimatePoseSingleMarkers(marker_corners, marker_length_meters, camera_matrix_, dist_coeffs_, rvecs, tvecs);

    for (size_t i = 0; i < marker_ids.size(); ++i)
    {
      // Draw axis for visualization
      cv::aruco::drawAxis(cv_ptr->image, camera_matrix_, dist_coeffs_, rvecs[i], tvecs[i], marker_length_meters * 0.5f);

      // Publish TF for each detected marker
      geometry_msgs::msg::TransformStamped marker_tf;
      marker_tf.header.stamp = this->now();
      marker_tf.header.frame_id = "camera_color_optical_frame";  // Change if your camera frame is different
      marker_tf.child_frame_id = "aruco_marker_" + std::to_string(marker_ids[i]);

      // Convert rotation vector to quaternion
      cv::Mat rotation_matrix;
      cv::Rodrigues(rvecs[i], rotation_matrix);
      tf2::Matrix3x3 tf3d(
        rotation_matrix.at<double>(0, 0), rotation_matrix.at<double>(0, 1), rotation_matrix.at<double>(0, 2),
        rotation_matrix.at<double>(1, 0), rotation_matrix.at<double>(1, 1), rotation_matrix.at<double>(1, 2),
        rotation_matrix.at<double>(2, 0), rotation_matrix.at<double>(2, 1), rotation_matrix.at<double>(2, 2));

      tf2::Quaternion quat;
      tf3d.getRotation(quat);

      marker_tf.transform.translation.x = tvecs[i][0];
      marker_tf.transform.translation.y = tvecs[i][1];
      marker_tf.transform.translation.z = tvecs[i][2];
      marker_tf.transform.rotation.x = quat.x();
      marker_tf.transform.rotation.y = quat.y();
      marker_tf.transform.rotation.z = quat.z();
      marker_tf.transform.rotation.w = quat.w();

      tf_broadcaster_->sendTransform(marker_tf);

      RCLCPP_INFO(this->get_logger(), "Published TF for marker ID %d", marker_ids[i]);
    }

    // Optional: show the image with detections for debugging
    cv::imshow("Aruco Detection", cv_ptr->image);
    cv::waitKey(1);
  }

  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
  rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr camera_info_sub_;

  cv::Mat camera_matrix_;
  cv::Mat dist_coeffs_;
  bool camera_info_received_;

  cv::Ptr<cv::aruco::Dictionary> dictionary_;
  std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<ArucoDetector>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
