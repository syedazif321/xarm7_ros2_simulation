#include <rclcpp/rclcpp.hpp>

// xarm_msgs service headers (use lowercase names!)
#include "xarm_msgs/srv/plan_joint.hpp"
#include "xarm_msgs/srv/plan_exec.hpp"
#include "xarm_msgs/srv/plan_single_straight.hpp"

// gazebo attach/detach service
#include "msg_gazebo/srv/attach_detach.hpp"

// MoveIt FK service
#include "moveit_msgs/srv/get_position_fk.hpp"

// Pose messages
#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "sensor_msgs/msg/joint_state.hpp"

// YAML parsing
#include "yaml-cpp/yaml.h"

// Standard
#include <fstream>
#include <vector>
#include <string>

class XArm7Pipeline : public rclcpp::Node {
public:
    XArm7Pipeline() : Node("xarm7_pipeline_node") {
        joint_plan_client_ = this->create_client<xarm_msgs::srv::PlanJoint>("xarm_joint_plan");
        exec_plan_client_ = this->create_client<xarm_msgs::srv::PlanExec>("xarm_exec_plan");
        straight_plan_client_ = this->create_client<xarm_msgs::srv::PlanSingleStraight>("xarm_straight_plan");
        attach_detach_client_ = this->create_client<msg_gazebo::srv::AttachDetach>("/AttachDetach");
        fk_client_ = this->create_client<moveit_msgs::srv::GetPositionFK>("/compute_fk");

        this->declare_parameter<std::string>("joint_file", "/home/azif/xarm_ros2_simulation/robot_data/saved_joint_poses.yaml");

        std::string joint_file;
        this->get_parameter("joint_file", joint_file);

        std::vector<std::string> steps = {
            "home", "up", "look", "pick_pre", "pick", "attach",
            "drop_post", "drop_pre", "drop", "detach", "drop_pre", "drop_post", "up", "home"
        };

        for (const auto& step : steps) {
            if (step == "attach") {
                call_attach_detach(true);
            }
            else if (step == "detach") {
                call_attach_detach(false);
            }
            else if ((step == "up" && prev_step_ == "home") || (step == "home" && prev_step_ == "up")) {
                std::vector<double> joint_from, joint_to;
                if (load_joint_from_yaml(joint_file, prev_step_, joint_from) && load_joint_from_yaml(joint_file, step, joint_to)) {
                    auto pose_from = compute_fk(joint_from);
                    auto pose_to = compute_fk(joint_to);
                    std::vector<geometry_msgs::msg::Pose> waypoints = {pose_from, pose_to};
                    plan_cartesian_path_and_execute(waypoints);
                } else {
                    RCLCPP_ERROR(this->get_logger(), "Failed to load joint for '%s' or '%s'", prev_step_.c_str(), step.c_str());
                }
            }
            else {
                std::vector<double> joints;
                if (load_joint_from_yaml(joint_file, step, joints)) {
                    plan_and_execute(joints);
                } else {
                    RCLCPP_ERROR(this->get_logger(), "Failed to load joint pose for '%s'", step.c_str());
                }
            }
            prev_step_ = step;
        }
    }

private:
    rclcpp::Client<xarm_msgs::srv::PlanJoint>::SharedPtr joint_plan_client_;
    rclcpp::Client<xarm_msgs::srv::PlanExec>::SharedPtr exec_plan_client_;
    rclcpp::Client<xarm_msgs::srv::PlanSingleStraight>::SharedPtr straight_plan_client_;
    rclcpp::Client<msg_gazebo::srv::AttachDetach>::SharedPtr attach_detach_client_;
    rclcpp::Client<moveit_msgs::srv::GetPositionFK>::SharedPtr fk_client_;
    std::string prev_step_;

    bool load_joint_from_yaml(const std::string& file_path, const std::string& key, std::vector<double>& joints) {
        try {
            YAML::Node config = YAML::LoadFile(file_path);
            if (!config[key]) return false;

            joints.clear();
            for (const auto& val : config[key]) joints.push_back(val.as<double>());
            return true;
        } catch (const std::exception& e) {
            RCLCPP_ERROR(this->get_logger(), "YAML error: %s", e.what());
            return false;
        }
    }

    geometry_msgs::msg::Pose compute_fk(const std::vector<double>& joint_values, const std::string& link_name = "link_tcp") {
        geometry_msgs::msg::Pose result;
        auto req = std::make_shared<moveit_msgs::srv::GetPositionFK::Request>();
        req->fk_link_names.push_back(link_name);
        req->robot_state.joint_state.name = {"joint1", "joint2", "joint3", "joint4", "joint5", "joint6", "joint7"};
        req->robot_state.joint_state.position = joint_values;

        while (!fk_client_->wait_for_service(std::chrono::seconds(1)))
            RCLCPP_WARN(this->get_logger(), "Waiting for FK service...");

        auto future = fk_client_->async_send_request(req);
        if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), future) == rclcpp::FutureReturnCode::SUCCESS) {
            auto res = future.get();
            if (!res->pose_stamped.empty()) {
                result = res->pose_stamped[0].pose;
            } else {
                RCLCPP_ERROR(this->get_logger(), "FK returned no poses");
            }
        } else {
            RCLCPP_ERROR(this->get_logger(), "FK service call failed");
        }
        return result;
    }

    void plan_and_execute(const std::vector<double>& joint_values) {
        auto req = std::make_shared<xarm_msgs::srv::PlanJoint::Request>();
        req->target = joint_values;

        while (!joint_plan_client_->wait_for_service(std::chrono::seconds(1)))
            RCLCPP_WARN(this->get_logger(), "Waiting for joint plan service...");

        auto future = joint_plan_client_->async_send_request(req);
        if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), future) != rclcpp::FutureReturnCode::SUCCESS || !future.get()->success) {
            RCLCPP_ERROR(this->get_logger(), "Joint planning failed.");
            return;
        }

        auto exec_req = std::make_shared<xarm_msgs::srv::PlanExec::Request>();
        exec_req->wait = true;

        auto future_exec = exec_plan_client_->async_send_request(exec_req);
        rclcpp::spin_until_future_complete(this->get_node_base_interface(), future_exec);
    }

    void plan_cartesian_path_and_execute(const std::vector<geometry_msgs::msg::Pose>& poses) {
        if (poses.empty()) return;

        auto req = std::make_shared<xarm_msgs::srv::PlanSingleStraight::Request>();
        req->target = poses.back();

        while (!straight_plan_client_->wait_for_service(std::chrono::seconds(1)))
            RCLCPP_WARN(this->get_logger(), "Waiting for Cartesian planning service...");

        auto future = straight_plan_client_->async_send_request(req);
        if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), future) != rclcpp::FutureReturnCode::SUCCESS || !future.get()->success) {
            RCLCPP_ERROR(this->get_logger(), "Cartesian planning failed.");
            return;
        }

        auto exec_req = std::make_shared<xarm_msgs::srv::PlanExec::Request>();
        exec_req->wait = true;
        auto future_exec = exec_plan_client_->async_send_request(exec_req);
        rclcpp::spin_until_future_complete(this->get_node_base_interface(), future_exec);
    }

    void call_attach_detach(bool attach) {
        auto req = std::make_shared<msg_gazebo::srv::AttachDetach::Request>();
        req->model1 = "UF_ROBOT";
        req->link1 = "link7";
        req->model2 = "box_2";
        req->link2 = "link";
        req->attach = attach;

        while (!attach_detach_client_->wait_for_service(std::chrono::seconds(1)))
            RCLCPP_WARN(this->get_logger(), "Waiting for AttachDetach service...");

        auto future = attach_detach_client_->async_send_request(req);
        rclcpp::spin_until_future_complete(this->get_node_base_interface(), future);
    }
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<XArm7Pipeline>());
    rclcpp::shutdown();
    return 0;
}