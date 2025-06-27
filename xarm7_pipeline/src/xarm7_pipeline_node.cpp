#include <rclcpp/rclcpp.hpp>
#include <xarm_msgs/srv/plan_joint.hpp>
#include <xarm_msgs/srv/plan_exec.hpp>
#include <yaml-cpp/yaml.h>
#include <fstream>

class XArm7Pipeline : public rclcpp::Node {
public:
    XArm7Pipeline() : Node("xarm7_pipeline_node") {
        joint_plan_client_ = this->create_client<xarm_msgs::srv::PlanJoint>("xarm_joint_plan");
        exec_plan_client_ = this->create_client<xarm_msgs::srv::PlanExec>("xarm_exec_plan");

        this->declare_parameter<std::string>("joint_file", "/home/azif/xarm_ros2_simulation/robot_data/saved_joint_poses.yaml");
        std::vector<std::string> targets = {"pose1", "pose2", "pose3"};
        this->declare_parameter("target_names", targets);

        std::string file_path;
        this->get_parameter("joint_file", file_path);
        this->get_parameter("target_names", targets);

        for (const auto& name : targets) {
            std::vector<double> joint_values;
            if (!load_joint_from_yaml(file_path, name, joint_values)) {
                RCLCPP_ERROR(this->get_logger(), "Failed to load joint '%s' from YAML.", name.c_str());
                continue;
            }
            plan_and_execute(joint_values);
        }
    }

private:
    rclcpp::Client<xarm_msgs::srv::PlanJoint>::SharedPtr joint_plan_client_;
    rclcpp::Client<xarm_msgs::srv::PlanExec>::SharedPtr exec_plan_client_;

    bool load_joint_from_yaml(const std::string& file_path, const std::string& key, std::vector<double>& joints) {
        try {
            YAML::Node config = YAML::LoadFile(file_path);
            if (!config[key]) return false;

            joints.clear();
            for (const auto& val : config[key]) joints.push_back(val.as<double>());
            return true;
        } catch (const std::exception& e) {
            RCLCPP_ERROR(this->get_logger(), "YAML Exception: %s", e.what());
            return false;
        }
    }

    void plan_and_execute(const std::vector<double>& joint_values) {
        auto req = std::make_shared<xarm_msgs::srv::PlanJoint::Request>();
        req->target = joint_values;

        while (!joint_plan_client_->wait_for_service(std::chrono::seconds(1)))
            RCLCPP_WARN(this->get_logger(), "Waiting for joint plan service...");

        auto future_plan = joint_plan_client_->async_send_request(req);
        if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), future_plan) != rclcpp::FutureReturnCode::SUCCESS || !future_plan.get()->success) {
            RCLCPP_ERROR(this->get_logger(), "Joint planning failed.");
            return;
        }

        auto exec_req = std::make_shared<xarm_msgs::srv::PlanExec::Request>();
        exec_req->wait = true;

        while (!exec_plan_client_->wait_for_service(std::chrono::seconds(1)))
            RCLCPP_WARN(this->get_logger(), "Waiting for exec plan service...");

        auto future_exec = exec_plan_client_->async_send_request(exec_req);
        if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), future_exec) != rclcpp::FutureReturnCode::SUCCESS || !future_exec.get()->success) {
            RCLCPP_ERROR(this->get_logger(), "Execution failed.");
        } else {
            RCLCPP_INFO(this->get_logger(), "Execution succeeded.");
        }
    }
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<XArm7Pipeline>());
    rclcpp::shutdown();
    return 0;
}
