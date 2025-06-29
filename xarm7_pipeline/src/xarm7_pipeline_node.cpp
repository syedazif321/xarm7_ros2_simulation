#include <rclcpp/rclcpp.hpp>
#include <xarm_msgs/srv/plan_joint.hpp>
#include <xarm_msgs/srv/plan_exec.hpp>
#include <yaml-cpp/yaml.h>
#include <fstream>
#include <msg_gazebo/srv/attach_detach.hpp>

class XArm7Pipeline : public rclcpp::Node {
public:
    XArm7Pipeline() : Node("xarm7_pipeline_node") {
        joint_plan_client_ = this->create_client<xarm_msgs::srv::PlanJoint>("xarm_joint_plan");
        exec_plan_client_ = this->create_client<xarm_msgs::srv::PlanExec>("xarm_exec_plan");
        attach_detach_client_ = this->create_client<msg_gazebo::srv::AttachDetach>("/AttachDetach");

        this->declare_parameter<std::string>("joint_file", "/home/azif/xarm7_ros2_simulation/robot_data/saved_joint_poses.yaml");

    
        pipeline_order_ = {
            "home",
            "look",
            "pick_pre",
            "pick",
            "attach",  
            "drop_post",
            "drop_pre",
            "drop",
            "detach",   
            "drop_pre",
            "drop_post",
            "up",
            "home"
        };

        std::string file_path;
        this->get_parameter("joint_file", file_path);

        for (const auto& step : pipeline_order_) {
            if (step == "attach") {
                call_attach_detach(true);
                continue;
            } else if (step == "detach") {
                call_attach_detach(false);
                continue;
            }

            std::vector<double> joint_values;
            if (!load_joint_from_yaml(file_path, step, joint_values)) {
                RCLCPP_ERROR(this->get_logger(), "Failed to load joint '%s' from YAML.", step.c_str());
                continue;
            }

            plan_and_execute(joint_values);
        }
    }

private:
    rclcpp::Client<xarm_msgs::srv::PlanJoint>::SharedPtr joint_plan_client_;
    rclcpp::Client<xarm_msgs::srv::PlanExec>::SharedPtr exec_plan_client_;
    rclcpp::Client<msg_gazebo::srv::AttachDetach>::SharedPtr attach_detach_client_;
    std::vector<std::string> pipeline_order_;

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

    void call_attach_detach(bool attach) {
        while (!attach_detach_client_->wait_for_service(std::chrono::seconds(1)))
            RCLCPP_WARN(this->get_logger(), "Waiting for AttachDetach service...");
    
        auto req = std::make_shared<msg_gazebo::srv::AttachDetach::Request>();
        req->model1 = "UF_ROBOT";
        req->link1 = "link7";
        req->model2 = "box_2";
        req->link2 = "link";
        req->attach = attach;
    
        auto future = attach_detach_client_->async_send_request(req);
        if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), future) != rclcpp::FutureReturnCode::SUCCESS) {
            RCLCPP_ERROR(this->get_logger(), "%s service call failed.", attach ? "Attach" : "Detach");
            return;
        }
    
        auto resp = future.get();
        if (resp->success) {
            RCLCPP_INFO(this->get_logger(), "%s succeeded: %s", attach ? "Attach" : "Detach", resp->message.c_str());
        } else {
            RCLCPP_ERROR(this->get_logger(), "%s failed: %s", attach ? "Attach" : "Detach", resp->message.c_str());
        }
    }
    
    
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<XArm7Pipeline>());
    rclcpp::shutdown();
    return 0;
}
