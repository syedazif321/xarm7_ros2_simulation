#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <yaml-cpp/yaml.h>
#include <fstream>
#include <iostream>
#include <iomanip>
#include <map>

class PoseSaver : public rclcpp::Node {
public:
    PoseSaver() : Node("joint_saver") {
        this->declare_parameter<std::string>("save_path", "/home/azif/xarm7_ros2_simulation/robot_data/saved_joint_poses.yaml");
        this->get_parameter("save_path", file_path_);

        std::cout << "Enter pose name to save: ";
        std::cin >> pose_name_;

        sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
            "/joint_states", 10,
            std::bind(&PoseSaver::joint_callback, this, std::placeholders::_1));
    }

private:
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr sub_;
    std::string pose_name_;
    std::string file_path_;
    bool saved_ = false;

    void joint_callback(const sensor_msgs::msg::JointState::SharedPtr msg) {
        if (saved_) return;

        // Desired joint order
        std::vector<std::string> expected_order = {
            "joint1", "joint2", "joint3", "joint4", "joint5", "joint6", "joint7"
        };

        // Map current joint names to positions
        std::map<std::string, double> joint_map;
        for (size_t i = 0; i < msg->name.size(); ++i) {
            joint_map[msg->name[i]] = msg->position[i];
        }

        // Reorder values based on expected joint order
        std::vector<double> ordered_values;
        for (const auto& joint_name : expected_order) {
            if (joint_map.find(joint_name) != joint_map.end()) {
                ordered_values.push_back(joint_map[joint_name]);
            } else {
                RCLCPP_ERROR(this->get_logger(), "Missing joint: %s", joint_name.c_str());
                return;
            }
        }

        // Load YAML
        YAML::Node all_poses;
        std::ifstream fin(file_path_);
        if (fin.good()) {
            all_poses = YAML::Load(fin);
            fin.close();
        }

        // Save new pose
        all_poses[pose_name_] = ordered_values;

        std::ofstream fout(file_path_);
        fout << all_poses;
        fout.close();

        RCLCPP_INFO(this->get_logger(), "Joint values '%s' saved successfully at '%s'.",
                    pose_name_.c_str(), file_path_.c_str());
        saved_ = true;
    }
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<PoseSaver>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
