#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <moveit_msgs/srv/get_position_fk.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <yaml-cpp/yaml.h>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <cmath>

class PoseSaver : public rclcpp::Node
{
public:
    PoseSaver() : Node("pose_saver")
    {
        fk_client_ = this->create_client<moveit_msgs::srv::GetPositionFK>("/compute_fk");

        while (!fk_client_->wait_for_service(std::chrono::seconds(3)))
            RCLCPP_INFO(this->get_logger(), "Waiting for FK service...");

        auto joint_state = get_joint_states();
        if (joint_state.name.empty())
        {
            RCLCPP_ERROR(this->get_logger(), "Failed to get joint states.");
            return;
        }

        std::string pose_name;
        std::cout << "Enter pose name to save: ";
        std::cin >> pose_name;

        geometry_msgs::msg::Pose pose;
        if (!get_fk_pose(joint_state, pose))
        {
            RCLCPP_ERROR(this->get_logger(), "Failed to compute FK.");
            return;
        }

        save_pose(pose_name, pose);
    }

private:
    rclcpp::Client<moveit_msgs::srv::GetPositionFK>::SharedPtr fk_client_;

    sensor_msgs::msg::JointState get_joint_states()
    {
        sensor_msgs::msg::JointState latest;
        auto sub = this->create_subscription<sensor_msgs::msg::JointState>(
            "/joint_states", 10,
            [&](sensor_msgs::msg::JointState::SharedPtr msg)
            {
                latest = *msg;
            });

        rclcpp::sleep_for(std::chrono::milliseconds(500));
        rclcpp::spin_some(this->get_node_base_interface());
        return latest;
    }

    bool get_fk_pose(const sensor_msgs::msg::JointState &joint_state, geometry_msgs::msg::Pose &out_pose)
    {
        auto request = std::make_shared<moveit_msgs::srv::GetPositionFK::Request>();
        request->fk_link_names.push_back("link_tcp");
        request->robot_state.joint_state = joint_state;

        auto result = fk_client_->async_send_request(request);
        if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), result) ==
            rclcpp::FutureReturnCode::SUCCESS)
        {
            auto response = result.get();
            if (response->pose_stamped.empty())
                return false;

            out_pose = response->pose_stamped[0].pose;
            return true;
        }
        return false;
    }

    void save_pose(const std::string &name, const geometry_msgs::msg::Pose &pose)
    {
        std::string file_path = "/home/azif/xarm_ros2_simulation/robot_data/saved_poses.yaml";
        YAML::Node all_poses;

        std::ifstream fin(file_path);
        if (fin.good())
        {
            all_poses = YAML::Load(fin);
            fin.close();
        }
        std::vector<std::string> position = {
            to_fixed_string(pose.position.x),
            to_fixed_string(pose.position.y),
            to_fixed_string(pose.position.z)};
        std::vector<std::string> orientation = {
            to_fixed_string(pose.orientation.x),
            to_fixed_string(pose.orientation.y),
            to_fixed_string(pose.orientation.z),
            to_fixed_string(pose.orientation.w)};

        all_poses[name]["position"] = position;
        all_poses[name]["orientation"] = orientation;

        std::ofstream fout(file_path);
        fout << all_poses;
        fout.close();

        RCLCPP_INFO(this->get_logger(), "Pose '%s' saved successfully.", name.c_str());
    }

    std::string to_fixed_string(double val, int precision = 3)
    {
        std::ostringstream out;
        out << std::fixed << std::setprecision(precision) << val;
        return out.str();
    }

    std::string get_share_directory()
    {
        std::string cmd = "ros2 pkg prefix xarm7_pose_saver";
        FILE *fp = popen(cmd.c_str(), "r");
        if (!fp) return "";

        char buffer[512];
        std::string result;
        while (fgets(buffer, sizeof(buffer), fp)) result += buffer;
        pclose(fp);

        result.erase(result.find_last_not_of(" \n\r\t") + 1);
        return result + "/share/xarm7_pose_saver";
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<PoseSaver>();
    rclcpp::shutdown();
    return 0;
}
