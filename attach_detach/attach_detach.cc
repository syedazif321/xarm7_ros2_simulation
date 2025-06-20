#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <rclcpp/rclcpp.hpp>
#include "msg_gazebo/srv/attach_detach.hpp"

namespace gazebo
{
  class AttachDetachService : public WorldPlugin
  {
    private: rclcpp::Node::SharedPtr node_;
    private: rclcpp::Service<msg_gazebo::srv::AttachDetach>::SharedPtr service_;

    private: rclcpp::executors::SingleThreadedExecutor::SharedPtr executor_;
    private: std::shared_ptr<std::thread> executor_thread_;

    private: physics::WorldPtr world_;
    private: std::map<std::string, physics::JointPtr> joints_;

    public: void Load(physics::WorldPtr _world, sdf::ElementPtr /*_sdf*/) override
    {
      this->world_ = _world;

      // Initialize ROS 2 context if it's not already initialized
      if (!rclcpp::ok())
      {
        rclcpp::init(0, nullptr);
      }
      

      // Initialize ROS 2 node
      if (!rclcpp::ok())
      {
        RCLCPP_ERROR(rclcpp::get_logger("AttachDetachService"), "ROS 2 not initialized, plugin will not work.");
        return;
      }

      this->node_ = rclcpp::Node::make_shared("attach_detach_service");

      // Create the service
      this->service_ = this->node_->create_service<msg_gazebo::srv::AttachDetach>(
        "AttachDetach",
        std::bind(&AttachDetachService::HandleService, this, std::placeholders::_1, std::placeholders::_2));

      RCLCPP_INFO(this->node_->get_logger(), "\033[1;32mAttach/Detach service started.\033[0m");

      // \033[1;31m for red.
      // \033[1;32m for green.
      // \033[1;33m for yellow.
      // \033[1;34m for blue.
      // \033[1;35m for magenta.
      // \033[1;36m for cyan.
      // \033[0m to reset the color.

      // this->executor_ = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
      // this->executor_->add_node(this->node_);

      this->executor_thread_ = std::make_shared<std::thread>([this]() {
        rclcpp::spin(this->node_);
      });
    }

    // Called every simulation step
    // public: void OnUpdate()
    // {
    //   // Spin the node in a non-blocking way
    //   this->executor_->spin_some();
    // }

    private: void HandleService(
      const std::shared_ptr<msg_gazebo::srv::AttachDetach::Request> request,
      std::shared_ptr<msg_gazebo::srv::AttachDetach::Response> response)
    {
      if (!this->world_)
      {
        response->success = false;
        response->message = "World pointer is null.";
        return;
      }

      auto model1 = this->world_->ModelByName(request->model1);
      auto model2 = this->world_->ModelByName(request->model2);

      if (!model1 || !model2)
      {
        response->success = false;
        response->message = "One or both models not found.";
        return;
      }

      auto link1 = model1->GetLink(request->link1);
      auto link2 = model2->GetLink(request->link2);

      if (!link1 || !link2)
      {
        response->success = false;
        response->message = "One or both links not found.";
        return;
      }

      std::string jointName = request->model1 + "_" + request->link1 + "_to_" + request->model2 + "_" + request->link2;

      if (request->attach)
      {
        if (this->joints_.find(jointName) != this->joints_.end())
        {
          response->success = false;
          response->message = "Links are already attached.";
          return;
        }

        auto joint = this->world_->Physics()->CreateJoint("fixed", model1);
        joint->Load(link1, link2, ignition::math::Pose3d::Zero);
        joint->Init();

        this->joints_[jointName] = joint;

        response->success = true;
        response->message = "Links successfully attached.";
        RCLCPP_INFO(this->node_->get_logger(), "%s", response->message.c_str());
      }
      else
      {
        auto it = this->joints_.find(jointName);
        if (it == this->joints_.end())
        {
          response->success = false;
          response->message = "Links are not attached.";
          return;
        }

        it->second->Fini();
        this->joints_.erase(it);

        response->success = true;
        response->message = "Links successfully detached.";
        RCLCPP_INFO(this->node_->get_logger(), "%s", response->message.c_str());
      }
    }
  };

  GZ_REGISTER_WORLD_PLUGIN(AttachDetachService)
}
