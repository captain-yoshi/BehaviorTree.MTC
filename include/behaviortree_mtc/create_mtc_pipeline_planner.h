#pragma once

#include <behaviortree_cpp/bt_factory.h>
#include <behaviortree_ros2/ros_node_params.hpp>

namespace BT {
namespace MTC {

class CreateMTCPipelinePlanner : public BT::SyncActionNode
{
public:
  CreateMTCPipelinePlanner(const std::string& name, const BT::NodeConfig& config, const BT::RosNodeParams& params);

  BT::NodeStatus tick() override;
  static BT::PortsList providedPorts();

private:
  rclcpp::Node::SharedPtr node_;
};

}  // namespace MTC
}  // namespace BT
