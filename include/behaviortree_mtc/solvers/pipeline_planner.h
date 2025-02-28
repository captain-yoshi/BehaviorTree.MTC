#pragma once

#include <behaviortree_cpp/bt_factory.h>

namespace BT {
namespace MTC {

class PipelinePlanner : public BT::SyncActionNode
{
public:
  PipelinePlanner(const std::string& name, const BT::NodeConfig& config);

  BT::NodeStatus tick() override;
  static BT::PortsList providedPorts();
};

}  // namespace MTC
}  // namespace BT
