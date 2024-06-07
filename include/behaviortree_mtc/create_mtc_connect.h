#pragma once

#include <behaviortree_cpp/bt_factory.h>

namespace bt_mtc
{
class CreateMTCConnect : public BT::SyncActionNode
{
public:
  CreateMTCConnect(const std::string& name, const BT::NodeConfig& config);

  BT::NodeStatus tick() override;
  static BT::PortsList providedPorts();
};
}  // namespace bt_mtc