#pragma once

#include <behaviortree_cpp/bt_factory.h>

namespace bt_mtc
{
class CreateMTCGeneratePlacePose : public BT::SyncActionNode
{
public:
  CreateMTCGeneratePlacePose(const std::string& name, const BT::NodeConfig& config);

  BT::NodeStatus tick() override;
  static BT::PortsList providedPorts();
};
}  // namespace bt_mtc
