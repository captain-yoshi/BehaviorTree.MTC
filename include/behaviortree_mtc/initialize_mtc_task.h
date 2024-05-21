#pragma once

#include <behaviortree_cpp/bt_factory.h>

class InitializeMTCTask : public BT::SyncActionNode
{
public:
  InitializeMTCTask(const std::string& name, const BT::NodeConfig& config);

  BT::NodeStatus tick() override;
  static BT::PortsList providedPorts();
};
