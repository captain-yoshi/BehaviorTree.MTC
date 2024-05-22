#pragma once

#include <behaviortree_cpp/bt_factory.h>

class PlanMTCTask : public BT::ThreadedAction
{
public:
  PlanMTCTask(const std::string& name, const BT::NodeConfig& config);

  BT::NodeStatus tick() override;
  static BT::PortsList providedPorts();
};
