#pragma once

#include <behaviortree_cpp/bt_factory.h>

class MoveMTCStageToContainer : public BT::SyncActionNode
{
public:
  MoveMTCStageToContainer(const std::string& name, const BT::NodeConfig& config);

  BT::NodeStatus tick() override;
  static BT::PortsList providedPorts();
};
