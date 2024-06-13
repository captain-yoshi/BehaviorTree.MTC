#pragma once

#include <behaviortree_cpp/bt_factory.h>

namespace bt_mtc
{
class MTCAllowCollisionsOneToMany : public BT::SyncActionNode
{
public:
  MTCAllowCollisionsOneToMany(const std::string& name, const BT::NodeConfig& config);

  BT::NodeStatus tick() override;
  static BT::PortsList providedPorts();
};

class MTCAllowCollisionsOnOneObject : public BT::SyncActionNode
{
public:
  MTCAllowCollisionsOnOneObject(const std::string& name, const BT::NodeConfig& config);

  BT::NodeStatus tick() override;
  static BT::PortsList providedPorts();
};
}  // namespace bt_mtc
