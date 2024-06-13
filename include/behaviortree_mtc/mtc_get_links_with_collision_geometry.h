#pragma once

#include <behaviortree_cpp/bt_factory.h>

namespace bt_mtc
{
class MTCGetLinksWithCollisionGeometry : public BT::SyncActionNode
{
public:
  MTCGetLinksWithCollisionGeometry(const std::string& name, const BT::NodeConfig& config);

  BT::NodeStatus tick() override;
  static BT::PortsList providedPorts();
};
}  // namespace bt_mtc
