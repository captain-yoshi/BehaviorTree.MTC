#pragma once

#include <behaviortree_cpp/bt_factory.h>

namespace bt_mtc
{
class GeometryMsgsPoseStamped : public BT::SyncActionNode
{
public:
  GeometryMsgsPoseStamped(const std::string& name, const BT::NodeConfig& config);

  BT::NodeStatus tick() override;
  static BT::PortsList providedPorts();
};

class GeometryMsgsVector3Stamped : public BT::SyncActionNode
{
public:
  GeometryMsgsVector3Stamped(const std::string& name, const BT::NodeConfig& config);

  BT::NodeStatus tick() override;
  static BT::PortsList providedPorts();
};

class GeometryMsgsTwistStamped : public BT::SyncActionNode
{
public:
  GeometryMsgsTwistStamped(const std::string& name, const BT::NodeConfig& config);

  BT::NodeStatus tick() override;
  static BT::PortsList providedPorts();
};
}  // namespace bt_mtc
