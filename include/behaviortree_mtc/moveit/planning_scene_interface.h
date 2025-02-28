#pragma once

#include <behaviortree_cpp/bt_factory.h>

namespace BT {
namespace MTC {

class PlanningSceneInterface : public BT::SyncActionNode
{
public:
  PlanningSceneInterface(const std::string& name, const BT::NodeConfig& config);

  BT::NodeStatus tick() override;
  static BT::PortsList providedPorts();
};

class PlanningSceneInterfaceAddCollisionObject : public BT::SyncActionNode
{
public:
  PlanningSceneInterfaceAddCollisionObject(const std::string& name, const BT::NodeConfig& config);

  BT::NodeStatus tick() override;
  static BT::PortsList providedPorts();
};


}  // namespace MTC
}  // namespace BT
