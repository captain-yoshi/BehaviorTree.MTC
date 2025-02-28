#pragma once

#include <behaviortree_cpp/bt_factory.h>

#include <moveit/task_constructor/stages/move_relative.h>

namespace BT {
namespace MTC {

class MoveRelativeBase : public BT::SyncActionNode
{
public:
  MoveRelativeBase(const std::string& name, const BT::NodeConfig& config);

  BT::NodeStatus tick(std::shared_ptr<moveit::task_constructor::stages::MoveRelative>& stage);
  static BT::PortsList providedPorts();
};

class MoveRelativeTwist : public MoveRelativeBase
{
public:
  MoveRelativeTwist(const std::string& name, const BT::NodeConfig& config);

  BT::NodeStatus tick() override;
  static BT::PortsList providedPorts();
};

class MoveRelativeTranslate : public MoveRelativeBase
{
public:
  MoveRelativeTranslate(const std::string& name, const BT::NodeConfig& config);

  BT::NodeStatus tick() override;
  static BT::PortsList providedPorts();
};

class MoveRelativeJoint : public MoveRelativeBase
{
public:
  MoveRelativeJoint(const std::string& name, const BT::NodeConfig& config);

  BT::NodeStatus tick() override;
  static BT::PortsList providedPorts();
};

}  // namespace MTC
}  // namespace BT
