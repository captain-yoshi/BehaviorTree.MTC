#pragma once

#include <behaviortree_cpp/bt_factory.h>

#include <moveit/task_constructor/stages/move_relative.h>

namespace BT {
namespace MTC {

class CreateMTCMoveRelativeBase : public BT::SyncActionNode
{
public:
  CreateMTCMoveRelativeBase(const std::string& name, const BT::NodeConfig& config);

  BT::NodeStatus tick(std::shared_ptr<moveit::task_constructor::stages::MoveRelative>& stage);
  static BT::PortsList providedPorts();
};

class CreateMTCMoveRelativeTwist : public CreateMTCMoveRelativeBase
{
public:
  CreateMTCMoveRelativeTwist(const std::string& name, const BT::NodeConfig& config);

  BT::NodeStatus tick() override;
  static BT::PortsList providedPorts();
};

class CreateMTCMoveRelativeTranslate : public CreateMTCMoveRelativeBase
{
public:
  CreateMTCMoveRelativeTranslate(const std::string& name, const BT::NodeConfig& config);

  BT::NodeStatus tick() override;
  static BT::PortsList providedPorts();
};

class CreateMTCMoveRelativeJoint : public CreateMTCMoveRelativeBase
{
public:
  CreateMTCMoveRelativeJoint(const std::string& name, const BT::NodeConfig& config);

  BT::NodeStatus tick() override;
  static BT::PortsList providedPorts();
};

}  // namespace MTC
}  // namespace BT
