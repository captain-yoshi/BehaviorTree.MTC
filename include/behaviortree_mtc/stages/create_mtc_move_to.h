#pragma once

#include <behaviortree_cpp/bt_factory.h>

#include <moveit/task_constructor/stages/move_to.h>

namespace BT {
namespace MTC {

class CreateMTCMoveToBase : public BT::SyncActionNode
{
public:
  CreateMTCMoveToBase(const std::string& name, const BT::NodeConfig& config);
  BT::NodeStatus tick(std::shared_ptr<moveit::task_constructor::stages::MoveTo>& stage);
  static BT::PortsList providedPorts();
};

class CreateMTCMoveToBaseCartesian : public CreateMTCMoveToBase
{
public:
  CreateMTCMoveToBaseCartesian(const std::string& name, const BT::NodeConfig& config);

  BT::NodeStatus tick(std::shared_ptr<moveit::task_constructor::stages::MoveTo>& stage);
  static BT::PortsList providedPorts();
};

class CreateMTCMoveToNamedJointPose : public CreateMTCMoveToBase
{
public:
  CreateMTCMoveToNamedJointPose(const std::string& name, const BT::NodeConfig& config);

  BT::NodeStatus tick() override;
  static BT::PortsList providedPorts();
};

class CreateMTCMoveToJoint : public CreateMTCMoveToBase
{
public:
  CreateMTCMoveToJoint(const std::string& name, const BT::NodeConfig& config);

  BT::NodeStatus tick() override;
  static BT::PortsList providedPorts();
};

class CreateMTCMoveToPose : public CreateMTCMoveToBaseCartesian
{
public:
  CreateMTCMoveToPose(const std::string& name, const BT::NodeConfig& config);

  BT::NodeStatus tick() override;
  static BT::PortsList providedPorts();
};

class CreateMTCMoveToPoint : public CreateMTCMoveToBaseCartesian
{
public:
  CreateMTCMoveToPoint(const std::string& name, const BT::NodeConfig& config);

  BT::NodeStatus tick() override;
  static BT::PortsList providedPorts();
};

}  // namespace MTC
}  // namespace BT
