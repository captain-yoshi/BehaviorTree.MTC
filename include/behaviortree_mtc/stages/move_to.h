#pragma once

#include <behaviortree_cpp/bt_factory.h>

#include <moveit/task_constructor/stages/move_to.h>

namespace BT {
namespace MTC {

class MoveToBase : public BT::SyncActionNode
{
public:
  MoveToBase(const std::string& name, const BT::NodeConfig& config);
  BT::NodeStatus tick(std::shared_ptr<moveit::task_constructor::stages::MoveTo>& stage);
  static BT::PortsList providedPorts();
};

class MoveToBaseCartesian : public MoveToBase
{
public:
  MoveToBaseCartesian(const std::string& name, const BT::NodeConfig& config);

  BT::NodeStatus tick(std::shared_ptr<moveit::task_constructor::stages::MoveTo>& stage);
  static BT::PortsList providedPorts();
};

class MoveToNamedJointPose : public MoveToBase
{
public:
  MoveToNamedJointPose(const std::string& name, const BT::NodeConfig& config);

  BT::NodeStatus tick() override;
  static BT::PortsList providedPorts();
};

class MoveToJoint : public MoveToBase
{
public:
  MoveToJoint(const std::string& name, const BT::NodeConfig& config);

  BT::NodeStatus tick() override;
  static BT::PortsList providedPorts();
};

class MoveToPose : public MoveToBaseCartesian
{
public:
  MoveToPose(const std::string& name, const BT::NodeConfig& config);

  BT::NodeStatus tick() override;
  static BT::PortsList providedPorts();
};

class MoveToPoint : public MoveToBaseCartesian
{
public:
  MoveToPoint(const std::string& name, const BT::NodeConfig& config);

  BT::NodeStatus tick() override;
  static BT::PortsList providedPorts();
};

}  // namespace MTC
}  // namespace BT
